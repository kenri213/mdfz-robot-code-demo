#pragma once

#include "WhiteFleetC9ForwardFlank.h"

/** Only proposal geometry. Every proposal still passes the original planner's
 * flight, swept-outline, native-risk and capture/ETA checks. No obstacle or
 * motion truth is created here. Team field +X attacks, negative Y is this
 * profile's receiver flank; camera orientation never enters the calculation.
 *
 * New endpoints are on an EXISTING table X station, at most 300mm laterally
 * from a table point. The receiver's independently verifiable closed domain
 * allows the existing 300mm wire tolerance, still clipped to legal field and
 * non-goal-directed geometry. It does not have to see the sender's obstacles.
 * This domain is NOT a pass/capture certificate or an invitation to replan an
 * already offered/confirmed endpoint. No wire fields or ACKs are added. */
namespace WhiteFleetC15DefenceAdaptive
{
  namespace K=WhiteFleetKickoffThroughV8;
  constexpr float generatedShift=300.f,wireTolerance=300.f;
  constexpr std::size_t maxAngles=24,maxExtraTargets=64;

  inline bool enabled(const K::Params& p,int side)
  {return p.defenceAdaptiveAngle&&p.restoreProvenPool&&p.c15DeepPass&&p.c14EarlyStart&&
      p.nativeReceive&&p.isolated&&p.playbook&&p.calibrated&&p.adaptiveMeet&&p.side==-1&&side==-1;}

  inline std::vector<K::Point> original(const K::Field& field,const K::Params& p,int side)
  {auto old=p;old.raceRendezvous=false;old.prepareYFraction=0.f;old.insideCornerLane=false;old.defenceAdaptiveAngle=false;return WhiteFleetC9ForwardFlank::candidates(field,old,side);}

  inline bool insideEnabled(const K::Field& field,const K::Params& p,int side)
  {
    return enabled(p,side)&&p.insideCornerLane&&std::isfinite(field.penaltyHalfWidth)&&
        field.penaltyHalfWidth>field.goalHalfWidth+300.f&&field.penaltyHalfWidth<field.halfWidth-p.fieldMargin;
  }

  /** A second, explicitly configured local domain, not a global angle waiver.
   * +X still attacks, Y remains on the SAME receiver flank. Unknown penalty
   * width never guesses from the goal width, field width or camera view. */
  inline bool insideDomain(K::Point ball,K::Point target,const K::Field& field,const K::Params& p,int side,float tolerance=0.f)
  {
    return insideEnabled(field,p,side)&&K::paramsValid(p)&&K::fieldValid(field)&&K::finite(ball)&&K::finite(target)&&
        std::isfinite(tolerance)&&tolerance>=0.f&&tolerance<=wireTolerance&&
        target.x>=field.opponentPenalty-1000.f-tolerance&&target.x<=field.opponentPenalty-500.f+tolerance&&
        std::abs(target.y+field.penaltyHalfWidth)<=250.f+tolerance&&
        K::targetSafe(ball,target,field,p,side)&&WhiteFleetC9ForwardFlank::directed(ball,target,side);
  }

  inline bool belowDefenderTrigger(K::Point ball,K::Point target,const K::Field& field,const K::Params& p,
                                  const std::vector<K::Obstacle>& obstacles,float ballRadius)
  {
    if(!insideDomain(ball,target,field,p,-1)||!std::isfinite(ballRadius)||ballRadius<=0.f)return false;
    for(const auto& o:obstacles)
      if(!K::finite(o.center)||!K::finite(o.left)||!K::finite(o.right)||!std::isfinite(o.radius)||o.radius<0.f||
         !std::isfinite(o.speedBound)||o.speedBound<0.f)return false;
    for(const auto& o:obstacles)
    {
      if(!o.angleSource||!o.mobile||o.teammate||o.center.x<=ball.x+250.f||
         target.x<std::max({o.center.x,o.left.x,o.right.x})+o.radius+300.f)continue;
      // Negative cross is below the forward ray, i.e. lower in the picture.
      // Require the WHOLE fresh contour below, not just a convenient centre.
      if(K::cross(ball,target,o.center)>=0.f||K::cross(ball,target,o.left)>=0.f||K::cross(ball,target,o.right)>=0.f)continue;
      const float clearance=K::obstacleDistance(ball,target,o)-o.radius-ballRadius-p.obstaclePadding-
          o.speedBound*p.opponentReleaseAdvantage;
      if(clearance>0.f)return true;
    }
    return false;
  }

  inline std::vector<K::Point> insideCandidates(K::Point ball,const K::Field& field,const K::Params& p,int side,
                                               const std::vector<K::Obstacle>& obstacles,float ballRadius)
  {
    std::vector<K::Point> out;
    if(!insideEnabled(field,p,side))return out;
    for(float gap:{500.f,750.f,1000.f})for(float offset:{0.f,250.f,-250.f})
    {
      const K::Point target{field.opponentPenalty-gap,-field.penaltyHalfWidth+offset};
      if(belowDefenderTrigger(ball,target,field,p,obstacles,ballRadius))out.push_back(target);
    }
    return out; // At most nine extra plans, still evaluated against ALL obstacles.
  }

  inline bool tableTarget(K::Point target,const K::Field& field,const K::Params& p,int side)
  {
    for(auto anchor:original(field,p,side))if(anchor.x==target.x&&anchor.y==target.y)return true;
    return false;
  }

  inline bool domain(K::Point ball,K::Point target,const K::Field& field,const K::Params& p,int side,float tolerance=0.f)
  {
    if(insideDomain(ball,target,field,p,side,tolerance))return true;
    if(!enabled(p,side)||!K::fieldValid(field)||!K::paramsValid(p)||!K::finite(ball)||!K::finite(target)||
       !std::isfinite(tolerance)||tolerance<0.f||tolerance>wireTolerance||
       !K::targetSafe(ball,target,field,p,side)||!WhiteFleetC9ForwardFlank::directed(ball,target,side))return false;
    const float angle=-std::atan2(target.y-ball.y,target.x-ball.x)*180.f/3.14159265358979323846f;
    if(!std::isfinite(angle)||angle<p.c12RightMinDegrees)return false;
    for(auto anchor:original(field,p,side))
      if(std::abs(target.x-anchor.x)<=tolerance+.01f&&
         std::abs(target.y-anchor.y)<=generatedShift+tolerance+.01f)return true;
    return false;
  }

  inline std::vector<K::Point> candidates(K::Point ball,const K::Field& field,const K::Params& p,int side,
                                         const std::vector<K::Obstacle>& obstacles,float ballRadius)
  {
    std::vector<K::Point> result;
    if(!enabled(p,side)||!K::fieldValid(field)||!K::paramsValid(p)||!K::finite(ball)||
       !std::isfinite(ballRadius)||ballRadius<=0.f)return result;
    // Invalid input is never an empty pitch. All contours (even those not used
    // to generate a ray) still participate in the downstream original gates.
    for(const auto& o:obstacles)
      if(!K::finite(o.center)||!K::finite(o.left)||!K::finite(o.right)||!std::isfinite(o.radius)||o.radius<0.f||
         !std::isfinite(o.speedBound)||o.speedBound<0.f)return result;
    std::vector<K::Obstacle> sources;
    for(const auto& o:obstacles)
      if(o.angleSource&&!o.teammate&&o.mobile&&o.center.x>ball.x+250.f&&o.center.x<field.opponentPenalty)sources.push_back(o);
    std::sort(sources.begin(),sources.end(),[&](const auto& a,const auto& b)
    {
      const float da=K::distance(ball,a.center),db=K::distance(ball,b.center);
      if(da!=db)return da<db;
      if(a.center.x!=b.center.x)return a.center.x<b.center.x;
      return a.center.y<b.center.y;
    });
    // Generation work is bounded; this is NOT a limit on the safety evaluator's
    // obstacle list. Taking the nearest eight only changes available proposals.
    std::vector<float> angles;
    for(std::size_t i=0;i<std::min<std::size_t>(sources.size(),8);++i)
    {
      const auto& o=sources[i];
      const float radius=o.radius+ballRadius+p.obstaclePadding+o.speedBound*p.opponentReleaseAdvantage+25.f;
      float low=INFINITY,high=-INFINITY;bool outside=true;
      for(auto vertex:{o.center,o.left,o.right})
      {
        const float d=K::distance(ball,vertex);
        if(d<=radius){outside=false;break;}
        const float a=std::atan2(vertex.y-ball.y,vertex.x-ball.x),delta=std::asin(radius/d);
        low=std::min(low,a-delta);high=std::max(high,a+delta);
      }
      if(outside)
        for(float a:{low,high})
          if(std::isfinite(a)&&a<-.05f&&a>-1.50f)angles.push_back(a);
    }
    std::sort(angles.begin(),angles.end());
    angles.erase(std::unique(angles.begin(),angles.end(),[](float a,float b){return std::abs(a-b)<1e-5f;}),angles.end());
    const auto edges=angles;
    for(std::size_t i=1;i<edges.size()&&angles.size()<maxAngles;++i)angles.push_back((edges[i-1]+edges[i])*.5f);
    const auto anchors=original(field,p,side);
    // Same angle at different existing stations is re-evaluated for its own
    // distance, native normal mapping, capture time and receiver run line.
    for(float a:angles)for(auto anchor:anchors)
    {
      const K::Point target{anchor.x,ball.y+(anchor.x-ball.x)*std::tan(a)};
      if(!domain(ball,target,field,p,side))continue;
      bool duplicate=false;
      for(auto old:anchors)duplicate|=K::distance(old,target)<1.f;
      for(auto old:result)duplicate|=K::distance(old,target)<1.f;
      if(duplicate)continue;
      result.push_back(target);
      if(result.size()>=maxExtraTargets)return result;
    }
    return result;
  }
}
