#pragma once
#include "WhiteFleetKickoffThroughV8Math.h"

/** Team-field coordinates: +X always attacks. Never camera-left/body-forward.
 * This is candidate geometry, not evidence that a pass/keeper race is safe.
 * Every opponent (including a keeper) still goes through the original reach
 * gates. No private opponent identity, heading or simulator truth is used. */
namespace WhiteFleetC9ForwardFlank
{
  namespace K=WhiteFleetKickoffThroughV8;
  inline bool directed(K::Point ball,K::Point target,int side)
  {
    return K::finite(ball)&&K::finite(target)&&(side==1||side==-1)
        &&target.x-ball.x>250.f&&target.y*side>0.f&&(target.y-ball.y)*side>250.f;
  }
  inline bool receiverSide(K::Point ball,K::Point receiver,int side)
  {return K::finite(ball)&&K::finite(receiver)&&(receiver.y-ball.y)*side>250.f;}
  inline bool directed(K::Point ball,K::Point target,int side,const K::Field& field,const K::Params& p,float wireTolerance=0.f)
  {
    if(!p.directCenter)return directed(ball,target,side);
    return side==-1&&K::finite(ball)&&target.x-ball.x>250.f&&K::targetSafe(ball,target,field,p,side,wireTolerance);
  }

  inline bool behindLine(K::Point ball,K::Point target,const K::Field& field,
                         const K::Params& params,const std::vector<K::Obstacle>& obstacles)
  {
    if(!K::fieldValid(field)||!K::paramsValid(params)||!K::finite(ball)||!K::finite(target)
       ||target.x-ball.x<=250.f)return false;
    for(const auto& o:obstacles)
    {
      if(!K::finite(o.center)||!K::finite(o.left)||!K::finite(o.right)
         ||!std::isfinite(o.radius)||o.radius<0.f)return false;
      if(o.teammate||!o.mobile||o.center.x<=ball.x+250.f||o.center.x>=field.opponentPenalty)continue;
      // Relevant defence is a local strip along this ray, extended up to the
      // penalty front. Not the maximum X of all opponents or an identified GK.
      const float rayY=ball.y+(o.center.x-ball.x)*(target.y-ball.y)/(target.x-ball.x);
      const float extent=std::max(std::abs(o.left.y-o.center.y),std::abs(o.right.y-o.center.y));
      if(std::abs(o.center.y-rayY)>1000.f+extent+o.radius+params.obstaclePadding)continue;
      const float front=std::max({o.center.x,o.left.x,o.right.x})+o.radius;
      if(target.x<front+300.f)return false;
    }
    return true;
  }
  inline bool valid(K::Point ball,K::Point receiver,K::Point target,const K::Field& field,
                    const K::Params& params,int side,const std::vector<K::Obstacle>& obstacles)
  {
    return directed(ball,target,side,field,params)&&receiverSide(ball,receiver,side)
        &&K::targetSafe(ball,target,field,params,side)&&behindLine(ball,target,field,params,obstacles);
  }
  inline bool preferred(K::Point target,const K::Field& field)
  {const float gap=field.opponentPenalty-target.x;return gap>=500.f&&gap<=1000.f;}

  inline K::Point stopPoint(K::Point ball,K::Point target,const K::FlightProfile& flight)
  {
    if(flight.calibrated)return flight.stopPoint;
    const float length=K::distance(ball,target);
    if(!std::isfinite(length)||length<=0.f)return {NAN,NAN};
    return {ball.x+(target.x-ball.x)*flight.stopDistance/length,
            ball.y+(target.y-ball.y)*flight.stopDistance/length};
  }
  inline bool receptionSafe(K::Point ball,K::Point target,const K::FlightProfile& flight,
                            const K::Field& field,const K::Params& params,int side)
  {
    // A nominal P outside the box is not enough: a positive rollout reserve
    // must not intentionally predict a stop inside the penalty area or out.
    const auto stop=stopPoint(ball,target,flight);
    return flight.valid&&directed(ball,stop,side,field,params)&&K::targetSafe(ball,stop,field,params,side);
  }

  inline std::vector<K::Point> candidates(const K::Field& field,const K::Params& params,int side)
  {
    if(params.directCenter)
    {
      if(params.centerFast)
      {
        std::vector<K::Point> points;
        for(float x:{-250.f,0.f,250.f,500.f})for(float y:{-500.f,-750.f,-1000.f,-1250.f,-1500.f,-1750.f,-2000.f})
          points.push_back({field.opponentPenalty+x,y});
        return points;
      }
      return K::directCenterDomain(K::directCenterTarget(field),field,params)?
          std::vector<K::Point>{K::directCenterTarget(field)}:std::vector<K::Point>{};
    }
    auto result=K::targetCandidates(field,params,side);
    if(!params.playbook||!K::fieldValid(field)||!K::paramsValid(params)||(side!=1&&side!=-1))return result;
    // Existing points remain regression candidates, never another kick path.
    // Preferred new points scale with the real penalty front, not 22x14 literals.
    for(float gap:{500.f,750.f,1000.f})for(float y:{.44f,.52f,.64f,.76f})
      result.push_back({field.opponentPenalty-gap,side*field.halfWidth*y});
    // Shared finite table: both planner and receiver wire-contract membership
    // must know the same experiment points. Do NOT bypass region validation.
    if(params.nativeReceive&&params.isolated&&side==-1&&(params.c12RightMinDegrees>0.f||params.c14EarlyStart))
      for(float y:{.64f,.76f})result.push_back({field.opponentGoal*.30f,-field.halfWidth*y});
    if(params.nativeReceive&&params.isolated&&side==-1&&params.c14EarlyStart)
      for(float x:{.34f,.38f,.42f})for(float y:{.64f,.70f})
        result.push_back({field.opponentGoal*x,-field.halfWidth*y});
    if(params.nativeReceive&&params.isolated&&side==-1&&params.c14EarlyStart&&params.c15DeepPass)
      for(float gap:{500.f,750.f,1000.f})for(float y:{.80f,.84f,.88f})
        result.push_back({field.opponentPenalty-gap,-field.halfWidth*y});
    return result;
  }
  inline float runHeading(K::Point from,K::Point target,int side,const K::Params& params)
  {return std::atan2(target.y-from.y,target.x-from.x)-side*params.headBodyYaw;}
  inline K::Point prestartPoint(K::Point prepare,K::Point target)
  {
    if(!K::finite(prepare)||!K::finite(target)||target.x<=prepare.x)return prepare;
    const float x=std::min(-350.f,prepare.x+300.f);
    if(x<=prepare.x)return prepare;
    return {x,prepare.y+(target.y-prepare.y)*(x-prepare.x)/(target.x-prepare.x)};
  }
}
