#pragma once
#include "WhiteFleetKickoffThroughV8Planner.h"
#include "WhiteFleetCloseCoverMath.h"
#include "WhiteFleetC9PassPlaybookGate.h"
#include "WhiteFleetC9ForwardFlank.h"

/** Finite proposals, never an owner election or a replacement motion planner.
 * Native callbacks are supplied by Strategy's actual providers. Hard parent
 * geometry/ETA/opponent gates run BEFORE scoring; missing ratings reject. */
namespace WhiteFleetC9PassPlaybook
{
  namespace K=WhiteFleetKickoffThroughV8;
  struct Ratings {float pass=0.f,goal=0.f;};
  inline bool ratingsValid(Ratings r)
  {return std::isfinite(r.pass)&&std::isfinite(r.goal)&&r.pass>=.5f&&r.pass<=1.f&&r.goal>=0.f&&r.goal<=1.f;}
  inline float score(Ratings r,float arrivalMargin,float reserve,float uncertainty)
  {
    return .55f*r.pass+.35f*r.goal+.10f*std::clamp(arrivalMargin,0.f,1.f)
        -.01f*reserve/800.f-.10f*std::clamp(uncertainty,0.f,1.f);
  }
  template<class NativeRating>
  K::Plan kickoff(K::Point ball,K::Point receiver,float heading,const K::Field& field,
                  const std::vector<K::Obstacle>& obstacles,const K::Params& params,
                  const KickInfo& info,const KickLengthPair& pairs,const BallSpecification& specs,
                  const WalkingEngineOutput& walking,NativeRating nativeRating,float uncertainty,K::CandidateAudit* audit=nullptr)
  {
    K::Plan best;float bestScore=-std::numeric_limits<float>::infinity();bool bestPreferred=false;
    if(!params.playbook||!K::paramsValid(params)||!K::fieldValid(field)||!K::finite(receiver)||!K::finite(ball))return best;
    const int side=K::effectiveSide(params,receiver);
    for(K::Point target:WhiteFleetC9ForwardFlank::candidates(field,params,side))for(float reserve:{0.f,200.f,500.f,800.f})
    {
      const float nominal=params.calibrated?K::calibratedNominalForStop(K::distance(ball,target)+reserve,params,info,specs):
          K::distance(ball,target)+reserve;
      const auto flight=K::mappedFlight(ball,target,nominal,info,pairs,specs,params);
      auto plan=K::evaluate(ball,receiver,target,field,obstacles,params,side,flight,
          walking.maxSpeed.translation.x(),walking.maxSpeed.translation.y(),walking.maxSpeed.rotation,specs.radius,heading);
      if(plan.valid&&!WhiteFleetC9ForwardFlank::valid(ball,receiver,target,field,params,side,obstacles))
      {plan.valid=false;plan.reason="not_forward_teammate_flank_behind_line";}
      if(plan.valid&&!WhiteFleetC9ForwardFlank::receptionSafe(ball,target,flight,field,params,side))
      {plan.valid=false;plan.reason="rollout_outside_safe_reception_geometry";}
      bool ranked=false;Ratings native;float value=0.f;
      if(plan.valid)
      {
        native=nativeRating(target);ranked=true;
        if(!ratingsValid(native)){plan.valid=false;plan.reason="native_rating_unavailable_or_unsafe";}
        else
        {
          value=score(native,plan.safetyMargin,reserve,uncertainty);
          const bool preferred=WhiteFleetC9ForwardFlank::preferred(target,field)
              &&WhiteFleetC9ForwardFlank::preferred(WhiteFleetC9ForwardFlank::stopPoint(ball,target,flight),field);
          if(!best.valid||(preferred&&!bestPreferred)||(preferred==bestPreferred&&value>bestScore))
          {best=plan;bestScore=value;bestPreferred=preferred;}
        }
      }
      if(audit)
      {
        audit->observe(target,reserve,flight,plan);
        auto& record=audit->records.back();record.ranked=ranked;record.nativePass=native.pass;
        record.nativeGoal=native.goal;record.score=value;record.uncertainty=uncertainty;
      }
      if(!plan.valid&&!best.valid)best=plan;
    }
    return best;
  }

  /** Front throw-in/low corner rendezvous. Receiver must arrive BEFORE the
   * original native pass readiness gate; ETA is a GC preparation deadline,
   * NOT a fabricated flight time for the native selected kick type. */
  template<class NativeRating>
  bool close(WhiteFleetCloseCoverMath::RestartPlan& plan,unsigned now,unsigned epoch,int kind,int kicker,int receiver,
             K::Point ball,K::Point here,float heading,const WhiteFleetCloseCoverMath::Field& field,
             const std::vector<K::Point>& opponents,unsigned timeLeft,bool corner,const K::Params& params,
             const WalkingEngineOutput& walking,NativeRating nativeRating,bool preserve)
  {
    const auto valid=[&](K::Point target,float& eta,float& value)
    {
      const WhiteFleetCloseCoverMath::Point b{ball.x,ball.y},p{target.x,target.y},r{here.x,here.y};
      std::vector<WhiteFleetCloseCoverMath::Point> blockers;
      for(auto o:opponents)blockers.push_back({o.x,o.y});
      if(!WhiteFleetCloseCoverMath::closeTargetValid(p,b,r,field,blockers)||(!corner&&target.x<ball.x+250.f))return false;
      eta=K::empiricalReceiverTime(here,target,heading,walking.maxSpeed.translation.x(),
          walking.maxSpeed.translation.y(),walking.maxSpeed.rotation,params,false);
      if(!std::isfinite(eta)||eta*1000.f+2000.f>timeLeft)return false;
      const Ratings rating=nativeRating(target);if(!ratingsValid(rating))return false;
      value=.6f*rating.pass+.4f*rating.goal-.02f*eta;return true;
    };
    if(!WhiteFleetCloseCoverMath::fieldValid(field)||!K::finite(ball)||!K::finite(here)||ball.x<0.f
       ||std::abs(ball.y)>field.halfWidth+250.f||!epoch||now<epoch||timeLeft<2000||kicker<=0||receiver<=0||kicker==receiver)
      return false;
    for(auto opponent:opponents)if(!K::finite(opponent))return false;
    if(preserve)
    {
      float eta=0.f,value=0.f;
      return plan.valid&&plan.epoch==epoch&&plan.kind==kind&&plan.kicker==kicker&&plan.receiver==receiver
          &&valid({plan.target.x,plan.target.y},eta,value);
    }
    float best=-std::numeric_limits<float>::infinity();K::Point chosen{};bool found=false;
    const float theta=std::atan2(-ball.y,field.opponentGoal-ball.x),side=ball.y<0.f?-1.f:1.f;
    for(float radius:{2500.f,3250.f,4000.f})for(float offset:{0.f,.26f,-.26f,.52f,-.52f,.78f,-.78f,1.04f,-1.04f})
    {
      const float angle=theta+side*offset;
      K::Point target{ball.x+radius*std::cos(angle),ball.y+radius*std::sin(angle)};
      float eta=0.f,value=0.f;
      if(valid(target,eta,value)&&value>best){best=value;chosen=target;found=true;}
    }
    if(!found)return false;
    plan={true,epoch,now,kind,kicker,receiver,{chosen.x,chosen.y},{ball.x,ball.y}};return true;
  }

}
