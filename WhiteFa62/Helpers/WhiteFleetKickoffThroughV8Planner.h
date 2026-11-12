#pragma once

#include "WhiteFleetKickoffThroughV8Math.h"
#include "WhiteFleetKickoffThroughV8CandidateAudit.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Configuration/KickLengthPair.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/Motion/KickLengthConverter.h"

namespace WhiteFleetKickoffThroughV8
{
  inline BallSpecification predictionBall(const Params& p,const BallSpecification& actual)
  {
    auto result=actual;
    if(p.centerFast)result.staticFriction=-p.predictionDeceleration*.001f;
    return result; // Local value for forecasting ONLY; mapping always uses actual.
  }
  inline float calibratedTransitionDistance(float command,const Params& p)
  {return .5f*(p.exitRatio+p.rollRatio)*command*p.transitionSeconds;}
  inline float calibratedStopDistance(float command,const Params& p,const BallSpecification& specs)
  {
    if(!paramsValid(p)||!std::isfinite(command)||command<=0.f||!std::isfinite(specs.staticFriction)||specs.staticFriction>=0.f)
      return std::numeric_limits<float>::quiet_NaN();
    return calibratedTransitionDistance(command,p)
        +BallPhysics::getEndPosition(Vector2f::Zero(),Vector2f(command*p.rollRatio,0.f),predictionBall(p,specs)).norm();
  }
  inline float calibratedTimeForDistance(float command,float distance,const Params& p,const BallSpecification& specs)
  {
    if(!paramsValid(p)||!std::isfinite(command)||command<=0.f||!std::isfinite(distance)||distance<0.f)
      return std::numeric_limits<float>::infinity();
    if(distance==0.f)return 0.f;
    const float transition=calibratedTransitionDistance(command,p);
    if(distance<=transition)
    {
      const float v0=command*p.exitRatio,a=command*(p.rollRatio-p.exitRatio)/p.transitionSeconds;
      const float discriminant=v0*v0+2.f*a*distance;
      if(discriminant<0.f)return std::numeric_limits<float>::infinity();
      return 2.f*distance/(v0+std::sqrt(discriminant));
    }
    return p.transitionSeconds+BallPhysics::timeForDistance(Vector2f(command*p.rollRatio,0.f),distance-transition,predictionBall(p,specs));
  }
  /** Invert only this scope's empirical rollout model. The resulting nominal
   * length still goes through the UNMODIFIED normal converter/network cap. */
  inline float calibratedNominalForStop(float wanted,const Params& p,const KickInfo& info,const BallSpecification& specs)
  {
    const auto& kick=info[KickInfo::rlKickDynamic];
    if(!paramsValid(p)||!std::isfinite(wanted)||wanted<=0.f||!std::isfinite(specs.staticFriction)||specs.staticFriction>=0.f
       ||!std::isfinite(kick.ballVelocity.min)
       ||!std::isfinite(kick.ballVelocity.max)||kick.ballVelocity.min<=0.f||kick.ballVelocity.max<=kick.ballVelocity.min)
      return std::numeric_limits<float>::quiet_NaN();
    float low=kick.ballVelocity.min,high=kick.ballVelocity.max;
    // A tiny numerical reserve avoids a max-float ETA at the exact stop root;
    // it is not a new receiving region or a claimed physical range margin.
    wanted+=5.f;
    if(calibratedStopDistance(high,p,specs)<wanted)return std::numeric_limits<float>::quiet_NaN();
    for(unsigned i=0;i<32;++i)
    {
      const float middle=.5f*(low+high);
      if(calibratedStopDistance(middle,p,specs)<wanted)low=middle;else high=middle;
    }
    const float nominal=BallPhysics::getEndPosition(Vector2f::Zero(),Vector2f(high,0.f),specs).norm();
    return nominal>=kick.range.min&&nominal<=kick.range.max?nominal:std::numeric_limits<float>::quiet_NaN();
  }
  /** Mirrors the normal (not soft/strong) WhiteFleetWalkingEngine mapping. Runtime
   * KickInfo range is already derived by ConfigurationDataProvider from its
   * ballVelocity and BallSpecification, not the literal range in kickInfo.cfg.
   * KickInfo's cap matches the protected SimulatedK1 normal ball-network cap;
   * the actual applied network value still requires Motion-thread evidence. */
  inline FlightProfile mappedFlight(Point ball,Point target,float nominal,const KickInfo& kickInfo,
                                    const KickLengthPair& pairs,const BallSpecification& ballSpecs,const Params& params)
  {
    FlightProfile result;result.nominalLength=nominal;
    const auto& kick=kickInfo[KickInfo::rlKickDynamic];
    if(!paramsValid(params)||!finite(ball)||!finite(target)||!std::isfinite(nominal)||nominal<=0.f
       ||!std::isfinite(ballSpecs.staticFriction)||ballSpecs.staticFriction>=0.f
       ||!std::isfinite(kick.range.min)||!std::isfinite(kick.range.max)||kick.range.min<=0.f||kick.range.max<=kick.range.min
       ||!std::isfinite(kick.ballVelocity.min)||!std::isfinite(kick.ballVelocity.max)
       ||kick.ballVelocity.min<=0.f||kick.ballVelocity.max<kick.ballVelocity.min
       ||nominal<kick.range.min||nominal>kick.range.max)return result;
    const float length=distance(ball,target);
    if(!std::isfinite(length)||length<=0.f)return result;
    const float power=KickLengthConverter::kickLengthToPower(KickInfo::rlKickDynamic,nominal,kickInfo,pairs);
    if(!std::isfinite(power)||power<0.f||power>1.f)return result;
    result.mappedLength=std::max(1.f,kick.range.min+(kick.range.max-kick.range.min)*power);
    result.initialSpeed=kick.ballVelocity.limit(BallPhysics::velocityForDistance(result.mappedLength,ballSpecs));
    if(!std::isfinite(result.initialSpeed)||result.initialSpeed<=0.f)return result;
    const Vector2f velocity(result.initialSpeed,0.f);
    result.stopDistance=params.calibrated?calibratedStopDistance(result.initialSpeed,params,ballSpecs):
        BallPhysics::getEndPosition(Vector2f::Zero(),velocity,ballSpecs).norm();
    if(!std::isfinite(result.stopDistance)||result.stopDistance<length)
    {result.reason="range_saturated_short";return result;}
    result.ballEta=params.calibrated?calibratedTimeForDistance(result.initialSpeed,length,params,ballSpecs):
        BallPhysics::timeForDistance(velocity,length,ballSpecs);
    if(!std::isfinite(result.ballEta)||result.ballEta<=0.f||result.ballEta>params.flight*.001f)
    {result.reason="flight_timeout";return result;}
    Vector2f position=Vector2f::Zero(),remaining=velocity;
    BallPhysics::propagateBallPositionAndVelocity(position,remaining,result.ballEta,ballSpecs);
    result.terminalSpeed=remaining.norm();
    if(params.calibrated)
    {
      result.calibrated=true;result.exitSpeed=result.initialSpeed*params.exitRatio;
      result.rollSpeed=result.initialSpeed*params.rollRatio;
      const float friction=predictionBall(params,ballSpecs).staticFriction;
      result.stopTime=params.transitionSeconds+result.rollSpeed/(-1000.f*friction);
      result.receiveUntil=std::min(params.flight*.001f,result.stopTime+params.receiveWait);
      result.stopPoint={ball.x+(target.x-ball.x)*result.stopDistance/length,ball.y+(target.y-ball.y)*result.stopDistance/length};
      result.terminalSpeed=result.ballEta<=params.transitionSeconds?
          result.exitSpeed+(result.rollSpeed-result.exitSpeed)*result.ballEta/params.transitionSeconds:
          std::max(0.f,result.rollSpeed+(result.ballEta-params.transitionSeconds)*1000.f*friction);
    }
    const unsigned count=static_cast<unsigned>(std::ceil(length/params.sampleSpacing));
    result.samples.reserve(count+1);
    for(unsigned i=0;i<=count;++i)
    {
      const float fraction=static_cast<float>(i)/static_cast<float>(count);
      const float time=i==0?0.f:params.calibrated?
          calibratedTimeForDistance(result.initialSpeed,length*fraction,params,ballSpecs):
          BallPhysics::timeForDistance(velocity,length*fraction,ballSpecs);
      if(!std::isfinite(time)||time<0.f||time>params.flight*.001f)
      {result.reason="invalid_flight";result.samples.clear();return result;}
      result.samples.push_back({{ball.x+(target.x-ball.x)*fraction,ball.y+(target.y-ball.y)*fraction},time});
    }
    result.valid=true;result.reason="ready";return result;
  }

  inline Plan planKickoffThrough(Point ball,Point receiver,const Field& field,const std::vector<Obstacle>& obstacles,
                                const Params& params,const KickInfo& kickInfo,const KickLengthPair& pairs,
                                const BallSpecification& ballSpecs,const WalkingEngineOutput& walking,
                                float receiverHeading=std::numeric_limits<float>::quiet_NaN(),CandidateAudit* audit=nullptr)
  {
    Plan best;
    if(!paramsValid(params)||!fieldValid(field)||!finite(ball)||!finite(receiver))return best;
    const int side=effectiveSide(params,receiver);
    float bestCost=std::numeric_limits<float>::infinity();
    for(Point target:targetCandidates(field,params,side))
      for(float reserve:{0.f,200.f,500.f,800.f})
      {
        const float nominal=params.calibrated?calibratedNominalForStop(distance(ball,target)+reserve,params,kickInfo,ballSpecs):
            distance(ball,target)+reserve;
        const auto flight=mappedFlight(ball,target,nominal,kickInfo,pairs,ballSpecs,params);
        auto plan=evaluate(ball,receiver,target,field,obstacles,params,side,flight,
            walking.maxSpeed.translation.x(),walking.maxSpeed.translation.y(),walking.maxSpeed.rotation,ballSpecs.radius,receiverHeading);
        if(audit)audit->observe(target,reserve,flight,plan);
        if(!plan.valid){if(!best.valid)best=plan;continue;}
        const Point desired{field.opponentGoal*params.targetXFraction,side*field.halfWidth*params.targetYFraction};
        const float cost=distance(target,desired)+reserve*.25f+plan.receiverEta*10.f;
        if(cost<bestCost){best=plan;bestCost=cost;}
      }
    return best;
  }
}
