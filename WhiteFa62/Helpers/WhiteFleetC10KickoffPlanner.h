#pragma once
#include "WhiteFleetC9PassPlaybook.h"
#include "WhiteFleetC9CaptureFeasibility.h"
#include "WhiteFleetC15DefenceAdaptive.h"
#include "WhiteFleetC15RollingRendezvous.h"

/** Native probability-based risk and real corridor checks, NOT a claim of
 * universal interception immunity. No speed/radius/obstacle is falsified. */
namespace WhiteFleetC10KickoffPlanner
{
  namespace K=WhiteFleetKickoffThroughV8;
  inline bool rightExperiment(const K::Params& p)
  {return p.nativeReceive&&p.isolated&&p.playbook&&p.side==-1&&p.c12RightMinDegrees>0.f;}
  inline bool experimentAngle(K::Point ball,K::Point target,const K::Params& p,const K::Field& field)
  {
    if(WhiteFleetC15DefenceAdaptive::insideDomain(ball,target,field,p,-1))return true;
    if(!rightExperiment(p))return true;
    const float angle=-std::atan2(target.y-ball.y,target.x-ball.x)*180.f/3.14159265358979323846f;
    return std::isfinite(angle)&&angle>=p.c12RightMinDegrees;
  }
  inline std::vector<K::Point> experimentCandidates(const K::Field& field,const K::Params& p,int side)
  {return WhiteFleetC9ForwardFlank::candidates(field,p,side);}
  inline bool originalPoolTarget(K::Point target,const K::Field& field,const K::Params& p,int side)
  {
    auto original=p;original.raceRendezvous=false;original.prepareYFraction=0.f;original.insideCornerLane=false;original.defenceAdaptiveAngle=false;original.restoreProvenPool=false;original.c15DeepPass=false;
    for(auto candidate:experimentCandidates(field,original,side))
      if(candidate.x==target.x&&candidate.y==target.y)return true;
    return false; // Producer accepts only the existing finite table, not arbitrary nearby points.
  }
  template<class Rating>
  K::Plan evaluateRolling(K::Point ball,K::Point receiver,float heading,K::Point target,int side,
                          const K::Field& field,const std::vector<K::Obstacle>& obstacles,
                          const K::Params& p,K::FlightProfile flight,const BallSpecification& specs,
                          const WalkingEngineOutput& walking,Rating nativeRating)
  {
    namespace R=WhiteFleetC15RollingRendezvous;
    K::Plan out;out.target=target;out.prepare=K::preparePoint(field,p,side);out.side=side;
    if(side!=-1||!R::domain(ball,target,field,p)||!K::finite(receiver)||!std::isfinite(heading)||
       !std::isfinite(specs.radius)||specs.radius<=0.f||!WhiteFleetC9ForwardFlank::receiverSide(ball,receiver,side)||
       K::distance(receiver,out.prepare)>150.f)
    {out.reason="race_receiver_not_ready_or_geometry";return out;}
    if(!flight.valid||!flight.calibrated||!std::isfinite(flight.initialSpeed)||flight.initialSpeed<=0.f||
       flight.initialSpeed>R::calibratedNormalLimit+.01f||flight.samples.size()<2||
       K::distance(flight.samples.front().position,ball)>1.f||K::distance(flight.samples.back().position,target)>1.f)
    {out.reason="race_invalid_normal_mapping";return out;}
    for(const auto& o:obstacles)
      if(!K::finite(o.center)||!K::finite(o.left)||!K::finite(o.right)||!std::isfinite(o.radius)||o.radius<0.f||
         !std::isfinite(o.speedBound)||o.speedBound<0.f)
      {out.reason="invalid_obstacle";return out;}
    const float preparation=K::receiverTravelTime(receiver,out.prepare,walking.maxSpeed.translation.x(),
        walking.maxSpeed.translation.y(),walking.maxSpeed.rotation,p);
    if(!std::isfinite(preparation)||preparation>p.maxPrepare*.001f)
    {out.reason="prepare_unreachable";return out;}

    // This NEW first-hop is a rolling rendezvous, NOT a claim that a full-speed
    // ball stops at P. Retain original650/1500 capture and .25 arrival margin;
    // instead require predicted control BEFORE the unchanged field margin.
    float boundary=p.flight*.001f;
    for(float t=.025f;t<=p.flight*.001f;t+=.025f)
    {
      const auto point=WhiteFleetC9CaptureFeasibility::ballAt(ball,target,flight,p,specs,t);
      if(!K::finite(point)||point.x<field.ownGoal+p.fieldMargin||point.x>field.opponentGoal-p.fieldMargin||
         std::abs(point.y)>field.halfWidth-p.fieldMargin){boundary=t;break;}
    }
    flight.receiveUntil=std::min({flight.receiveUntil,R::maxControlHorizon,boundary-p.arrivalMargin});
    if(flight.receiveUntil<=0.f){out.reason="race_no_infield_control_window";return out;}
    const auto capture=WhiteFleetC9CaptureFeasibility::forecast(ball,target,receiver,field,p,flight,specs,
        [&](K::Point from,K::Point entry){return K::empiricalReceiverTime(from,entry,heading,
            walking.maxSpeed.translation.x(),walking.maxSpeed.translation.y(),walking.maxSpeed.rotation,p);});
    if(!capture.valid){out.reason="receiver_capture_not_feasible";return out;}
    if(capture.time+p.arrivalMargin>boundary+.001f){out.reason="race_control_after_boundary";return out;}
    out.captureTiming=true;out.timingOrigin=receiver;out.timingTarget=capture.captureEntry;
    out.captureBall=capture.ball;out.captureTime=capture.time;
    out.receiverEta=capture.receiverEta;out.safetyMargin=capture.margin;
    const auto prefix=R::assessPrefix(obstacles,p,specs.radius,capture.time,
        [&](float t){return WhiteFleetC9CaptureFeasibility::ballAt(ball,target,flight,p,specs,t);});
    out.earlyRiskChecked=prefix.mobileChecks>0;
    out.earlyClearance=std::isfinite(prefix.hardClearance)?prefix.hardClearance:0.f;
    out.temporalGap=prefix.minGap;out.hardRaceConflict=prefix.hardConflict;
    if(!prefix.valid){out.reason=prefix.reason;return out;}
    const auto atTarget=nativeRating(target),atCapture=nativeRating(capture.ball);
    out.nativeRiskChecked=true;out.nativePass=atTarget.pass;out.nativeCapturePass=atCapture.pass;out.nativeGoal=atTarget.goal;
    if(!WhiteFleetC9PassPlaybook::ratingsValid(atTarget)||!WhiteFleetC9PassPlaybook::ratingsValid(atCapture))
    {out.reason="native_pass_risk_rejected";return out;}
    out.valid=true;out.reason="race_rolling_control_before_boundary";out.rollingRendezvous=true;
    out.temporalGap=std::isfinite(prefix.minGap)?prefix.minGap:p.flight*.001f;out.controlBoundaryTime=boundary;
    out.nominalLength=flight.nominalLength;out.mappedLength=flight.mappedLength;out.initialSpeed=flight.initialSpeed;
    out.ballEta=flight.ballEta;out.exitSpeed=flight.exitSpeed;out.rollSpeed=flight.rollSpeed;out.receiveUntil=flight.receiveUntil;
    return out;
  }
  template<class Rating>
  K::Plan evaluate(K::Point ball,K::Point receiver,float heading,K::Point target,int side,
                   const K::Field& field,const std::vector<K::Obstacle>& obstacles,
                   const K::Params& p,const K::FlightProfile& flight,const BallSpecification& specs,
                   const WalkingEngineOutput& walking,Rating nativeRating)
  {
    if(p.raceRendezvous)return evaluateRolling(ball,receiver,heading,target,side,field,obstacles,p,flight,specs,walking,nativeRating);
    K::Plan out;out.target=target;out.prepare=K::preparePoint(field,p,side);out.side=side;
    if(p.c15DeepPass&&(!p.c14EarlyStart||(!WhiteFleetC9ForwardFlank::preferred(target,field)&&
       !(p.restoreProvenPool&&originalPoolTarget(target,field,p,side))&&
       !WhiteFleetC15DefenceAdaptive::domain(ball,target,field,p,side))))
    {out.reason="c15_not_penalty_front_target";return out;}
    if(!p.isolated||!p.playbook||!p.calibrated||!p.adaptiveMeet||side!=-1
       ||!K::finite(receiver)||!std::isfinite(heading)||!K::fieldValid(field)||!K::paramsValid(p)
       ||!std::isfinite(specs.radius)||specs.radius<=0.f
       ||!WhiteFleetC9ForwardFlank::valid(ball,receiver,target,field,p,side,obstacles))
    {out.reason="isolated_invalid_geometry";return out;}
    if(!experimentAngle(ball,target,p,field))
    {out.reason="c12_right_angle_below_registered_minimum";return out;}
    // Same short-circuit predicates/order as before; only the reason is split.
    if(!flight.valid)
    {out.reason="isolated_flight_invalid";return out;}
    if(!WhiteFleetC9ForwardFlank::receptionSafe(ball,target,flight,field,p,side))
    {out.reason="isolated_reception_unsafe";return out;}
    if(p.directCenter&&(!flight.calibrated||!std::isfinite(flight.initialSpeed)||flight.initialSpeed<=0.f
       ||flight.initialSpeed>3400.f||!K::directCenterDomain(target,field,p)
       ||flight.stopPoint.x>(p.centerFast?field.opponentPenalty+500.f:field.opponentPenalty-600.f)
       ||std::abs(flight.stopPoint.y)>(p.centerFast?2000.f:300.f)))
    {out.reason="central_finite_normal_pass_required";return out;}
    if(K::distance(flight.stopPoint,target)>1200.f)
    {out.reason="isolated_stop_error_exceeded";return out;}
    const float preparation=K::receiverTravelTime(receiver,out.prepare,walking.maxSpeed.translation.x(),
        walking.maxSpeed.translation.y(),walking.maxSpeed.rotation,p);
    if(!std::isfinite(preparation)||preparation>p.maxPrepare*.001f)
    {out.reason="prepare_unreachable";return out;}
    for(const auto& obstacle:obstacles)
    {
      if(!K::finite(obstacle.center)||!K::finite(obstacle.left)||!K::finite(obstacle.right)
         ||!std::isfinite(obstacle.radius)||obstacle.radius<0.f
         ||!std::isfinite(obstacle.speedBound)||obstacle.speedBound<0.f)
      {out.reason="invalid_obstacle";return out;}
      // A wide pass must still go behind the near-ball defensive front;
      // an empty ray strip alone must not admit a short sideways outlet.
      if(obstacle.mobile&&!obstacle.teammate&&obstacle.center.x>ball.x+250.f
         &&obstacle.center.x<field.opponentPenalty
         &&std::abs(obstacle.center.y-ball.y)<=field.goalHalfWidth+1000.f
         &&target.x<std::max({obstacle.center.x,obstacle.left.x,obstacle.right.x})+obstacle.radius+300.f)
      {out.reason="not_behind_near_ball_defence";return out;}
      if(K::obstacleDistance(ball,flight.stopPoint,obstacle)<=obstacle.radius+specs.radius+p.obstaclePadding)
      {out.reason="blocked_corridor";return out;}
      if((p.nativeReceive||p.directCenter)&&obstacle.mobile&&!obstacle.teammate&&
         (p.directCenter||WhiteFleetC15DefenceAdaptive::insideDomain(ball,target,field,p,side)||
          (obstacle.center.x>ball.x+250.f&&obstacle.center.x<field.opponentPenalty&&
           std::abs(obstacle.center.y-ball.y)<=field.goalHalfWidth+1000.f)))
      {
        // Short unobserved-motion allowance from EXISTING parameters, not an
        // all-flight interception guarantee or fitted cold-start capability.
        const float clearance=K::obstacleDistance(ball,flight.stopPoint,obstacle)
            -obstacle.radius-specs.radius-p.obstaclePadding-obstacle.speedBound*p.opponentReleaseAdvantage;
        out.earlyClearance=out.earlyRiskChecked?std::min(out.earlyClearance,clearance):clearance;
        out.earlyRiskChecked=true;
        if(clearance<=0.f){out.reason="early_defence_swept_corridor";return out;}
      }
    }
    const auto capture=WhiteFleetC9CaptureFeasibility::forecast(ball,target,receiver,field,p,flight,specs,
        [&](K::Point from,K::Point entry){return K::empiricalReceiverTime(from,entry,heading,
            walking.maxSpeed.translation.x(),walking.maxSpeed.translation.y(),walking.maxSpeed.rotation,p);});
    if(!capture.valid){out.reason="receiver_capture_not_feasible";return out;}
    out.captureTiming=true;out.timingOrigin=receiver;out.timingTarget=capture.captureEntry;
    out.captureBall=capture.ball;out.captureTime=capture.time;
    out.receiverEta=capture.receiverEta;out.safetyMargin=capture.margin;
    const auto atTarget=nativeRating(target),atCapture=nativeRating(capture.ball);
    out.nativeRiskChecked=true;out.nativePass=atTarget.pass;out.nativeCapturePass=atCapture.pass;out.nativeGoal=atTarget.goal;
    if(!WhiteFleetC9PassPlaybook::ratingsValid(atTarget)||!WhiteFleetC9PassPlaybook::ratingsValid(atCapture))
    {out.reason="native_pass_risk_rejected";return out;}
    out.valid=true;out.reason="isolated_native_risk_capture_ready";
    out.nominalLength=flight.nominalLength;out.mappedLength=flight.mappedLength;out.initialSpeed=flight.initialSpeed;
    out.ballEta=flight.ballEta;out.receiverEta=capture.receiverEta;out.safetyMargin=capture.margin;
    out.exitSpeed=flight.exitSpeed;out.rollSpeed=flight.rollSpeed;out.receiveUntil=flight.receiveUntil;
    return out;
  }
  template<class Rating>
  K::Plan chooseWithoutInside(K::Point ball,K::Point receiver,float heading,const K::Field& field,
                 const std::vector<K::Obstacle>& obstacles,const K::Params& p,const KickInfo& info,
                 const KickLengthPair& pairs,const BallSpecification& specs,const WalkingEngineOutput& walking,
                 Rating nativeRating,K::CandidateAudit* audit=nullptr)
  {
    K::Plan best;float bestScore=-INFINITY,bestAngle=INFINITY,bestReserve=0.f;bool bestPreferred=false,bestDeep=false;
    const int side=K::effectiveSide(p,receiver);
    unsigned considered=0;
    const std::vector<float> reserves=p.directCenter?std::vector<float>{0.f,200.f}:
        p.c14EarlyStart?std::vector<float>{0.f,200.f,500.f,800.f,1000.f,1100.f}:std::vector<float>{0.f,200.f,500.f,800.f};
    for(auto target:experimentCandidates(field,p,side))for(float reserve:reserves)
    {
      if(p.c15DeepPass&&!WhiteFleetC9ForwardFlank::preferred(target,field)&&
         !(p.restoreProvenPool&&originalPoolTarget(target,field,p,side))&&
         !WhiteFleetC15DefenceAdaptive::domain(ball,target,field,p,side))continue;
      if(rightExperiment(p)&&reserve<p.c12ReserveMin)continue;
      ++considered;
      const auto flight=K::mappedFlight(ball,target,K::calibratedNominalForStop(K::distance(ball,target)+reserve,p,info,specs),info,pairs,specs,p);
      auto plan=evaluate(ball,receiver,heading,target,side,field,obstacles,p,flight,specs,walking,nativeRating);
      if(audit)
      {
        audit->observe(target,reserve,flight,plan);
        auto& r=audit->records.back();r.ranked=plan.nativeRiskChecked;r.nativePass=plan.nativePass;r.nativeGoal=plan.nativeGoal;
      }
      if(!plan.valid){if(!best.valid)best=plan;continue;}
      const float score=WhiteFleetC9PassPlaybook::score({plan.nativePass,plan.nativeGoal},plan.safetyMargin,reserve,0.f);
      if(audit)audit->records.back().score=score; // Actual rank, never the old default zero.
      const bool preferred=WhiteFleetC9ForwardFlank::preferred(target,field)&&WhiteFleetC9ForwardFlank::preferred(flight.stopPoint,field);
      if(p.c14EarlyStart)
      {
        // Only after ALL guards: a tighter same-side legal corridor first,
        // faster mapped normal pass on that ray second, native score as tie.
        const float angle=std::abs(std::atan2(target.y-ball.y,target.x-ball.x));
        const bool sameRay=std::abs(angle-bestAngle)<1e-5f;
        const bool deep=WhiteFleetC9ForwardFlank::preferred(target,field);
        if(p.c15DeepPass&&p.restoreProvenPool&&best.valid&&deep!=bestDeep)
        {
          if(!deep)continue; // Any admitted deep target outranks the restored pool.
          best=plan;bestScore=score;bestAngle=angle;bestPreferred=preferred;bestDeep=true;bestReserve=reserve;continue;
        }
        if(p.c15DeepPass&&(!p.restoreProvenPool||deep))
        {
          // Among admitted deep points only: the native shooting opportunity
          // comes before raw power; faster legal normal mapping breaks ties.
          const bool sameGoal=std::abs(plan.nativeGoal-best.nativeGoal)<1e-5f;
          const bool sameSpeed=std::abs(plan.initialSpeed-best.initialSpeed)<=.01f;
          if(!best.valid||plan.nativeGoal>best.nativeGoal+1e-5f
             ||(sameGoal&&(plan.initialSpeed>best.initialSpeed+.01f
                 ||(sameSpeed&&(angle<bestAngle-1e-5f||(sameRay&&score>bestScore))))))
          {best=plan;bestScore=score;bestAngle=angle;bestPreferred=preferred;bestDeep=deep;bestReserve=reserve;}
          continue;
        }
        if(!best.valid||angle<bestAngle-1e-5f||(sameRay&&(plan.initialSpeed>best.initialSpeed+.01f
           ||(std::abs(plan.initialSpeed-best.initialSpeed)<=.01f&&score>bestScore))))
        {best=plan;bestScore=score;bestAngle=angle;bestPreferred=preferred;bestDeep=deep;bestReserve=reserve;}
        continue;
      }
      if(!best.valid||(preferred&&!bestPreferred)||(preferred==bestPreferred&&score>bestScore))
      {best=plan;bestScore=score;bestPreferred=preferred;bestReserve=reserve;}
    }
    if(WhiteFleetC15DefenceAdaptive::enabled(p,side))
    {
      // Keep the successful original selection as the baseline. This feature
      // adjusts the angle, not the whole old plan into a new short outlet.
      // No new ray -> exact original policy, including deep-first ordering.
      const bool baselineValid=best.valid,baselineDeep=best.valid&&WhiteFleetC9ForwardFlank::preferred(best.target,field);
      const float baselineX=best.target.x;
      const auto value=[&](const K::Plan& plan,float reserve)
      {
        return WhiteFleetC9PassPlaybook::score({std::min(plan.nativePass,plan.nativeCapturePass),plan.nativeGoal},
            plan.safetyMargin,reserve,0.f)+.10f*std::clamp(plan.earlyClearance/1000.f,0.f,1.f)
            -.02f*plan.receiverEta/(p.flight*.001f);
      };
      float adaptiveScore=best.valid?value(best,bestReserve):-INFINITY;
      bool chosenDeep=baselineDeep;
      for(auto target:WhiteFleetC15DefenceAdaptive::candidates(ball,field,p,side,obstacles,specs.radius))for(float reserve:reserves)
      {
        ++considered;
        const auto flight=K::mappedFlight(ball,target,K::calibratedNominalForStop(K::distance(ball,target)+reserve,p,info,specs),info,pairs,specs,p);
        auto plan=evaluate(ball,receiver,heading,target,side,field,obstacles,p,flight,specs,walking,nativeRating);
        if(audit)
        {
          audit->observe(target,reserve,flight,plan);
          auto& r=audit->records.back();r.ranked=plan.nativeRiskChecked;r.nativePass=plan.nativePass;r.nativeGoal=plan.nativeGoal;
        }
        if(!plan.valid)continue;
        const bool deep=WhiteFleetC9ForwardFlank::preferred(target,field);
        const float score=value(plan,reserve),angle=std::abs(std::atan2(target.y-ball.y,target.x-ball.x));
        if(audit)audit->records.back().score=score;
        // Preserve an available deep plan, and never shorten an available old
        // plan merely to make a near-side route score higher. Invalid old plans
        // still allow any fully admitted bounded alternative.
        if((baselineDeep&&!deep)||(baselineValid&&!deep&&target.x<baselineX-.01f))continue;
        if(best.valid&&chosenDeep&&!deep)continue;
        if(!best.valid||(deep&&!chosenDeep)||score>adaptiveScore+1e-6f||
           (std::abs(score-adaptiveScore)<=1e-6f&&(angle<bestAngle-1e-5f||
             (std::abs(angle-bestAngle)<1e-5f&&plan.initialSpeed>best.initialSpeed+.01f))))
        {best=plan;adaptiveScore=score;bestAngle=angle;chosenDeep=deep;}
      }
    }
    // An invalid final enumeration is not the reason all earlier ones failed.
    // Preserve validity/target/ranking exactly; full reasons come from audit.
    if(!best.valid)best.reason=considered?"c15_no_candidate_admitted":"c15_no_candidate_enumerated";
    return best;
  }

  template<class Rating>
  K::Plan chooseRolling(K::Point ball,K::Point receiver,float heading,const K::Field& field,
                       const std::vector<K::Obstacle>& obstacles,const K::Params& p,const KickInfo& info,
                       const KickLengthPair& pairs,const BallSpecification& specs,const WalkingEngineOutput& walking,
                       Rating nativeRating,K::CandidateAudit* audit=nullptr)
  {
    K::Plan best;best.reason="race_no_admitted_rendezvous";
    if(!WhiteFleetC15RollingRendezvous::enabled(p))return best;
    const float maximum=std::min(info[KickInfo::rlKickDynamic].ballVelocity.max,WhiteFleetC15RollingRendezvous::calibratedNormalLimit);
    for(auto target:WhiteFleetC15RollingRendezvous::targets(field,p))for(float fraction:{1.f,.95f,.90f})
    {
      const float speed=maximum*fraction;
      const float length=BallPhysics::getEndPosition(Vector2f::Zero(),{speed,0.f},specs).norm();
      const auto flight=K::mappedFlight(ball,target,length,info,pairs,specs,p);
      auto plan=evaluateRolling(ball,receiver,heading,target,-1,field,obstacles,p,flight,specs,walking,nativeRating);
      if(audit){audit->observe(target,0.f,flight,plan);auto& r=audit->records.back();r.ranked=plan.nativeRiskChecked;r.nativePass=plan.nativePass;r.nativeGoal=plan.nativeGoal;r.score=plan.temporalGap;}
      if(!plan.valid)continue;
      if(!best.valid||plan.initialSpeed>best.initialSpeed+.01f||
         (std::abs(plan.initialSpeed-best.initialSpeed)<=.01f&&
           (plan.temporalGap>best.temporalGap+.0001f||
             (std::abs(plan.temporalGap-best.temporalGap)<=.0001f&&plan.captureTime<best.captureTime))))best=plan;
    }
    return best; // No unsafe legacy/ordinary first-foot fallback in this profile.
  }

  template<class Rating>
  K::Plan chooseCenterFast(K::Point ball,K::Point receiver,float heading,const K::Field& field,
                 const std::vector<K::Obstacle>& obstacles,const K::Params& p,const KickInfo& info,
                 const KickLengthPair& pairs,const BallSpecification& specs,const WalkingEngineOutput& walking,
                 Rating nativeRating,K::CandidateAudit* audit=nullptr)
  {
    K::Plan best;best.reason="c2_no_jointly_safe_fast_receive";float score=-INFINITY;
    const float maximum=std::min(3400.f,info[KickInfo::rlKickDynamic].ballVelocity.max);
    for(auto target:WhiteFleetC9ForwardFlank::candidates(field,p,-1))for(float decrement:{0.f,50.f,100.f,150.f,200.f,250.f,300.f,350.f,400.f})
    {
      const float command=maximum-decrement;if(command<info[KickInfo::rlKickDynamic].ballVelocity.min)continue;
      const float length=BallPhysics::getEndPosition(Vector2f::Zero(),{command,0.f},specs).norm();
      const auto flight=K::mappedFlight(ball,target,length,info,pairs,specs,p);
      auto plan=evaluate(ball,receiver,heading,target,-1,field,obstacles,p,flight,specs,walking,nativeRating);
      const float value=plan.valid?WhiteFleetC9PassPlaybook::score({std::min(plan.nativePass,plan.nativeCapturePass),plan.nativeGoal},plan.safetyMargin,0.f,0.f):-INFINITY;
      if(audit){audit->observe(target,0.f,flight,plan);auto& a=audit->records.back();a.ranked=plan.nativeRiskChecked;a.nativePass=plan.nativePass;a.nativeGoal=plan.nativeGoal;a.score=plan.valid?value:0.f;}
      if(!plan.valid)continue;
      // Rank only the unchanged admitted pool. This existing short swept
      // clearance is not a full-flight interception guarantee or a new gate.
      if(p.firstPassClearance&&best.valid&&plan.earlyRiskChecked&&best.earlyRiskChecked
         &&std::isfinite(plan.earlyClearance)&&std::isfinite(best.earlyClearance)
         &&std::abs(plan.earlyClearance-best.earlyClearance)>.01f)
      {
        if(plan.earlyClearance>best.earlyClearance){best=plan;score=value;}
        continue;
      }
      if(!best.valid||plan.initialSpeed>best.initialSpeed+.01f||
         (std::abs(plan.initialSpeed-best.initialSpeed)<=.01f&&value>score)){best=plan;score=value;}
    }
    return best; // Same normal actuator cap; no Strong5400 or unsafe fallback.
  }

  template<class Rating>
  K::Plan choose(K::Point ball,K::Point receiver,float heading,const K::Field& field,
                 const std::vector<K::Obstacle>& obstacles,const K::Params& p,const KickInfo& info,
                 const KickLengthPair& pairs,const BallSpecification& specs,const WalkingEngineOutput& walking,
                 Rating nativeRating,K::CandidateAudit* audit=nullptr)
  {
    if(p.centerFast)return chooseCenterFast(ball,receiver,heading,field,obstacles,p,info,pairs,specs,walking,nativeRating,audit);
    if(p.raceRendezvous)return chooseRolling(ball,receiver,heading,field,obstacles,p,info,pairs,specs,walking,nativeRating,audit);
    auto old=p;old.insideCornerLane=false;
    auto baseline=chooseWithoutInside(ball,receiver,heading,field,obstacles,old,info,pairs,specs,walking,nativeRating,audit);
    if(!WhiteFleetC15DefenceAdaptive::insideEnabled(field,p,-1))return baseline;
    K::Plan inside;float bestScore=-INFINITY;
    for(auto target:WhiteFleetC15DefenceAdaptive::insideCandidates(ball,field,p,-1,obstacles,specs.radius))
      for(float reserve:{0.f,200.f,500.f,800.f,1000.f,1100.f})
      {
        const auto flight=K::mappedFlight(ball,target,K::calibratedNominalForStop(K::distance(ball,target)+reserve,p,info,specs),info,pairs,specs,p);
        auto plan=evaluate(ball,receiver,heading,target,-1,field,obstacles,p,flight,specs,walking,nativeRating);
        if(audit){audit->observe(target,reserve,flight,plan);auto& r=audit->records.back();r.ranked=plan.nativeRiskChecked;r.nativePass=plan.nativePass;r.nativeGoal=plan.nativeGoal;}
        if(!plan.valid)continue;
        const float score=WhiteFleetC9PassPlaybook::score({std::min(plan.nativePass,plan.nativeCapturePass),plan.nativeGoal},plan.safetyMargin,reserve,0.f)
            +.10f*std::clamp(plan.earlyClearance/1000.f,0.f,1.f)-.02f*plan.receiverEta/(p.flight*.001f);
        if(audit)audit->records.back().score=score;
        if(!inside.valid||score>bestScore){inside=plan;bestScore=score;}
      }
    // A safe inner-corner pass has explicit tactical priority when a low
    // defender leaves that lane open. Unsafe/late inner plans never force it:
    // the same existing C15 selection stays intact, NOT another kickoff path.
    return inside.valid?inside:baseline;
  }
}
