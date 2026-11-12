#pragma once

#include <cstdlib>

#pragma push_macro("transition")
#undef transition
#include <vector>
#include "WhiteFleetKickoffThroughV8Planner.h"
#pragma pop_macro("transition")

/** A local pass INTENT, not a contract or a risk/receiver-ETA certificate.
 * No peer, obstacle classifier, acknowledgement or tactical score is required.
 * Robot avoidance and the actual kick remain the native skill's responsibility. */
namespace WhiteFleetAutonomousKickoff
{
  // One common playbook role, independent of each robot's local cost vote or
  // peer acknowledgement. A missing receiver never blocks the kicker's foot;
  // a missing kicker never silently promotes the receiver into a reverse pass.
  inline constexpr int kickerNumber=3;
  inline constexpr int receiverNumber=2;
  inline bool isKicker(int number){return number==kickerNumber;}
  inline bool isReceiver(int number){return number==receiverNumber;}

  /** One algorithm, explicit enrollment. A simulator flag can NEVER authorize
   * physical hardware or replace its exact Main identity + F1 permission. */
  inline bool enabled(bool physical,bool guardedPhysical,bool simulated,unsigned team,const char* simulationFlag)
  {
    if(physical)return guardedPhysical;
    return simulated&&team==5&&simulationFlag&&simulationFlag[0]=='1'&&simulationFlag[1]=='\0';
  }
  struct Intent
  {
    bool valid=false,rangeLimited=false;
    WhiteFleetKickoffThroughV8::Point target{};
    float nominalLength=0.f,requestedStopDistance=0.f;
    float observedClearance=0.f;
    unsigned observedBlockers=0;
    const char* reason="invalid_geometry_or_model";
  };

  struct Blocker
  {
    WhiteFleetKickoffThroughV8::Point center{};
    float radius=0.f; // Observed footprint + ball + measured uncertainty/margin, not a velocity prediction.
  };

  inline float clearance(WhiteFleetKickoffThroughV8::Point ball,WhiteFleetKickoffThroughV8::Point target,
                         const std::vector<Blocker>& blockers)
  {
    namespace K=WhiteFleetKickoffThroughV8;
    const float dx=target.x-ball.x,dy=target.y-ball.y,d2=dx*dx+dy*dy;
    float result=100000.f;
    if(!std::isfinite(d2)||d2<=1.f)return -100000.f;
    for(const auto& obstacle:blockers)
    {
      if(!K::finite(obstacle.center)||!std::isfinite(obstacle.radius)||obstacle.radius<=0.f)continue;
      const float fraction=((obstacle.center.x-ball.x)*dx+(obstacle.center.y-ball.y)*dy)/d2;
      if(fraction<=0.f||fraction>=1.f)continue;
      const K::Point nearest{ball.x+fraction*dx,ball.y+fraction*dy};
      result=std::min(result,K::distance(nearest,obstacle.center)-obstacle.radius);
    }
    return result;
  }

  inline Intent intent(WhiteFleetKickoffThroughV8::Point ball,const WhiteFleetKickoffThroughV8::Field& field,
                       const WhiteFleetKickoffThroughV8::Params& calibration,const KickInfo& info,
                       const BallSpecification& specs)
  {
    namespace K=WhiteFleetKickoffThroughV8;
    Intent result;
    if(!K::finite(ball)||!K::fieldValid(field)||!K::paramsValid(calibration)||
       !std::isfinite(specs.radius)||specs.radius<=0.f)return result;
    const float goalDx=field.opponentGoal-ball.x;
    const float x=std::min(field.opponentPenalty+250.f,field.opponentGoal-600.f);
    if(goalDx<=600.f||x<=ball.x+250.f)return result;
    // Forward and receiver/right side, even with a displaced centre estimate.
    // Project the ray outside the goal mouth rather than waiting for a sector
    // score. This remains a finite pass, not a deliberate direct kickoff goal.
    const float goalY=-field.goalHalfWidth-specs.radius-300.f;
    const float goalSafeY=ball.y+(goalY-ball.y)*(x-ball.x)/goalDx;
    const float y=std::min({-1000.f,ball.y-500.f,goalSafeY});
    if(y < -field.halfWidth+300.f)return result;
    const K::Point desired{x,y};
    const float wanted=K::distance(ball,desired);
    const auto& kick=info[KickInfo::rlKickDynamic];
    const float capacity=K::calibratedStopDistance(kick.ballVelocity.max,calibration,specs);
    if(!std::isfinite(wanted)||wanted<=0.f||!std::isfinite(capacity)||capacity<=600.f)return result;
    // Reachable distance changes the endpoint continuously. It never creates a
    // second playbook or waits for an unreachable deep target/receiver timing.
    const float stop=std::min(wanted,capacity-100.f);
    const float nominal=K::calibratedNominalForStop(stop,calibration,info,specs);
    if(!std::isfinite(nominal)||nominal<kick.range.min||nominal>kick.range.max)return result;
    const float fraction=stop/wanted;
    result.target={ball.x+(desired.x-ball.x)*fraction,ball.y+(desired.y-ball.y)*fraction};
    result.valid=K::finite(result.target)&&result.target.x>ball.x&&result.target.y<ball.y;
    result.rangeLimited=stop<wanted;result.requestedStopDistance=stop;result.nominalLength=nominal;
    result.reason=result.valid?"native_pass_intent":"invalid_forward_ray";
    return result;
  }

  /** Optional lower-side angle trim of a finite kickoff pass, at the same
   * distance and nominal power. Failure to trim cannot block the original kick. */
  inline Intent lowerFirstPassAngle(Intent pass,WhiteFleetKickoffThroughV8::Point ball,
                                    const WhiteFleetKickoffThroughV8::Field& field,
                                    const std::vector<Blocker>& observed)
  {
    namespace K=WhiteFleetKickoffThroughV8;
    if(!pass.valid||!K::finite(ball)||!K::finite(pass.target)||!K::fieldValid(field))
      return pass;
    constexpr float cos3=.998629535f,sin3=.052335956f;
    const float x=pass.target.x-ball.x,y=pass.target.y-ball.y;
    const K::Point lower{ball.x+x*cos3+y*sin3,ball.y+y*cos3-x*sin3};
    if(!K::finite(lower)||lower.x<=ball.x||lower.y>=ball.y||
       lower.x>field.opponentGoal-600.f||lower.y<-field.halfWidth+300.f)
      return pass;
    if(!observed.empty())
    {
      const float originalGap=clearance(ball,pass.target,observed);
      const float lowerGap=clearance(ball,lower,observed);
      // Preserve a useful observed margin, or at least do not worsen an
      // already narrow lane. This only rejects the trim, not the first pass.
      // With observed opponents, never trim towards them: keep the trim only
      // when it does not narrow the observed lane.
      if(lowerGap<originalGap)
        return pass;
      pass.observedClearance=lowerGap;
    }
    pass.target=lower;
    return pass;
  }

  /** Soft choice inside the original finite forward/right domain. No observation,
   * positive score, recipient ETA or ACK is an admission requirement. Unknown
   * obstacle velocities are NOT filled with zero or declared safe. */
  inline Intent laneIntent(WhiteFleetKickoffThroughV8::Point ball,const WhiteFleetKickoffThroughV8::Field& field,
                           const WhiteFleetKickoffThroughV8::Params& calibration,const KickInfo& info,
                           const BallSpecification& specs,const std::vector<Blocker>& blockers,
                           const WhiteFleetKickoffThroughV8::Point* receiver=nullptr)
  {
    namespace K=WhiteFleetKickoffThroughV8;
    Intent best=intent(ball,field,calibration,info,specs);
    if(!best.valid)return best;
    std::vector<Blocker> observed;
    for(const auto& b:blockers)
      if(K::finite(b.center)&&std::isfinite(b.radius)&&b.radius>0.f)observed.push_back(b);
    // The old proven finite P=(7810,-500) must remain eligible. An imaginary
    // extension of a stopped pass through the goal mouth is NOT its endpoint.
    // Indirect-kick/referee handling stays native; no goal legality is asserted.
    const auto& kick=info[KickInfo::rlKickDynamic];
    const float capacity=K::calibratedStopDistance(kick.ballVelocity.max,calibration,specs)-100.f;
    const K::Point base{best.target.x,std::min(-500.f,ball.y-250.f)};
    float bestScore=-std::numeric_limits<float>::infinity();
    for(unsigned i=0;i<=16;++i)
    {
      // Keep the landing point stable: at most 800 mm wider than the base lane,
      // never a blind wide wing pass when only part of the defence is observed.
      const float outer=std::max(std::min(base.y,-field.halfWidth+700.f),base.y-800.f);
      const K::Point desired{base.x,base.y+(outer-base.y)*static_cast<float>(i)/16.f};
      const float wanted=K::distance(ball,desired),stop=std::min(wanted,capacity);
      if(!std::isfinite(wanted)||wanted<=0.f||!std::isfinite(stop)||stop<=0.f)continue;
      Intent candidate;
      const float fraction=stop/wanted;
      candidate.target={ball.x+(desired.x-ball.x)*fraction,ball.y+(desired.y-ball.y)*fraction};
      candidate.nominalLength=K::calibratedNominalForStop(stop,calibration,info,specs);
      candidate.valid=K::finite(candidate.target)&&candidate.target.x>ball.x&&candidate.target.y<ball.y&&
        candidate.target.x<=field.opponentGoal-600.f&&candidate.target.y>=-field.halfWidth+300.f&&
        std::isfinite(candidate.nominalLength)&&candidate.nominalLength>=kick.range.min&&candidate.nominalLength<=kick.range.max;
      if(!candidate.valid)continue;
      candidate.requestedStopDistance=stop;candidate.rangeLimited=stop<wanted;
      candidate.observedBlockers=static_cast<unsigned>(observed.size());
      candidate.observedClearance=observed.empty()?std::numeric_limits<float>::quiet_NaN():clearance(ball,candidate.target,observed);
      const float travel=receiver&&K::finite(*receiver)?K::distance(*receiver,candidate.target):0.f;
      // Prefer a genuinely wider observed gap; once generous, prefer deeper,
      // less lateral and easier-to-reach endpoints. Never convert this to a veto.
      const float score=(observed.empty()?0.f:std::min(candidate.observedClearance,1800.f))+.04f*(candidate.target.x-ball.x)
                        -.04f*std::abs(candidate.target.y-ball.y)-.015f*travel;
      if(score>bestScore)
      {bestScore=score;candidate.reason=observed.empty()?"native_finite_lane_no_observation":"native_observed_lane_intent";best=candidate;}
    }
    return lowerFirstPassAngle(best,ball,field,observed);
  }

  /** Raise the normal kick's actual native velocity command by at most 6%,
   * never its range/model cap. P stays fixed; finite rollout may pass beyond P. */
  inline float strongerNominal(const Intent& pass,WhiteFleetKickoffThroughV8::Point ball,const WhiteFleetKickoffThroughV8::Field& field,
                               const WhiteFleetKickoffThroughV8::Params& calibration,const KickInfo& info,const BallSpecification& specs)
  {
    namespace K=WhiteFleetKickoffThroughV8;
    if(!pass.valid||!K::finite(ball)||!K::fieldValid(field))return pass.nominalLength;
    const auto& kick=info[KickInfo::rlKickDynamic];
    const float distance=K::distance(ball,pass.target);
    if(!std::isfinite(distance)||distance<=1.f)return pass.nominalLength;
    const float ux=(pass.target.x-ball.x)/distance,uy=(pass.target.y-ball.y)/distance;
    if(ux<=0.f||uy>=0.f)return pass.nominalLength;
    const float oldVelocity=kick.ballVelocity.limit(BallPhysics::velocityForDistance(pass.nominalLength,specs));
    float velocity=std::min(kick.ballVelocity.max,oldVelocity*1.06f);
    const float boundary=std::min((field.opponentGoal-600.f-ball.x)/ux,(-field.halfWidth+300.f-ball.y)/uy);
    if(!std::isfinite(oldVelocity)||!std::isfinite(velocity)||!std::isfinite(boundary))return pass.nominalLength;
    if(K::calibratedStopDistance(velocity,calibration,specs)>boundary-10.f)
    {
      const float limited=K::calibratedNominalForStop(boundary-10.f,calibration,info,specs);
      if(!std::isfinite(limited))return pass.nominalLength;
      velocity=std::min(velocity,BallPhysics::velocityForDistance(limited,specs));
    }
    const float nominal=kick.range.limit(BallPhysics::getEndPosition(Vector2f::Zero(),{velocity,0.f},specs).norm());
    return std::isfinite(nominal)?std::max(pass.nominalLength,nominal):pass.nominalLength;
  }

  /** A navigation target only, never a new contract or a fresh-ball claim.
   * Native BallModel already carries ordinary odometry/prediction to now. */
  inline WhiteFleetKickoffThroughV8::Point receiveIntercept(WhiteFleetKickoffThroughV8::Point contract,WhiteFleetKickoffThroughV8::Point ball,
                                                 WhiteFleetKickoffThroughV8::Point velocity,unsigned age,
                                                 const WhiteFleetKickoffThroughV8::Field& field,const BallSpecification& specs)
  {
    namespace K=WhiteFleetKickoffThroughV8;
    if(age>600u||!K::finite(ball)||!K::fieldValid(field)||ball.x<1000.f||
       ball.x>field.opponentGoal||std::abs(ball.y)>field.halfWidth)return contract;
    Vector2f target(ball.x,ball.y);
    if(K::finite(velocity))
      target=BallPhysics::propagateBallPosition(target,{velocity.x,velocity.y},.35f*(1.f-static_cast<float>(age)/600.f),specs);
    if(!target.allFinite())return contract;
    return {std::clamp(target.x(),1000.f,field.opponentGoal-600.f),
            std::clamp(target.y(),-field.halfWidth+300.f,field.halfWidth-300.f)};
  }
}
