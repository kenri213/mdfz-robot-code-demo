#pragma once
#include "WhiteFleetKickoffThroughV8Planner.h"
#include "WhiteFleetKickoffThroughV8Session.h"

/** Prediction of the SAME original capture predicate, never capture evidence.
 * No fitted speeds/response constants and no production activation live here.
 * A caller must supply a calibrated, domain-qualified arrival-time model.
 * Both the actual receiver and this forecast keep immutable P/1500/650/1190.
 */
namespace WhiteFleetC9CaptureFeasibility
{
  namespace K=WhiteFleetKickoffThroughV8;
  struct Forecast
  {
    bool valid=false;
    K::Point ball{},bodyGoal{},captureEntry{};
    float time=0.f,receiverEta=0.f,margin=0.f;
    const char* reason="no_same_region_capture";
  };

  inline K::Point ballAt(K::Point launch,K::Point target,const K::FlightProfile& flight,
                          const K::Params& p,const BallSpecification& specs,float t)
  {
    const float length=K::distance(launch,target);
    if(!K::finite(launch)||!K::finite(target)||!flight.valid||!std::isfinite(length)||length<=0.f
       ||!std::isfinite(t)||t<0.f||!K::paramsValid(p)
       ||!std::isfinite(specs.staticFriction)||specs.staticFriction>=0.f
       ||!std::isfinite(flight.initialSpeed)||flight.initialSpeed<=0.f
       ||!std::isfinite(flight.stopDistance)||flight.stopDistance<=0.f)return {NAN,NAN};
    float travel=0.f;
    if(flight.calibrated)
    {
      if(!std::isfinite(flight.exitSpeed)||flight.exitSpeed<=0.f
         ||!std::isfinite(flight.rollSpeed)||flight.rollSpeed<=0.f
         ||!std::isfinite(flight.stopTime)||flight.stopTime<p.transitionSeconds)return {NAN,NAN};
      if(t<=p.transitionSeconds)
        travel=flight.exitSpeed*t+.5f*(flight.rollSpeed-flight.exitSpeed)/p.transitionSeconds*t*t;
      else
      {
        const float rolling=std::min(t-p.transitionSeconds,std::max(0.f,flight.stopTime-p.transitionSeconds));
        travel=K::calibratedTransitionDistance(flight.initialSpeed,p)
            +flight.rollSpeed*rolling+.5f*1000.f*K::predictionBall(p,specs).staticFriction*rolling*rolling;
      }
    }
    else
      travel=BallPhysics::propagateBallPosition(Vector2f::Zero(),Vector2f(flight.initialSpeed,0.f),t,specs).norm();
    travel=std::min(travel,flight.stopDistance);
    return {launch.x+(target.x-launch.x)*travel/length,launch.y+(target.y-launch.y)*travel/length};
  }

  /** First entry into the foot650 disc on the qualified straight approach.
   * Rolling race uses its actually executed locked P; other profiles retain
   * the same 1190-clamped body goal used by meetingInRegion.
   * Native avoidance can add time; qualified ETA must include its uncertainty.
   */
  inline bool firstEntry(K::Point receiver,K::Point bodyGoal,K::Point ball,K::Point& entry)
  {
    if(!K::finite(receiver)||!K::finite(bodyGoal)||!K::finite(ball))return false;
    constexpr float radius=K::captureDistance;
    if(K::distance(receiver,ball)<=radius){entry=receiver;return true;}
    const float dx=bodyGoal.x-receiver.x,dy=bodyGoal.y-receiver.y,a=dx*dx+dy*dy;
    if(a<=1e-6f)return false;
    const float x=receiver.x-ball.x,y=receiver.y-ball.y,b=2.f*(x*dx+y*dy),c=x*x+y*y-radius*radius;
    const float discriminant=b*b-4.f*a*c;
    if(!std::isfinite(discriminant)||discriminant<0.f)return false;
    const float fraction=(-b-std::sqrt(discriminant))/(2.f*a);
    if(!std::isfinite(fraction)||fraction<0.f||fraction>1.f)return false;
    entry={receiver.x+fraction*dx,receiver.y+fraction*dy};return K::finite(entry);
  }

  template<class QualifiedArrivalTime>
  Forecast forecast(K::Point launch,K::Point target,K::Point receiver,const K::Field& field,
                    const K::Params& p,const K::FlightProfile& flight,const BallSpecification& specs,
                    QualifiedArrivalTime arrival)
  {
    Forecast out;
    if(!p.playbook||!p.adaptiveMeet||!p.calibrated||!flight.calibrated
       ||!K::paramsValid(p)||!K::fieldValid(field)||!flight.valid
       ||!K::finite(launch)||!K::finite(target)||!K::finite(receiver)
       ||!std::isfinite(specs.staticFriction)||specs.staticFriction>=0.f
       ||!std::isfinite(flight.receiveUntil)||flight.receiveUntil<=0.f
       ||flight.receiveUntil>p.flight*.001f)return out;
    const int side=K::effectiveSide(p,receiver);
    if(!K::targetSafe(launch,target,field,p,side)
       ||!WhiteFleetC9ForwardFlank::receiverSide(launch,receiver,side))return out;
    const float dx=target.x-launch.x,dy=target.y-launch.y,length2=dx*dx+dy*dy;
    if(length2<=1.f)return out;
    std::vector<float> times;
    for(float t=.25f;t<=flight.receiveUntil+.001f;t+=.25f)times.push_back(t);
    for(float t:{flight.ballEta,flight.stopTime,flight.receiveUntil})
      if(std::isfinite(t)&&t>0.f&&t<=flight.receiveUntil)times.push_back(t);
    std::sort(times.begin(),times.end());
    for(float t:times)
    {
      const auto ball=ballAt(launch,target,flight,p,specs,t);
      if(!K::finite(ball)||K::distance(ball,target)>K::captureTargetRadius
         ||((ball.x-launch.x)*dx+(ball.y-launch.y)*dy)/length2<.5f
         ||ball.x<field.ownGoal+p.fieldMargin||ball.x>field.opponentGoal-p.fieldMargin
         ||std::abs(ball.y)>field.halfWidth-p.fieldMargin)continue;
      const bool lockedMeeting=p.raceRendezvous||p.directCenter;
      auto body=lockedMeeting?target:ball;const float fromP=K::distance(ball,target);
      if(!lockedMeeting&&fromP>1190.f)body={target.x+(ball.x-target.x)*1190.f/fromP,target.y+(ball.y-target.y)*1190.f/fromP};
      if(p.centerFast)body=K::centerMeetingBody(launch,target,ball,p);
      K::Point entry;
      if(!firstEntry(receiver,body,ball,entry))continue;
      const float eta=arrival(receiver,entry);
      if(!std::isfinite(eta)||eta<0.f||t-eta<p.arrivalMargin)continue;
      out={true,ball,body,entry,t,eta,t-eta,"same_original_capture_region_prediction"};return out;
    }
    return out;
  }
}
