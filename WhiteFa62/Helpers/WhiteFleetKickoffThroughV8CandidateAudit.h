#pragma once

#include "WhiteFleetKickoffThroughV8Math.h"
#include "WhiteFleetC9ForwardFlank.h"
#include <array>
#include <cstdio>
#include <cstring>

/** Read-only witnesses for the existing candidate loop. No audit value feeds a
 * planning decision. The ETA breakdown is explicitly recomputed from the same
 * inputs; result.receiverEta is the value from the actual evaluate() call. */
namespace WhiteFleetKickoffThroughV8
{
  struct CandidateAudit
  {
    struct Record
    {
      Point target{};
      float reserve=0.f;
      FlightProfile flight;
      Plan result;
      bool ranked=false;float nativePass=0.f,nativeGoal=0.f,score=0.f,uncertainty=0.f;
    };
    std::vector<Record> records;
    void observe(Point target,float reserve,const FlightProfile& flight,const Plan& result)
    {records.push_back({target,reserve,flight,result});}
  };

  struct CandidateAuditLatch
  {
    unsigned event=0,last=0,peerStamp=0,batches=0;
    bool firstEnumeration=false,firstQualified=false;
    bool select(unsigned now,unsigned nextEvent,unsigned nextPeerStamp,
                bool enumerated=false,bool qualified=false,bool reserveDecisions=false)
    {
      if(nextEvent!=event){event=nextEvent;last=peerStamp=batches=0;firstEnumeration=firstQualified=false;}
      if(!event||batches>=64)return false;
      const bool firstDecision=reserveDecisions&&((enumerated&&!firstEnumeration)||(qualified&&!firstQualified));
      const unsigned reserved=reserveDecisions?static_cast<unsigned>(!firstEnumeration)+static_cast<unsigned>(!firstQualified):0u;
      if(!firstDecision&&batches>=64-reserved)return false;
      if(!firstDecision&&batches&&now>=last&&now-last<1000&&(nextPeerStamp==peerStamp||now-last<250))return false;
      if(reserveDecisions){firstEnumeration|=enumerated;firstQualified|=qualified;}
      last=now;peerStamp=nextPeerStamp;++batches;return true;
    }
  };

  struct ReceiverTimingAudit
  {
    bool evaluated=false;
    float travel=0.f,turn=0.f,start=0.f,head=0.f,release=0.f,recomputed=0.f;
    float forward=0.f,lateral=0.f,rotation=0.f;
  };

  /** Local diagnostics only. Never used to select a candidate or make an
   * invalid flight's default stop point look like an observed zero error. */
  inline void printRolloutAudit(const CandidateAudit& audit,int self,unsigned now,unsigned event,
                               Point ball,const Field& field,const Params& p,int side)
  {
    for(std::size_t i=0;i<audit.records.size();++i)
    {
      const auto& r=audit.records[i];const auto& f=r.flight;
      const bool receptionEvaluated=f.valid;
      const bool receptionSafe=receptionEvaluated&&WhiteFleetC9ForwardFlank::receptionSafe(ball,r.target,f,field,p,side);
      const bool stopEvaluated=f.valid&&f.calibrated&&finite(f.stopPoint)&&finite(r.target);
      const float stopError=stopEvaluated?distance(f.stopPoint,r.target):NAN;
      std::printf("C15_ROLLOUT same_update=1 self=%d now=%u event=%u index=%zu flight_valid=%d flight_reason=%s reception_evaluated=%d reception_safe=%d stop_error_evaluated=%d stop_error_mm=%.3f stop_error_within1200=%d final_valid=%d first_rejection=%s\n",
          self,now,event,i,f.valid,f.reason,receptionEvaluated,receptionSafe,stopEvaluated,stopError,
          stopEvaluated&&stopError<=1200.f,r.result.valid,r.result.reason);
    }
  }
  inline ReceiverTimingAudit receiverTimingAudit(const Plan& plan,const FlightProfile& flight,const Params& p,
                                                float maxForward,float maxSide,float maxYaw,float heading)
  {
    ReceiverTimingAudit result;
    if(p.isolated&&!plan.captureTiming)return result;
    // These precede the ETA in evaluate(); zero from an early rejection is not
    // a measured zero-second arrival. Do not invent a ready receiver.
    if(!std::strcmp(plan.reason,"invalid_input")||!std::strcmp(plan.reason,"invalid_geometry")
       ||!std::strcmp(plan.reason,"prepare_unreachable")||!flight.valid
       ||!std::strcmp(plan.reason,"invalid_calibrated_reception"))return result;
    result.evaluated=true;
    const Point destination=plan.captureTiming?plan.timingTarget:(p.calibrated?flight.stopPoint:plan.target);
    const Point origin=plan.captureTiming?plan.timingOrigin:plan.prepare;
    const float length=distance(origin,destination);
    result.start=p.startDelay;result.head=p.headBodyDelay;
    result.forward=maxForward*p.receiverSpeedFactor;
    result.lateral=maxSide*p.receiverSpeedFactor;
    result.rotation=maxYaw;
    if(p.calibrated)
    {
      result.forward=std::min(result.forward,p.empiricalReceiverSpeed);
      result.lateral=std::min(result.lateral,900.f);result.rotation=std::min(maxYaw,1.5f);
      if(!std::isfinite(heading))heading=plan.side>0?-1.3f:1.3f;
      const float side=origin.y<0.f?1.f:-1.f;
      const float desired=(length>1.f?std::atan2(destination.y-origin.y,destination.x-origin.x):heading)+side*p.headBodyYaw;
      result.turn=std::abs(std::atan2(std::sin(desired-heading),std::cos(desired-heading)))/result.rotation;
      result.release=p.empiricalReleaseDelay;
    }
    else {result.turn=p.headBodyYaw/maxYaw;result.release=p.releaseDelay;}
    result.travel=std::hypot(length*std::cos(p.headBodyYaw)/result.forward,length*std::sin(p.headBodyYaw)/result.lateral);
    result.recomputed=result.travel+result.turn+result.start+result.head+result.release;
    return result;
  }

  inline void printCandidateAudit(const CandidateAudit& audit,int team,int self,unsigned now,unsigned event,
                                 Point receiver,float heading,const Params& params,
                                 float maxForward,float maxSide,float maxYaw)
  {
    unsigned valid=0;
    for(std::size_t i=0;i<audit.records.size();++i)
    {
      const auto& record=audit.records[i];const auto& plan=record.result;const auto& flight=record.flight;
      const auto timing=receiverTimingAudit(plan,flight,params,maxForward,maxSide,maxYaw,heading);
      valid+=plan.valid;
      std::printf("C9_CANDIDATE same_update=1 team=%d self=%d now=%u event=%u index=%zu target=%.3f,%.3f reserve=%.1f prepare=%.3f,%.3f receiver=%.3f,%.3f receiver_heading=%.6f max_forward=%.3f max_side=%.3f max_yaw=%.6f normal_kick=rlKickDynamic nominal=%.3f mapped=%.3f command_velocity=%.3f flight_valid=%d flight_reason=%s stop=%.3f,%.3f stop_distance=%.3f ball_eta=%.6f receive_until=%.6f receiver_eta_evaluated=%d receiver_eta=%.6f receiver_eta_recomputed=%.6f eta_travel=%.6f eta_turn=%.6f eta_start=%.6f eta_head=%.6f eta_release=%.6f receiver_forward_cap=%.3f receiver_side_cap=%.3f receiver_yaw_cap=%.6f margin=%.6f required_margin=%.6f valid=%d reason=%s\n",
          team,self,now,event,i,record.target.x,record.target.y,record.reserve,plan.prepare.x,plan.prepare.y,
          receiver.x,receiver.y,heading,maxForward,maxSide,maxYaw,flight.nominalLength,flight.mappedLength,flight.initialSpeed,
          flight.valid,flight.reason,flight.stopPoint.x,flight.stopPoint.y,flight.stopDistance,flight.ballEta,flight.receiveUntil,
          timing.evaluated,plan.receiverEta,timing.recomputed,timing.travel,timing.turn,timing.start,timing.head,timing.release,
          timing.forward,timing.lateral,timing.rotation,plan.safetyMargin,params.arrivalMargin,plan.valid,plan.reason);
      if(record.ranked)
        std::printf("C9_PLAYBOOK_RATING same_update=1 team=%d self=%d now=%u event=%u index=%zu native_pass=%.6f native_goal=%.6f score=%.6f observation_uncertainty=%.6f valid=%d reason=%s\n",
            team,self,now,event,i,record.nativePass,record.nativeGoal,record.score,record.uncertainty,plan.valid,plan.reason);
      if(params.nativeReceive)
        std::printf("C11_EARLY_RISK same_update=1 self=%d event=%u now=%u index=%zu checked=%d clearance_mm=%.3f horizon_s=%.3f full_flight_guarantee=0\n",
            self,event,now,i,plan.earlyRiskChecked,plan.earlyClearance,params.opponentReleaseAdvantage);
      if(params.isolated)
        std::printf("C10_CAPTURE_FORECAST same_update=1 self=%d event=%u now=%u index=%zu evaluated=%d time=%.6f entry=%.3f,%.3f ball=%.3f,%.3f native_risk_checked=%d native_P=%.6f native_capture=%.6f actual_capture_permission=0\n",
            self,event,now,i,plan.captureTiming,plan.captureTime,plan.timingTarget.x,plan.timingTarget.y,
            plan.captureBall.x,plan.captureBall.y,plan.nativeRiskChecked,plan.nativePass,plan.nativeCapturePass);
    }
    // Count the actual first-failing reason of EVERY candidate, not merely the
    // last failure returned by the legacy best-plan loop. No fixed reason list.
    for(std::size_t i=0;i<audit.records.size();++i)
    {
      const char* reason=audit.records[i].result.reason;bool earlier=false;unsigned count=0;
      for(std::size_t j=0;j<i;++j)earlier|=!std::strcmp(reason,audit.records[j].result.reason);
      if(earlier)continue;
      for(const auto& record:audit.records)count+=!std::strcmp(reason,record.result.reason);
      std::printf("C9_CANDIDATE_REASON same_update=1 team=%d self=%d now=%u event=%u reason=%s count=%u total=%zu valid=%u\n",
          team,self,now,event,reason,count,audit.records.size(),valid);
    }
  }
}
