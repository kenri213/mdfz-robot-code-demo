#pragma once
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

/** Locally scoped fixed RUN. Guarded versions0/1 retain a<=1200mm endpoint;
 * version2 is explicitly an unchecked full-destination SIM-ONLY ablation.
 * No ball state, simulator/GT access, or actuator interface lives here. */
namespace WhiteFleetC15FixedRunDrive
{
  constexpr unsigned leaseMs=100,phaseMs=20;
  constexpr float horizon=1200.f,maxSpeed=2000.f;
  struct Point{float x=0,y=0;};
  struct Pose{float x=0,y=0,h=0;};
  struct Input
  {
    unsigned now=0,event=0,frame=0,cognitionStamp=0;
    float speed=0,targetYaw=0;Point target{};Pose issued{},current{},ordinary{};
    unsigned coordinateMode=0;bool splitAvailable=false,simulatedActor=false; // current=execution Preview, ordinary=committed Motion ODO.
    bool allowed=false;
  };
  struct Decision
  {
    bool active=false,valid=false;Point velocity{};float heading=0,remaining=0,previewRemaining=0,originalLength=0,along=0,lateral=0,previewOffset=0;
    const char* reason="off";
  };
  inline float angle(float a){return std::remainder(a,6.28318530718f);}
  inline bool finite(Point p){return std::isfinite(p.x)&&std::isfinite(p.y);}
  inline bool finite(Pose p){return finite(Point{p.x,p.y})&&std::isfinite(p.h);}
  inline Point rotate(Point p,float h){return {p.x*std::cos(h)-p.y*std::sin(h),p.x*std::sin(h)+p.y*std::cos(h)};}
  inline Decision evaluate(const Input& i)
  {
    Decision d;d.active=i.event||i.frame||i.speed!=0.f||i.coordinateMode;if(!d.active)return d;
    const auto stop=[&](const char* why){d.reason=why;return d;};
    if(!i.allowed)return stop("native_guard");
    if(i.coordinateMode>2u||(i.coordinateMode>0u&&!i.splitAvailable))return stop("coordinate_version");
    if(i.coordinateMode==2u&&!i.simulatedActor)return stop("sim_max_run_forbidden");
    if(!i.event||i.event>=i.frame||!i.frame||i.frame!=i.cognitionStamp||i.frame>i.now||i.now-i.frame>leaseMs)
      return stop("certificate_time");
    if(!finite(i.target)||!finite(i.issued)||!finite(i.current)||!std::isfinite(i.targetYaw)
       ||!std::isfinite(i.speed)||i.speed<=0.f||i.speed>maxSpeed)return stop("certificate_data");
    const float length=std::hypot(i.target.x,i.target.y);d.originalLength=length;
    if(i.coordinateMode==2u)
    {
      // Explicit simulator ablation, NOT a1200mm collision certificate. The
      // direct full destination is visible in the request/witness. No corridor,
      // obstacle, ODO-increment or near-point taper is claimed in this mode.
      if(length<=1.f||length>30000.f)return stop("sim_max_destination_data");
      const Point delta=rotate({i.current.x-i.issued.x,i.current.y-i.issued.y},-i.issued.h);
      const float yaw=angle(i.current.h-i.issued.h);
      const Point remaining=rotate({i.target.x-delta.x,i.target.y-delta.y},-yaw);
      d.remaining=d.previewRemaining=std::hypot(remaining.x,remaining.y);d.heading=angle(i.targetYaw-yaw);
      if(d.remaining<=1.f)return stop("sim_max_destination_reached");
      d.velocity={remaining.x*i.speed/d.remaining,remaining.y*i.speed/d.remaining};
      d.valid=true;d.reason="SIM_ONLY_max_run_unchecked";return d;
    }
    if(length<=1.f||length>horizon+.001f)return stop("unchecked_endpoint");
    const float dt=(i.now-i.frame)*.001f;
    const Point delta=rotate({i.current.x-i.issued.x,i.current.y-i.issued.y},-i.issued.h);
    const float yaw=angle(i.current.h-i.issued.h);
    if(std::hypot(delta.x,delta.y)>2500.f*dt+100.f||std::abs(yaw)>2.0943952f*dt+.1f)
      return stop("odometry_epoch");
    const float along=(delta.x*i.target.x+delta.y*i.target.y)/length;
    const float lateral=std::abs(delta.x*i.target.y-delta.y*i.target.x)/length;
    if(lateral>100.f||along< -100.f||along>=length)return stop("endpoint_or_corridor");
    const Point remaining=rotate({i.target.x-delta.x,i.target.y-delta.y},-yaw);
    d.remaining=std::hypot(remaining.x,remaining.y);d.heading=angle(i.targetYaw-yaw);
    d.previewRemaining=d.remaining;d.along=along;d.lateral=lateral;
    if(d.remaining<=1.f)return stop("endpoint_reached");
    if(i.coordinateMode==1u)
    {
      if(!finite(i.ordinary))return stop("ordinary_odometry_data");
      const Point body=rotate({i.ordinary.x-i.issued.x,i.ordinary.y-i.issued.y},-i.issued.h);
      const float bodyYaw=angle(i.ordinary.h-i.issued.h);
      if(std::hypot(body.x,body.y)>2500.f*dt+100.f||std::abs(bodyYaw)>2.0943952f*dt+.1f)
        return stop("ordinary_odometry_epoch");
      d.previewOffset=std::hypot(i.current.x-i.ordinary.x,i.current.y-i.ordinary.y);
      if(d.previewOffset>100.f||std::abs(angle(i.current.h-i.ordinary.h))>.1f)
        return stop("preview_ordinary_mismatch");
      d.along=(body.x*i.target.x+body.y*i.target.y)/length;
      d.lateral=std::abs(body.x*i.target.y-body.y*i.target.x)/length;
      if(d.lateral>100.f||d.along< -100.f||d.along>=length)return stop("ordinary_endpoint_or_corridor");
      const Point bodyRemaining=rotate({i.target.x-body.x,i.target.y-body.y},-bodyYaw);
      d.remaining=std::hypot(bodyRemaining.x,bodyRemaining.y);
      if(d.remaining<=1.f)return stop("ordinary_endpoint_reached");
      // The certificate is the immutable ISSUED segment (length above), not
      // an assumption that an estimated distance can only shrink. Ordinary
      // ODO and Preview both retain the original increment/corridor bounds;
      // their bounded difference cannot mint or extend a new endpoint.
    }
    else if(d.remaining>horizon+.001f)return stop("remaining_exceeds_checked_segment"); // Exact sealed64 policy when OFF.
    // Original finite endpoint feedback is retained, not a velocity hold.
    // Even the last valid command has <=120ms normal phase exposure (<=240mm
    // at the unchanged2000mm/s ceiling). The rest of the checked segment is
    // never turned into a longer target. Native Stand handles real braking.
    float speed=i.speed*std::min(1.f,d.remaining/horizon);
    speed=std::min(speed,d.previewRemaining/((leaseMs+phaseMs)*.001f));
    if(std::abs(std::atan2(remaining.y,remaining.x))>.785398164f)speed=0.f; // Turn with original native yaw first.
    d.velocity={remaining.x*speed/d.previewRemaining,remaining.y*speed/d.previewRemaining};
    d.valid=true;d.reason="bounded_native_velocity";return d;
  }
  template<class Request,class Odometry>
  Input input(const Request& r,unsigned now,const Odometry& o,bool allowed)
  {
    Input i;i.now=now;i.event=r.fixedRunDriveEvent;i.frame=r.fixedRunDriveFrame;
    i.cognitionStamp=r.ballEstimateTimestamp;i.speed=r.fixedRunDriveSpeed;
    i.target={r.walkTarget.translation.x(),r.walkTarget.translation.y()};i.targetYaw=r.walkTarget.rotation;
    i.issued={r.odometryData.translation.x(),r.odometryData.translation.y(),r.odometryData.rotation};
    i.current={o.translation.x(),o.translation.y(),o.rotation};i.allowed=allowed;
    if constexpr(requires{r.fixedRunDriveCoordinateVersion;})i.coordinateMode=r.fixedRunDriveCoordinateVersion;
    return i;
  }
  template<class Request,class Ordinary,class Preview>
  Input input(const Request& r,unsigned now,const Ordinary& ordinary,const Preview& preview,bool allowed,bool simulatedActor=false)
  {
    Input i=input(r,now,preview,allowed);
    i.ordinary={ordinary.translation.x(),ordinary.translation.y(),ordinary.rotation};i.splitAvailable=true;
    i.coordinateMode=r.fixedRunDriveCoordinateVersion;i.simulatedActor=simulatedActor;return i;
  }
  /** Diagnostic only, owned by each consumer's Motion thread. No control or
   * wireless side effects. Bounded48/event, changes or250ms heartbeat. */
  struct Witness
  {
    const void* owner=nullptr;unsigned event=0,count=0,lastTime=0,lastMotion=0,lastCoordinateMode=0;bool lastValid=false,lastAllowed=false;
    Point lastSent{};const char* lastReason="";
    void note(const char* consumer,const void* source,const Input& i,const Decision& d,Point sent,float geometry,unsigned motion)
    {
      if(!d.active)return;
      if(owner!=source||event!=i.event){*this=Witness();owner=source;event=i.event;}
      const bool changed=lastValid!=d.valid||lastAllowed!=i.allowed||lastMotion!=motion||lastCoordinateMode!=i.coordinateMode||std::strcmp(lastReason,d.reason)!=0
          ||std::hypot(sent.x-lastSent.x,sent.y-lastSent.y)>100.f;
      if(count>=48u||(!changed&&count&&i.now>=lastTime&&i.now-lastTime<250u))return;
      ++count;lastTime=i.now;lastValid=d.valid;lastAllowed=i.allowed;lastMotion=motion;lastCoordinateMode=i.coordinateMode;lastSent=sent;lastReason=d.reason;
      std::printf("C15_DRIVE_CONSUMER consumer=%s event=%u frame=%u motion_now=%u age_ms=%lld allowed=%d valid=%d reason=%s desired_mm_s=%.1f candidate=(%.1f,%.1f) sent_step=(%.1f,%.1f) geometry_mm=%.1f motion=%u witness=%u coordinate_mode=%u split=%d simulated_actor=%d issued_odo=(%.6f,%.6f,%.6f) ordinary_odo=(%.6f,%.6f,%.6f) preview_odo=(%.6f,%.6f,%.6f) original_segment=(%.6f,%.6f) original_length=%.6f body_remaining=%.6f preview_remaining=%.6f along=%.6f lateral=%.6f preview_offset=%.6f\n",
          consumer,i.event,i.frame,i.now,static_cast<long long>(i.now)-i.frame,i.allowed,d.valid,d.reason,i.speed,
          d.velocity.x,d.velocity.y,sent.x,sent.y,geometry,motion,count,i.coordinateMode,i.splitAvailable,i.simulatedActor,
          i.issued.x,i.issued.y,i.issued.h,i.ordinary.x,i.ordinary.y,i.ordinary.h,i.current.x,i.current.y,i.current.h,
          i.target.x,i.target.y,d.originalLength,d.remaining,d.previewRemaining,d.along,d.lateral,d.previewOffset);
    }
  };
}
