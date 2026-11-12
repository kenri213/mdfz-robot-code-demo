#pragma once
#include "WhiteFleetKickoffThroughV8Math.h"

namespace WhiteFleetKickoffThroughV8
{
  struct CaptureOdometry {bool valid=false;Point position{};float heading=0.f;};
  inline Point captureRotate(Point p,float a)
  {return {std::cos(a)*p.x-std::sin(a)*p.y,std::sin(a)*p.x+std::cos(a)*p.y};}
  inline float captureAngle(float a){return std::atan2(std::sin(a),std::cos(a));}
  inline bool validCaptureOdometry(const CaptureOdometry& o)
  {return o.valid&&finite(o.position)&&std::isfinite(o.heading);}

  /** Use ONLY odometry DIFFERENCES. RobotPose corrections are not ball travel.
   * This never transforms P, rewrites perception or authorizes field geometry. */
  struct CaptureMotionEvidence
  {
    bool anchored=false,known=false;unsigned lastTime=0;
    CaptureOdometry origin{},last{};Point originalRelative{};
    float motionDistance=0.f;
    const char* reason="unanchored";
    bool begin(unsigned now,const CaptureOdometry& odometry,Point relative)
    {
      if(!now||!validCaptureOdometry(odometry)||!finite(relative))return false;
      anchored=known=true;lastTime=now;origin=last=odometry;originalRelative=relative;
      motionDistance=0.f;reason="capture_odometry_anchor";return true;
    }
    bool observe(unsigned now,const CaptureOdometry& odometry,Point relative,float speedBound,float yawBound)
    {
      known=false;
      if(!anchored||!validCaptureOdometry(odometry)||!finite(relative)||now<lastTime
         ||!std::isfinite(speedBound)||speedBound<=0.f||!std::isfinite(yawBound)||yawBound<=0.f)
      {reason="capture_odometry_unavailable";return false;}
      const float dt=static_cast<float>(now-lastTime)*.001f;
      if(distance(odometry.position,last.position)>speedBound*dt+100.f
         ||std::abs(captureAngle(odometry.heading-last.heading))>yawBound*dt+.1f)
      {reason="capture_odometry_discontinuous";return false;}
      const Point translation=captureRotate({odometry.position.x-origin.position.x,
          odometry.position.y-origin.position.y},-origin.heading);
      const Point ball=captureRotate(relative,captureAngle(odometry.heading-origin.heading));
      motionDistance=distance({translation.x+ball.x,translation.y+ball.y},originalRelative);
      last=odometry;lastTime=now;known=true;reason="capture_odometry_motion_measured";return true;
    }
  };
}
