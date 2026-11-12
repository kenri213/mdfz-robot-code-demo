#pragma once
#include "WhiteFleetC9CaptureMotionEvidence.h"

namespace WhiteFleetKickoffThroughV8
{
  /** One bounded sender-foot aim, not a replacement RobotPose or ball model.
   * Lock the already admitted FIELD endpoint in ODOM using a qualified entry
   * pose. Later requests use current odometry and their own fresh relative
   * ball; Motion retains its normal request-to-preview odometry correction.
   * A failed anchored observation is terminal: never reseed a new aim/lease.
   */
  struct WhiteFleetKickoffAimAnchor
  {
    bool anchored=false,valid=false,retired=false;
    unsigned stamp=0,lastTime=0;
    Point targetOdometry{};
    const char* reason="aim_unanchored";
    static constexpr unsigned maximumAge=1500;
    static constexpr float maximumEntryYaw=.03490658504f; // 2 degrees, entry only.
    static constexpr float maximumPoseYaw=.1745329252f; // 10 degrees.

    bool begin(unsigned now,Point fieldTarget,Point fieldPosition,float fieldHeading,
               float yawVariance,const CaptureOdometry& odometry)
    {
      valid=false;
      if(anchored||retired)return reject("aim_reanchor_forbidden");
      if(!now||!finite(fieldTarget)||!finite(fieldPosition)||!std::isfinite(fieldHeading)
         ||!std::isfinite(yawVariance)||yawVariance<0.f
         ||yawVariance>maximumEntryYaw*maximumEntryYaw||!validCaptureOdometry(odometry))
      {reason="aim_entry_not_qualified";return false;}
      const Point relative=captureRotate({fieldTarget.x-fieldPosition.x,fieldTarget.y-fieldPosition.y},-fieldHeading);
      const Point rotated=captureRotate(relative,odometry.heading);
      const Point target{odometry.position.x+rotated.x,odometry.position.y+rotated.y};
      if(!finite(relative)||!finite(target))
      {reason="aim_entry_transform_invalid";return false;}
      targetOdometry=target;targetField=fieldTarget;entryPosition=fieldPosition;entryHeading=fieldHeading;
      origin=last=odometry;stamp=lastTime=now;anchored=valid=true;
      reason="aim_odometry_anchored";return true;
    }

    bool observe(unsigned now,Point sameFieldTarget,Point fieldPosition,float fieldHeading,
                 const CaptureOdometry& odometry,float speedBound,float yawBound)
    {
      valid=false;
      if(!anchored||retired)return reject("aim_not_live");
      if(!now||now<stamp||now<lastTime||now-stamp>maximumAge
         ||!finite(sameFieldTarget)||distance(sameFieldTarget,targetField)>1.f
         ||!finite(fieldPosition)||!std::isfinite(fieldHeading)||!validCaptureOdometry(odometry)
         ||!std::isfinite(speedBound)||speedBound<=0.f||!std::isfinite(yawBound)||yawBound<=0.f)
        return reject("aim_frame_or_contract_invalid");
      const float dt=static_cast<float>(now-lastTime)*.001f;
      const float translationLimit=speedBound*dt+100.f,yawLimit=yawBound*dt+.1f;
      // Identical continuity allowances to the original C9 odometry guard.
      if(!std::isfinite(translationLimit)||!std::isfinite(yawLimit)
         ||distance(odometry.position,last.position)>translationLimit
         ||std::abs(captureAngle(odometry.heading-last.heading))>yawLimit)
        return reject("aim_odometry_discontinuous");
      const Point delta=captureRotate({odometry.position.x-origin.position.x,
          odometry.position.y-origin.position.y},-origin.heading);
      const Point fieldDelta=captureRotate(delta,entryHeading);
      const Point anchoredPosition{entryPosition.x+fieldDelta.x,entryPosition.y+fieldDelta.y};
      const float anchoredHeading=captureAngle(entryHeading+captureAngle(odometry.heading-origin.heading));
      // Current localization remains authoritative for this finite validity
      // check. Do not write it, compensate it, or silently accept a large jump.
      if(!finite(anchoredPosition)||!std::isfinite(anchoredHeading)
         ||distance(anchoredPosition,fieldPosition)>300.f
         ||std::abs(captureAngle(anchoredHeading-fieldHeading))>maximumPoseYaw)
        return reject("aim_field_pose_disagreement");
      last=odometry;lastTime=now;valid=true;reason="aim_current_frame_valid";return true;
    }

  private:
    Point targetField{},entryPosition{};
    float entryHeading=0.f;
    CaptureOdometry origin{},last{};
    bool reject(const char* why)
    {valid=false;retired|=anchored;reason=why;return false;}
  };
}
