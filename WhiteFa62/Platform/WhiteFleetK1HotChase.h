#include "WhiteFleetSimContext.h"
#pragma once

#include <atomic>
#include <algorithm>
#include <cmath>
#include <initializer_list>

/** Small pursuit-only profile. Main enables it only after matching a robot's
 * SoC and its independent HotChase project path. No environment activation.
 * All distances are millimetres, times milliseconds, angles radians.
 */
namespace WhiteFleetK1HotChase
{
  namespace detail
  {
    inline std::atomic<bool>& storage() noexcept
    { return WhiteFleetSimContext::storage<std::atomic<bool>,107>(); }
    inline std::atomic<bool>& reacquireStorage() noexcept
    { return WhiteFleetSimContext::storage<std::atomic<bool>,108>(); }
    inline std::atomic<bool>& logReacquireStorage() noexcept
    { return WhiteFleetSimContext::storage<std::atomic<bool>,109>(); }
  }
  inline void enable() noexcept {detail::storage().store(true, std::memory_order_release);}
  inline bool enabled() noexcept {return detail::storage().load(std::memory_order_acquire);}
  inline void enableReacquire() noexcept {detail::reacquireStorage().store(true, std::memory_order_release);}
  inline bool reacquireEnabled() noexcept {return enabled() && detail::reacquireStorage().load(std::memory_order_acquire);}
  inline void enableLogReacquire() noexcept {detail::logReacquireStorage().store(true, std::memory_order_release);}
  inline bool logReacquireEnabled() noexcept {return reacquireEnabled() && detail::logReacquireStorage().load(std::memory_order_acquire);}

  /** The extra stamp is a genuine outgoing-ball witness, not a new observation
   * or a synthetic MotionInfo kick. Current-ball/path/referee gates still apply.
   */
  inline int kickAge(unsigned now, unsigned nativeKick, unsigned outgoingWitness, bool continued=false) noexcept
  {
    const unsigned stamp = std::max(nativeKick <= now ? nativeKick : 0,
                                    outgoingWitness <= now ? outgoingWitness : 0);
    return stamp && now - stamp < (continued ? 5000u:1200u) ? static_cast<int>(now - stamp) : -1;
  }

  inline bool pursue(bool profile, bool fieldPlayer, bool regularPlay,
                     bool clearPath, int kickAge, int ballAge, float distance,
                     float forward, float ballAngle, float awayVelocity,
                     float bodyTilt, float gyro, bool continued=false) noexcept
  {
    for(float x : {distance, forward, ballAngle, awayVelocity, bodyTilt, gyro})
      if(!std::isfinite(x)) return false;
    return profile && fieldPlayer && regularPlay && clearPath &&
           kickAge >= 0 && kickAge < (continued ? 5000:1200) && ballAge >= 0 && ballAge < 300 &&
           distance >= 750.f && distance <= 2500.f && forward > 450.f &&
           std::abs(ballAngle) < .70f && awayVelocity > (continued ? -250.f:250.f) &&
           bodyTilt >= 0.f && bodyTilt < .22f && gyro >= 0.f && gyro < 2.5f;
  }

  inline bool fastApproach(bool profile, bool fieldPlayer, bool regularPlay,
                           int ballAge, float distance, float forward,
                           float ballAngle, float targetAngle) noexcept
  {
    for(float x : {distance, forward, ballAngle, targetAngle})
      if(!std::isfinite(x)) return false;
    constexpr float pi = 3.14159265358979323846f;
    return profile && fieldPlayer && regularPlay && ballAge >= 0 && ballAge < 500 &&
           distance > 1500.f && forward > 450.f && std::abs(ballAngle) < 50.f*pi/180.f &&
           std::abs(targetAngle) < 60.f*pi/180.f;
  }
}
