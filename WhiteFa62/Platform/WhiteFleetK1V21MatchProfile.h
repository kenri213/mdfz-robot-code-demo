#include "WhiteFleetSimContext.h"
#pragma once

#include <atomic>
#include <algorithm>
#include <cmath>

/** Explicit process-local profile. Only the identity-bound NativeF1 V2.1
 * entry enables it; no environment variable can authorize a physical robot.
 */
namespace WhiteFleetK1V21MatchProfile
{
  namespace detail
  {
    inline std::atomic<bool>& enabledStorage() noexcept
    {
      return WhiteFleetSimContext::storage<std::atomic<bool>,110>();
    }
    inline std::atomic<bool>& physicalReceiverStorage() noexcept
    {
      static std::atomic<bool> value{false};
      return value;
    }
  }

  inline void enable() noexcept
  {
    detail::enabledStorage().store(true, std::memory_order_release);
  }

  inline bool enabled() noexcept
  {
    return detail::enabledStorage().load(std::memory_order_acquire);
  }

  // Main calls this only after the new exact SN + isolated-directory check.
  // This does NOT authorize the simulator's collision/navigation ablation.
  inline void enableGuardedPhysicalReceiver() noexcept
  {
    enable();
    detail::physicalReceiverStorage().store(true, std::memory_order_release);
  }

  inline bool guardedPhysicalReceiver() noexcept
  {
    return enabled() && detail::physicalReceiverStorage().load(std::memory_order_acquire);
  }

  template<class Params>
  Params guardedPrepare(Params params, float halfway, float lineWidth, float largeMargin) noexcept
  {
    if(!guardedPhysicalReceiver())return params;
    // Entire original150mm prepare tolerance stays50mm behind the native
    // large-area exit boundary. Do not weaken IllegalAreas to fit a SIM pose.
    if(!std::isfinite(halfway)||!std::isfinite(lineWidth)||!std::isfinite(largeMargin)
       ||lineWidth<=0.f||largeMargin<0.f)
    {params.prepareX=std::nanf("");return params;}
    // Do NOT mutate prepareX: direct-centre profile validation deliberately
    // binds it to the selected playbook. Adjust only the navigation point.
    const float safeX=halfway-lineWidth*.5f-largeMargin-200.f;
    params.prepareSafetyBackoff=std::max(params.prepareSafetyBackoff,std::max(0.f,params.prepareX-safeX));
    return params;
  }
}
