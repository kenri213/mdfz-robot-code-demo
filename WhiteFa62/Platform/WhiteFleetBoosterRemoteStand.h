#include "WhiteFleetSimContext.h"
/** Opt-in remote-loss ownership policy. No SDK, motion or mode commands. */
#pragma once

#include <atomic>
#include <cstdint>

namespace WhiteFleetBoosterRemoteStand
{
  enum class Owner : std::uint64_t {unknown = 0, manual = 1, automatic = 2};
  namespace detail
  {
    constexpr std::uint64_t holdBit = 1u, neutralBit = 2u, manualBit = 4u, automaticBit = 8u;
    constexpr unsigned ownerShift = 4, generationShift = 8;
    constexpr std::uint64_t ownerMask = 3u << ownerShift, takeoverFenceBit = 64u, choiceReleasedBit = 128u;
    constexpr std::uint64_t flagMask = 255u;
    constexpr unsigned modifierButtons = 64u, manualButtons = 576u, automaticButtons = 320u;
    struct Storage
    {
      std::atomic<bool> active{false};
      std::atomic<bool> automaticContinuity{false};
      std::atomic<bool> awaitingRecoveryStand{false};
      // A single CAS binds recovery to the same disconnect generation.
      std::atomic<std::uint64_t> packed{0};
    };
    inline Storage& remoteStandStorage() noexcept {return WhiteFleetSimContext::storage<Storage,105>();}
  }

  inline void enable() noexcept {detail::remoteStandStorage().active.store(true);}
  inline bool enabled() noexcept {return detail::remoteStandStorage().active.load();}
  inline void enableAutomaticContinuity() noexcept {detail::remoteStandStorage().automaticContinuity.store(true);}
  inline bool automaticContinuityEnabled() noexcept
  {return enabled() && detail::remoteStandStorage().automaticContinuity.load();}
  inline Owner owner() noexcept
  {return static_cast<Owner>((detail::remoteStandStorage().packed.load() & detail::ownerMask) >> detail::ownerShift);}
  inline const char* ownerName(Owner value) noexcept
  {return value == Owner::automatic ? "automatic" : value == Owner::manual ? "manual" : "unknown";}
  inline bool takeoverFenced() noexcept
  {return automaticContinuityEnabled() && (detail::remoteStandStorage().packed.load() & detail::takeoverFenceBit);}
  inline bool held() noexcept
  {return enabled() && (detail::remoteStandStorage().packed.load() & detail::holdBit) != 0;}
  inline std::uint64_t generation() noexcept
  {return detail::remoteStandStorage().packed.load() >> detail::generationShift;}

  inline void latchDisconnect() noexcept
  {
    if(!enabled()) return;
    auto& packed = detail::remoteStandStorage().packed;
    auto previous = packed.load();
    for(;;)
    {
      const bool keepAutomatic = automaticContinuityEnabled()
          && (previous & detail::ownerMask) == (static_cast<std::uint64_t>(Owner::automatic) << detail::ownerShift)
          && !(previous & detail::holdBit)
          && !detail::remoteStandStorage().awaitingRecoveryStand.load();
      const auto next = (((previous >> detail::generationShift) + 1u) << detail::generationShift)
          | (previous & detail::ownerMask)
          | (automaticContinuityEnabled() ? detail::takeoverFenceBit : 0u)
          | (keepAutomatic ? 0u : detail::holdBit);
      if(packed.compare_exchange_weak(previous, next)) return;
    }
  }

  /** A successful behavior handoff, not a raw chord or cached inactive state. */
  inline bool tryTransfer(std::uint64_t expectedGeneration, Owner requested) noexcept
  {
    if(!automaticContinuityEnabled()) return true;
    if(requested == Owner::unknown || detail::remoteStandStorage().awaitingRecoveryStand.load()) return false;
    auto& packed = detail::remoteStandStorage().packed;
    auto previous = packed.load();
    if((previous >> detail::generationShift) != expectedGeneration
       || (previous & (detail::holdBit | detail::takeoverFenceBit))) return false;
    const auto next = (previous & ~detail::flagMask)
        | (static_cast<std::uint64_t>(requested) << detail::ownerShift);
    return packed.compare_exchange_strong(previous, next);
  }

  /** Physical F1 revokes AUTO; an old handoff cannot cross this generation. */
  inline void revokeOwnership() noexcept
  {
    if(!automaticContinuityEnabled()) return;
    auto& packed = detail::remoteStandStorage().packed;
    auto previous = packed.load();
    while(!packed.compare_exchange_weak(previous,
        (((previous >> detail::generationShift) + 1u) << detail::generationShift) | (previous & detail::holdBit))) {}
  }

  /** A manual in-match get-up must not hand straight to automatic football. */
  inline void beginBodyRecovery() noexcept
  {
    if(!enabled()) return;
    auto& value = detail::remoteStandStorage();
    value.awaitingRecoveryStand.store(true);
    auto previous = value.packed.load();
    while(!value.packed.compare_exchange_weak(previous,
        (((previous >> detail::generationShift) + 1u) << detail::generationShift) | detail::holdBit)) {}
  }

  inline bool bodyRecoveryPending() noexcept
  {return enabled() && detail::remoteStandStorage().awaitingRecoveryStand.load();}

  /** Called by Cognition only after a real upright Stand/Walk feedback frame. */
  inline bool onRecoveryStanding(bool realUprightStand) noexcept
  {
    if(!enabled() || !realUprightStand
       || !detail::remoteStandStorage().awaitingRecoveryStand.exchange(false)) return false;
    // Discard all neutral/LT evidence observed before the robot really stood.
    latchDisconnect();
    return true;
  }

  /** Called only for a real new canonical Main callback, never cached frames. */
  inline void observeSample(unsigned buttons, bool axesNeutral) noexcept
  {
    if(!enabled()) return;
    auto& packed = detail::remoteStandStorage().packed;
    auto previous = packed.load();
    const auto sampleGeneration = previous >> detail::generationShift;
    while((previous & (detail::holdBit | detail::takeoverFenceBit))
          && (previous >> detail::generationShift) == sampleGeneration)
    {
      auto next = previous & ~(std::uint64_t{14} | detail::choiceReleasedBit);
      if(axesNeutral)
      {
        if(buttons == 0)
        {
          // Natural release after a requested recovery must not erase it
          // before the behavior thread has had a chance to consume the CAS.
          const auto ready = previous & (detail::manualBit | detail::automaticBit);
          next |= ready ? ready | detail::choiceReleasedBit : detail::neutralBit;
        }
        else if(buttons == detail::modifierButtons)
          // Event-driven controllers publish LT separately. It proves no new
          // neutral and requests no action, but preserves existing evidence.
          next |= previous & (detail::neutralBit | detail::manualBit | detail::automaticBit);
        else if(buttons == detail::manualButtons || buttons == detail::automaticButtons)
        {
          const auto requested = buttons == detail::manualButtons ? detail::manualBit : detail::automaticBit;
          if((previous & detail::neutralBit) || (previous & 14u) == requested)
            next |= requested; // Spend neutral once; an old held chord cannot mint another action.
        }
      }
      if(packed.compare_exchange_weak(previous, next)) return;
      // Never attach an old sample to a concurrently created loss generation.
    }
  }

  /** Snapshot only. tryResume must validate the generation again atomically. */
  inline unsigned resumeChoice() noexcept
  {
    if(!enabled()) return 0;
    const auto packed = detail::remoteStandStorage().packed.load();
    if(!(packed & (detail::holdBit | detail::takeoverFenceBit))) return 0;
    if(automaticContinuityEnabled() && !(packed & detail::choiceReleasedBit)) return 0;
    const auto value = packed & 14u;
    if(value == detail::manualBit) return 1;
    if(value == detail::automaticBit) return 2;
    return 0;
  }

  /** Illegal football context cannot queue a future LT handoff. Keep the hold,
   * invalidate only observed input evidence and bind it to a new generation. */
  inline void discardResumeEvidence() noexcept
  {
    if(!enabled()) return;
    auto& packed = detail::remoteStandStorage().packed;
    auto previous = packed.load();
    while((previous & (detail::holdBit | detail::takeoverFenceBit))
          && (previous & (14u | detail::choiceReleasedBit)))
    {
      const auto next = (((previous >> detail::generationShift) + 1u) << detail::generationShift)
          | (previous & detail::flagMask & ~(std::uint64_t{14} | detail::choiceReleasedBit));
      if(packed.compare_exchange_weak(previous, next)) return;
    }
  }

  inline bool tryResume(std::uint64_t expectedGeneration, unsigned choice) noexcept
  {
    if(!enabled() || bodyRecoveryPending() || (choice != 1 && choice != 2)) return false;
    auto& packed = detail::remoteStandStorage().packed;
    auto previous = packed.load();
    const auto required = choice == 1 ? detail::manualBit : detail::automaticBit;
    if((previous >> detail::generationShift) != expectedGeneration || (previous & 14u) != required
       || !(previous & (detail::holdBit | detail::takeoverFenceBit))
       || (automaticContinuityEnabled() && !(previous & detail::choiceReleasedBit))) return false;
    const auto next = (previous & ~detail::flagMask) | (automaticContinuityEnabled()
        ? static_cast<std::uint64_t>(choice == 1 ? Owner::manual : Owner::automatic) << detail::ownerShift : 0u);
    return packed.compare_exchange_strong(previous, next);
  }

  /** Only the physical Pickup gate calls this after fresh WALK and real stable Stand.
   * It is not a synthetic LT choice and cannot clear a concurrent new REMOVE.
   */
  inline bool resumeFromPhysicalPickup(std::uint64_t expectedGeneration) noexcept
  {
    if(!enabled()) return true;
    auto& value = detail::remoteStandStorage();
    auto previous = value.packed.load();
    if((previous >> detail::generationShift) != expectedGeneration) return false;
    const auto next = (previous & ~detail::flagMask) | (automaticContinuityEnabled()
        ? static_cast<std::uint64_t>(Owner::automatic) << detail::ownerShift : 0u);
    if(!value.packed.compare_exchange_strong(previous, next)) return false;
    value.awaitingRecoveryStand.store(false);
    return true;
  }
}
