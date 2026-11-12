#include "WhiteFleetSimContext.h"
/** Opt-in match entry permission; never a replacement for physical F1/X safety. */
#pragma once

#include <atomic>
#include <cmath>
#include <cstdint>
#pragma push_macro("state")
#undef state
#include <string>
#pragma pop_macro("state")

namespace WhiteFleetBoosterMatchSafety
{
  struct VisualEvidence
  {
    std::uint64_t epoch = 0;
    unsigned frameTime = 0, consecutiveFrames = 0, poses = 0, landmarks = 0, lines = 0;
  };
  namespace detail
  {
    struct MatchState
    {
      std::atomic<bool> active{false}, rawInitial{false};
      std::atomic<bool> stoppedManualGetUp{false};
      std::atomic<bool> independentManualBodyRecovery{false};
      std::atomic<bool> getUpJoystickPending{false};
      // Zero is deliberately not a configured source. Match mode stays closed
      // until Main has validated its single runtime settings value.
      std::atomic<std::uint32_t> gameControllerAddress{0};
      std::atomic<std::uint64_t> permission{2}, confirmations{0}, joystickReleaseGeneration{0}; // epoch << 1 | confirmed.
      std::atomic<std::uint64_t> visualEpoch{0};
      std::atomic<unsigned> visualFrame{0}, visualCount{0}, poses{0}, landmarks{0}, lines{0};
    };
    inline MatchState& matchSafetyStorage() {return WhiteFleetSimContext::storage<MatchState,102>();}
    inline bool parseGameControllerSource(const std::string& text, std::uint32_t& address) noexcept
    {
      if(text.size() < 7 || text.size() > 15) return false;
      std::size_t offset = 0;
      std::uint32_t parsed = 0;
      for(unsigned part = 0; part < 4; ++part)
      {
        const std::size_t start = offset;
        unsigned octet = 0;
        while(offset < text.size() && text[offset] >= '0' && text[offset] <= '9')
        {
          if(offset - start == 3) return false;
          octet = octet * 10 + static_cast<unsigned>(text[offset] - '0');
          if(octet > 255) return false;
          ++offset;
        }
        if(offset == start || (offset - start > 1 && text[start] == '0')) return false;
        parsed = (parsed << 8) | octet;
        if(part < 3)
        {
          if(offset == text.size() || text[offset] != '.') return false;
          ++offset;
        }
        else if(offset != text.size()) return false;
      }
      const auto first = parsed >> 24;
      // No unspecified/this-network, loopback, multicast, class-E/reserved,
      // or limited-broadcast source. Subnet-directed broadcasts require the
      // actual subnet mask; this parser never guesses or changes that mask.
      if(first == 0 || first == 127 || first >= 224) return false;
      address = parsed;
      return true;
    }
  }
  inline bool enabled() noexcept {return detail::matchSafetyStorage().active.load();}
  inline void enable() noexcept {detail::matchSafetyStorage().active.store(true);}
  /** Body recovery only: never changes raw GC state or authorizes football. */
  inline void enableStoppedManualGetUp() noexcept {detail::matchSafetyStorage().stoppedManualGetUp.store(true);}
  inline bool stoppedManualGetUpEnabled() noexcept
  {return enabled() && detail::matchSafetyStorage().stoppedManualGetUp.load();}
  /** Explicit physical release only. Does not grant locomotion or clear penalties. */
  inline void enableIndependentManualBodyRecovery() noexcept
  {detail::matchSafetyStorage().independentManualBodyRecovery.store(true);}
  inline bool independentManualBodyRecoveryEnabled() noexcept
  {return enabled() && detail::matchSafetyStorage().independentManualBodyRecovery.load();}
  /** Freeze one source for this process. Repeating the SAME value is harmless;
   * an invalid value or any attempt to hot-switch the policy changes nothing.
   */
  inline bool configureGameControllerSource(const std::string& text) noexcept
  {
    std::uint32_t address = 0;
    if(!detail::parseGameControllerSource(text, address)) return false;
    auto& value = detail::matchSafetyStorage().gameControllerAddress;
    std::uint32_t expected = 0;
    return value.compare_exchange_strong(expected, address, std::memory_order_release, std::memory_order_relaxed)
           || expected == address;
  }
  inline std::uint32_t gameControllerSource() noexcept
  {return detail::matchSafetyStorage().gameControllerAddress.load(std::memory_order_acquire);}
  /** Release placement control ONCE, not a permanent ban on later LT-plus. */
  class InitialHandoff
  {
  public:
    bool update(bool matchMode, bool gcFresh, bool initial) noexcept
    {
      if(!matchMode || !gcFresh) return false;
      const bool previousInitial = wasInitial.exchange(initial);
      return previousInitial && !initial;
    }
    void arm() noexcept {wasInitial.store(true);}
    void disarm() noexcept {wasInitial.store(false);}
  private:
    std::atomic<bool> wasInitial{true};
  };
  // Unlike CABSL vars this survives returning to the option's inactive state.
  // Only the Cognition behavior thread reads/writes this edge detector.
  inline InitialHandoff& initialHandoff() noexcept
  {return WhiteFleetSimContext::storage<InitialHandoff,103>();}
  inline bool acceptGameControllerSource(unsigned hostOrderAddress) noexcept
  {
    if(!enabled()) return true; // Preserve the unmodified official/non-match path.
    const auto source = gameControllerSource();
    return source != 0 && hostOrderAddress == source;
  }
  inline std::uint64_t epoch() noexcept {return detail::matchSafetyStorage().permission.load() >> 1;}
  inline bool confirmed() noexcept
  {return enabled() && (detail::matchSafetyStorage().permission.load() & 1u) != 0;}
  /** One request per accepted fresh live get-up; never a permanent manual default. */
  inline void requestJoystickAfterGetUp() noexcept
  {if(enabled()) detail::matchSafetyStorage().getUpJoystickPending.store(true);}
  inline void cancelJoystickAfterGetUp() noexcept
  {detail::matchSafetyStorage().getUpJoystickPending.store(false);}
  inline bool consumeJoystickAfterGetUp(bool ready) noexcept
  {return enabled() && ready && detail::matchSafetyStorage().getUpJoystickPending.exchange(false);}
  inline void invalidateConfirmation() noexcept
  {
    if(!enabled()) return;
    cancelJoystickAfterGetUp();
    auto& permission = detail::matchSafetyStorage().permission;
    auto previous = permission.load();
    while(!permission.compare_exchange_weak(previous, ((previous >> 1) + 1) << 1)) {}
  }
  inline void noteGcInitial(bool initial) noexcept
  {
    if(enabled() && detail::matchSafetyStorage().rawInitial.exchange(initial) != initial && initial)
    {
      // Physical/get-up may hide HandleJoystick for the whole INITIAL interval.
      // A real GC transition still arms exactly one later automatic handoff.
      initialHandoff().arm();
      invalidateConfirmation();
    }
  }
  inline bool confirm(std::uint64_t expectedEpoch) noexcept
  {
    if(!enabled()) return false;
    std::uint64_t expected = expectedEpoch << 1;
    if(!detail::matchSafetyStorage().permission.compare_exchange_strong(expected, expected | 1u)) return false;
    cancelJoystickAfterGetUp();
    detail::matchSafetyStorage().confirmations.fetch_add(1);
    return true;
  }
  inline std::uint64_t confirmationGeneration() noexcept
  {return detail::matchSafetyStorage().confirmations.load();}
  inline bool consumeJoystickRelease() noexcept
  {
    if(!confirmed()) return false;
    auto& value = detail::matchSafetyStorage();
    const auto generation = value.confirmations.load();
    auto previous = value.joystickReleaseGeneration.load();
    while(generation > previous)
      if(value.joystickReleaseGeneration.compare_exchange_weak(previous, generation))
      {
        cancelJoystickAfterGetUp();
        return true;
      }
    return false;
  }

  /** Diagnostics only: count distinct real registration frames, not particles or INITIAL templates. */
  inline void noteVisualRegistration(unsigned frameTime, bool initial, unsigned poses, unsigned landmarks, unsigned lines) noexcept
  {
    if(!enabled()) return;
    auto& value = detail::matchSafetyStorage();
    const auto currentEpoch = epoch();
    const unsigned previousFrame = value.visualFrame.load();
    const bool sameEpoch = value.visualEpoch.load() == currentEpoch;
    if(sameEpoch && frameTime <= previousFrame) return;
    const bool observed = !initial && frameTime != 0 && (poses || landmarks || lines);
    // A transient featureless image is not contrary pose evidence. Keep the
    // LAST REAL registration timestamp, so the independent 500ms freshness
    // gate expires naturally; never turn an empty/repeated frame into evidence.
    if(!initial && !observed && sameEpoch && frameTime > previousFrame && frameTime - previousFrame <= 500)
      return;
    const unsigned previousCount = value.visualCount.load();
    const unsigned count = observed ? (sameEpoch && frameTime > previousFrame && frameTime - previousFrame <= 500
                                      ? previousCount + 1 : 1) : 0;
    value.visualCount.store(count);
    value.poses.store(poses); value.landmarks.store(landmarks); value.lines.store(lines);
    value.visualFrame.store(frameTime);
    value.visualEpoch.store(currentEpoch);
  }
  inline VisualEvidence visualEvidence() noexcept
  {
    const auto& value = detail::matchSafetyStorage();
    return {value.visualEpoch.load(), value.visualFrame.load(), value.visualCount.load(),
            value.poses.load(), value.landmarks.load(), value.lines.load()};
  }

  struct EligibilityInput
  {
    bool gcActive = false, readyOrSet = false, rawReadyOrSet = false, stopped = true, penalized = true;
    bool authorized = false, stableStand = false, poseQuality = false, poseFinite = false;
    unsigned now = 0, packetTime = 0, frameTime = 0;
    float positionStdMm = 0.f, headingStdRad = 0.f;
  };
  inline bool eligible(const EligibilityInput& input, const VisualEvidence& visual, std::uint64_t currentEpoch) noexcept
  {
    return input.gcActive && input.readyOrSet && input.rawReadyOrSet && !input.stopped && !input.penalized
           && input.authorized && input.stableStand && input.poseQuality && input.poseFinite
           && input.packetTime > 0 && input.now >= input.packetTime && input.now - input.packetTime <= 2000
           && visual.epoch == currentEpoch && visual.consecutiveFrames >= 6 && visual.frameTime > 0
           && input.frameTime >= visual.frameTime && input.frameTime - visual.frameTime <= 500
           && std::isfinite(input.positionStdMm) && input.positionStdMm >= 0.f && input.positionStdMm <= 500.f
           && std::isfinite(input.headingStdRad) && input.headingStdRad >= 0.f && input.headingStdRad <= 0.261799388f;
  }

  /** Only a neutral release AFTER eligibility may start a fresh human confirmation. */
  class ConfirmationGesture
  {
  public:
    bool update(std::uint64_t currentEpoch, unsigned now, bool candidateEligible, bool neutral, bool rbYOnly) noexcept
    {
      if(currentEpoch != lastEpoch || now <= lastFrame || !candidateEligible)
      {
        releaseSeen = holding = false;
        startedAt = 0;
      }
      lastEpoch = currentEpoch;
      lastFrame = now;
      if(!candidateEligible) return false;
      if(neutral) {releaseSeen = true; holding = false; startedAt = 0; return false;}
      if(!releaseSeen || !rbYOnly) {holding = false; startedAt = 0; return false;}
      if(!holding) {holding = true; startedAt = now; return false;}
      if(now - startedAt < 2000) return false;
      releaseSeen = holding = false;
      return true;
    }
  private:
    std::uint64_t lastEpoch = 0;
    unsigned lastFrame = 0, startedAt = 0;
    bool releaseSeen = false, holding = false;
  };
}
