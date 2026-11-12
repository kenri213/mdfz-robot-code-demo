#pragma once
// SIM-only physical-path emulation (see tools/apply_physical_emulation.py).
#include <atomic>
#include <cstdlib>
#include <cstring>
#include "Platform/SystemCall.h"
#include "WhiteFleetSimContext.h"
#include "WhiteFleetK1V21MatchProfile.h"
#include "WhiteFleetK1HotChase.h"
#include "WhiteFleetOpponentGoalKickGuard.h"
#include "WhiteFleetBoosterOfficialLiveTest.h"

namespace WhiteFleetSimPhysical
{
  inline bool requested() noexcept
  {
    static const bool on = []
    {
      const char* v = std::getenv("MDFZ_SIM_EMULATE_PHYSICAL");
      return v && !std::strcmp(v, "1");
    }();
    return on;
  }
  /** True only in the simulator, for team 5, with MDFZ_SIM_EMULATE_PHYSICAL=1. */
  inline bool emulate() noexcept
  {
    if(!requested() || SystemCall::getMode() != SystemCall::simulatedRobot)
      return false;
    const auto k = WhiteFleetSimContext::current();
    return k.simulated && k.team == 5;
  }
  inline bool physical() noexcept {return SystemCall::getMode() == SystemCall::physicalRobot || emulate();}
  inline bool simulated() noexcept {return SystemCall::getMode() == SystemCall::simulatedRobot && !emulate();}
  /** F1 body authority: in emulation the operator is assumed to have granted F1 (normal match). */
  inline bool bodyAuthorized() noexcept {return emulate() || WhiteFleetBoosterOfficialLiveTest::bodyAuthorized();}
  /** Same enables as Main.cpp guarded4a2Entry on the real robots (F1/remote layers excluded). */
  inline void ensureProfile() noexcept
  {
    if(!emulate())
      return;
    WhiteFleetK1V21MatchProfile::enableGuardedPhysicalReceiver();
    WhiteFleetK1HotChase::enable();
    WhiteFleetK1HotChase::enableReacquire();
    WhiteFleetK1HotChase::enableLogReacquire();
    WhiteFleetOpponentGoalKickGuard::enable();
  }
}
