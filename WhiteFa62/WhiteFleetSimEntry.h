#pragma once
#include "WhiteFleetSimContext.h"
#include "WhiteFleetK1V21MatchProfile.h"
#include "WhiteFleetK1HotChase.h"
#include "WhiteFleetOpponentGoalKickGuard.h"
#include "WhiteFleetBoosterMatchSafety.h"
#include "WhiteFleetBoosterOfficialLiveTest.h"
#include "WhiteFleetBoosterRemoteStand.h"
#include <cstdio>
#ifndef FLEET_SCOPE_TEST
#include "Representations/Configuration/BehaviorParameters.h"
#include "WhiteFleetMatchTacticalRepair.h"
#include "WhiteFleetC15KickoffSwitch.h"
#endif
namespace WhiteFleetSimEntry
{
 inline void activate()
 {
  if(!WhiteFleetSimContext::entryAllowed())return;
  WhiteFleetK1V21MatchProfile::enable();WhiteFleetK1HotChase::enable();WhiteFleetK1HotChase::enableReacquire();WhiteFleetK1HotChase::enableLogReacquire();WhiteFleetOpponentGoalKickGuard::enable();
  // No physical F1, SDK body authority, remote owner or manual-recovery permit.
 }
 inline void witness(const char* provider)
 {
  const auto k=WhiteFleetSimContext::current();if(!k.simulated)return;
  std::printf("FLEET_IMPLEMENTATION team=%d player=%d provider=%s head=3d11ae0363afca121f55a586ed7221a71e1c9577\n",k.team,k.player,provider);
  std::printf("FLEET_SIM_PROFILE team=%d player=%d slot=%u v21=%d hot=%d reacquire=%d log_gaze=%d goalkick=%d physical_body_authority=%d official_f1=%d remote_owner=%d\n",k.team,k.player,WhiteFleetSimContext::index(),WhiteFleetK1V21MatchProfile::enabled(),WhiteFleetK1HotChase::enabled(),WhiteFleetK1HotChase::reacquireEnabled(),WhiteFleetK1HotChase::logReacquireEnabled(),WhiteFleetOpponentGoalKickGuard::enabled(),WhiteFleetBoosterMatchSafety::enabled(),WhiteFleetBoosterOfficialLiveTest::enabled(),WhiteFleetBoosterRemoteStand::enabled());std::fflush(stdout);
 }
#ifndef FLEET_SCOPE_TEST
 inline void tactical(const BehaviorParameters& p)
 {
  const auto k=WhiteFleetSimContext::current();if(!k.simulated||k.team!=5)return;
  std::printf("FLEET_TACTICAL_PROFILE team=%d player=%d enabled=%d outside_mm=%.3f buffer_mm=%.3f corner_mm=%.3f\n",k.team,k.player,WhiteFleetMatchTacticalRepair::enabled(p.matchTacticalRepair),p.keeperPursuitOutsidePenalty,p.keeperPursuitBuffer,p.kickoffCornerOutside);std::fflush(stdout);
  const auto c15=WhiteFleetC15KickoffSwitch::throughParams(p.c15EarlyStart,p.c15RestoreProvenPool);
  std::printf("FLEET_PROGRESS_PROFILE team=%d player=%d early=%d outlet=%d support=%d legal_preload=%d prepare_x=%.1f enabled=%d\n",k.team,k.player,p.c15EarlyStart,p.backfieldStrongOutlet,p.opponentSidePosition,c15.legalPreload,c15.prepareX,WhiteFleetMatchTacticalRepair::enabled(p.matchTacticalRepair));std::fflush(stdout);

  std::printf("FLEET_C15_PROFILE team=%d player=%d configured=%d effective=%d kicker=%d receiver=%d side=%d early=%d deep=%d native_receive=%d prepare_ms=%u total_ms=%u\n",k.team,k.player,p.c15Kickoff,WhiteFleetC15KickoffSwitch::enabled(p.matchTacticalRepair&&p.c15Kickoff),c15.kicker,c15.kicker==3?2:3,c15.side,c15.c14EarlyStart,c15.c15DeepPass,c15.nativeReceive,c15.maxPrepare,c15.total);std::fflush(stdout);

 }
#endif

}
