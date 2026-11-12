#pragma once
#include "WhiteFleetDefenseCounterMath.h"
#include "WhiteFleetSimPhysical.h"
#include "Framework/Settings.h"
#include "Platform/SystemCall.h"
#include "WhiteFleetK1V21MatchProfile.h"
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace WhiteFleetDefenseCounterSwitch
{
  inline bool enabled()
  {
    if(WhiteFleetSimPhysical::physical())
      return WhiteFleetK1V21MatchProfile::enabled();
    const char* flag=std::getenv("MDFZ_DEFENSE_COUNTER_V2_ENABLE");
    return WhiteFleetDefenseCounterMath::active(flag && std::strcmp(flag,"1")==0,
      WhiteFleetSimPhysical::simulated(),Global::getSettings().teamNumber);
  }
  inline bool keeperPredictionEnabled()
  {
    if(WhiteFleetSimPhysical::physical()) return false;
    const char* flag=std::getenv("MDFZ_DEFENSE_GK_PREDICT_ENABLE");return enabled()&&flag&&std::strcmp(flag,"1")==0;
  }
  inline bool recoveryEnabled()
  {
    if(WhiteFleetSimPhysical::physical()) return false;
    const char* flag=std::getenv("MDFZ_DEFENSE_RECOVERY_SCREEN_ENABLE");return enabled()&&flag&&std::strcmp(flag,"1")==0;
  }
  // Summary counters only, no replay, world-state history or truth inputs.
  inline void record(unsigned mode)
  {
    // Physical Cognition never performs synchronous diagnostic file I/O here.
    if(WhiteFleetSimPhysical::physical()) return;
    const char* dir=std::getenv("MDFZ_COUNTER_DIAGNOSTICS_DIR");
    if(!dir || !enabled()) return;
    thread_local std::array<unsigned long,7> counts{};
    thread_local unsigned calls=0;
    ++counts[std::min(mode,6u)];
    if(++calls%50) return;
    const std::string file=std::string(dir)+"/white_fa62_defense_v2_player"+std::to_string(Global::getSettings().playerNumber)+".json";
    FILE* out=std::fopen((file+".tmp").c_str(),"w");
    if(!out) return;
    std::fprintf(out,"{\"sampled_frames\":%u,\"fallback\":%lu,\"keeper_ray\":%lu,\"keeper_prediction\":%lu,\"kickoff_screen\":%lu,\"recovery_screen\":%lu,\"counter_outlet\":%lu,\"keeper_control\":%lu}\n",
      calls,counts[0],counts[1],counts[2],counts[3],counts[4],counts[5],counts[6]);
    std::fclose(out);std::rename((file+".tmp").c_str(),file.c_str());
  }
}
