#pragma once
#include <array>
#include <cstdlib>
#include <cstring>
#ifdef TARGET_ROBOT
#error WhiteFleetSimContext is a desktop-only simulation adapter, not robot authorization
#endif
#ifndef FLEET_SCOPE_TEST
#include "Framework/Settings.h"
#include "Platform/SystemCall.h"
#endif
namespace WhiteFleetSimContext
{
 struct Key{bool simulated=false;int team=0,player=0;};
#ifdef FLEET_SCOPE_TEST
 inline thread_local Key testKey;
 inline Key current(){return testKey;}
#else
 inline Key current(){if(!Global::settingsExist())return {};return {SystemCall::getMode()==SystemCall::simulatedRobot,Global::getSettings().teamNumber,Global::getSettings().playerNumber};}
#endif
 inline unsigned index(){const auto k=current();return k.simulated&&k.player>=1&&k.player<=3&&(k.team==5||k.team==70)?unsigned((k.team==5?0:3)+k.player-1):6u;}
 template<typename T,int Tag> inline T& storage(){static std::array<T,7> values{};return values[index()];}
 inline bool entryAllowed(){const auto k=current();const char* p=std::getenv("MDFZ_FLEET_SIM_PROFILE");const char* mode=std::getenv("MDFZ_CLOSE_RUN_MODE");return k.simulated&&k.team==5&&k.player>=1&&k.player<=3&&p&&mode&&!std::strcmp(p,"901_vs_fa62")&&!std::strcmp(mode,"match");}
}
