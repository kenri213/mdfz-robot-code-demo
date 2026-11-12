#pragma once
#include "WhiteFleetMatchTacticalRepair.h"
#include "WhiteFleetKickoffThroughV8Math.h"

/** Deliberate fleet adapter, not the old simulation/team-5 experiment switch.
 * Physical use still requires Main's existing SN + exact-entry V2.1 profile;
 * environment variables cannot authorize a physical robot. No startup changes.
 */
namespace WhiteFleetC15KickoffSwitch
{
  inline bool enabled(bool configured)
  {return WhiteFleetMatchTacticalRepair::enabled(configured);}
  inline WhiteFleetKickoffThroughV8::Params throughParams(bool early=false,bool restore=false,bool adaptive=false,bool inside=false,bool race=false)
  {
    WhiteFleetKickoffThroughV8::Params p;
    p.kicker=3;p.side=-1;
    p.calibrated=p.adaptiveMeet=p.playbook=p.isolated=p.nativeReceive=true;
    p.c14EarlyStart=p.c15DeepPass=true;
    p.restoreProvenPool=restore;
    p.defenceAdaptiveAngle=adaptive&&restore;
    p.insideCornerLane=inside&&p.defenceAdaptiveAngle;
    p.c12RightMinDegrees=40.f;p.c12ReserveMin=0.f;
    p.legalPreload=early;
    if(early)p.prepareX=-1000.f; // More legal runway; P, kick mapping and release unchanged.
    p.raceRendezvous=race&&p.defenceAdaptiveAngle;
    if(p.raceRendezvous){p.prepareX=-850.f;p.prepareYFraction=.80f;p.legalPreload=false;}
    return p; // Exact successful C15 profile; ordinary speed/mapping unchanged.
  }
  inline WhiteFleetKickoffThroughV8::Params directCenterParams()
  {
    WhiteFleetKickoffThroughV8::Params p;
    p.kicker=3;p.side=-1;p.directCenter=true;
    p.calibrated=p.adaptiveMeet=p.playbook=p.isolated=p.c14EarlyStart=true;
    p.prepareX=-850.f;p.prepareYFraction=.15f;p.headBodyYaw=0.f;
    return p; // Normal Dynamic mapping/real speed limits and original budgets.
  }
  inline WhiteFleetKickoffThroughV8::Params configuredParams(bool early,bool restore,bool adaptive,bool inside,bool race,bool direct)
  {return direct?directCenterParams():throughParams(early,restore,adaptive,inside,race);}
  inline WhiteFleetKickoffThroughV8::Params centerFastParams()
  {
    auto p=directCenterParams();p.centerFast=true;p.prepareX=-750.f;
    // C1 measured roll deceleration ~.416m/s^2 and clear active approach1.411m/s.
    // Prediction only, not a change to physical friction or actuator speed.
    p.exitRatio=1.01f;p.rollRatio=.755f;p.predictionDeceleration=416.f;p.empiricalReceiverSpeed=1400.f;
    return p;
  }
  inline WhiteFleetKickoffThroughV8::Params configuredFastParams(bool early,bool restore,bool adaptive,bool inside,bool race,bool direct,bool fast)
  {return fast?centerFastParams():configuredParams(early,restore,adaptive,inside,race,direct);}
  inline WhiteFleetKickoffThroughV8::Params receiverStartParams()
  {auto p=centerFastParams();p.receiverPrepareHold=true;return p;}
  inline WhiteFleetKickoffThroughV8::Params configuredReceiverStartParams(bool early,bool restore,bool adaptive,bool inside,bool race,bool direct,bool fast,bool hold)
  {return hold?receiverStartParams():configuredFastParams(early,restore,adaptive,inside,race,direct,fast);}
  inline WhiteFleetKickoffThroughV8::Params receiverApproachParams()
  {auto p=receiverStartParams();p.receiverBallApproach=true;return p;}
  inline WhiteFleetKickoffThroughV8::Params configuredReceiverApproachParams(bool early,bool restore,bool adaptive,bool inside,bool race,bool direct,bool fast,bool hold,bool approach)
  {return approach?receiverApproachParams():configuredReceiverStartParams(early,restore,adaptive,inside,race,direct,fast,hold);}
  inline WhiteFleetKickoffThroughV8::Params followFromReleaseParams()
  {auto p=receiverApproachParams();p.receiverFollowFromRelease=true;return p;}
  inline WhiteFleetKickoffThroughV8::Params configuredFollowFromReleaseParams(bool early,bool restore,bool adaptive,bool inside,bool race,bool direct,bool fast,bool hold,bool approach,bool follow)
  {return follow?followFromReleaseParams():configuredReceiverApproachParams(early,restore,adaptive,inside,race,direct,fast,hold,approach);}
  inline WhiteFleetKickoffThroughV8::Params stableKickAimParams()
  {auto p=followFromReleaseParams();p.stableKickAim=true;return p;}
  inline WhiteFleetKickoffThroughV8::Params configuredStableKickAimParams(bool early,bool restore,bool adaptive,bool inside,bool race,bool direct,bool fast,bool hold,bool approach,bool follow,bool stable)
  {return stable?stableKickAimParams():configuredFollowFromReleaseParams(early,restore,adaptive,inside,race,direct,fast,hold,approach,follow);}
  inline WhiteFleetKickoffThroughV8::Params earlyNativeParams()
  {auto p=stableKickAimParams();p.receiverEarlyNative=true;return p;}
  inline WhiteFleetKickoffThroughV8::Params configuredEarlyNativeParams(bool early,bool restore,bool adaptive,bool inside,bool race,bool direct,bool fast,bool hold,bool approach,bool follow,bool stable,bool earlyNative)
  {return earlyNative?earlyNativeParams():configuredStableKickAimParams(early,restore,adaptive,inside,race,direct,fast,hold,approach,follow,stable);}
  inline WhiteFleetKickoffThroughV8::Params gapWalkParams()
  {auto p=earlyNativeParams();p.receiverGapWalk=true;return p;}
  inline WhiteFleetKickoffThroughV8::Params configuredGapWalkParams(bool early,bool restore,bool adaptive,bool inside,bool race,bool direct,bool fast,bool hold,bool approach,bool follow,bool stable,bool earlyNative,bool gapWalk)
  {return gapWalk?gapWalkParams():configuredEarlyNativeParams(early,restore,adaptive,inside,race,direct,fast,hold,approach,follow,stable,earlyNative);}
  inline WhiteFleetKickoffThroughV8::Params firstPassClearanceParams()
  {auto p=gapWalkParams();p.firstPassClearance=true;return p;}
  inline WhiteFleetKickoffThroughV8::Params configuredFirstPassClearanceParams(bool early,bool restore,bool adaptive,bool inside,bool race,bool direct,bool fast,bool hold,bool approach,bool follow,bool stable,bool earlyNative,bool gapWalk,bool clearance)
  {return clearance?firstPassClearanceParams():configuredGapWalkParams(early,restore,adaptive,inside,race,direct,fast,hold,approach,follow,stable,earlyNative,gapWalk);}
  inline WhiteFleetKickoffThroughV8::Params forwardPrepareParams()
  {auto p=firstPassClearanceParams();p.firstPassPrepareAdvance=250.f;p.prepareX+=p.firstPassPrepareAdvance;return p;}
  inline WhiteFleetKickoffThroughV8::Params configuredForwardPrepareParams(bool early,bool restore,bool adaptive,bool inside,bool race,bool direct,bool fast,bool hold,bool approach,bool follow,bool stable,bool earlyNative,bool gapWalk,bool clearance,bool forward)
  {return forward?forwardPrepareParams():configuredFirstPassClearanceParams(early,restore,adaptive,inside,race,direct,fast,hold,approach,follow,stable,earlyNative,gapWalk,clearance);}
}
