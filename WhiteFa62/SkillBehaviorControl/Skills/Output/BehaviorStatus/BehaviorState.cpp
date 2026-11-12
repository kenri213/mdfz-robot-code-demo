/**
 * @file BehaviorState.cpp
 *
 * This file implements the BehaviorState skill.
 *
 * @author Philip Reichenberg
 */

#include "WhiteFleetSkillBehaviorControl.h"

option((WhiteFleetSkillBehaviorControl) BehaviorStateTransitionFailed)
{
  initial_state(execute)
  {
    action
    {
      theBehaviorStateInformation.transitionFailed = true;
      theLibCheck.inc(LibCheck::behaviorState);
    }
  }
}

option((WhiteFleetSkillBehaviorControl) BehaviorStateRemoteControlActive)
{
  initial_state(execute)
  {
    action
    {
      theBehaviorStateInformation.remoteControlActive = true;
      theLibCheck.inc(LibCheck::behaviorState);
    }
  }
}
