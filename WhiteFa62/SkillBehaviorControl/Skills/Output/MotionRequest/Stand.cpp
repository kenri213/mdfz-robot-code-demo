/**
 * @file Stand.cpp
 *
 * This file implements the Stand skill.
 *
 * @author Arne Hasselbring
 */

#include "WhiteFleetSkillBehaviorControl.h"

option((WhiteFleetSkillBehaviorControl) Stand)
{
  theMotionRequest.motion = MotionRequest::stand;
  theLibCheck.inc(LibCheck::motionRequest);

  initial_state(execute)
  {
    transition
    {
      if(theMotionInfo.executedPhase == MotionPhase::stand)
        goto done;
    }
  }

  target_state(done) {}
}
