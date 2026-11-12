/**
 * @file FallRequest.cpp
 *
 * This file implements the FallRequest skill.
 *
 * @author Philip Reichenberg
 */

#include "WhiteFleetSkillBehaviorControl.h"

option((WhiteFleetSkillBehaviorControl) FallRequest)
{
  theMotionRequest.motion = MotionRequest::fallRequest;
  theLibCheck.inc(LibCheck::motionRequest);

  initial_state(execute)
  {
    transition
    {
      if(theMotionInfo.executedPhase == MotionPhase::fall)
        goto done;
    }
  }

  target_state(done) {}
}
