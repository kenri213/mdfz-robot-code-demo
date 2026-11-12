/**
 * @file CalibrationFinished.cpp
 *
 * This file implements the CalibrationFinished skill.
 *
 * @author Arne Hasselbring
 */

#include "WhiteFleetSkillBehaviorControl.h"

option((WhiteFleetSkillBehaviorControl) CalibrationFinished)
{
  initial_state(execute)
  {
    action
    {
      theBehaviorStatus.calibrationFinished = true;
      theLibCheck.inc(LibCheck::calibrationFinished);
    }
  }
}
