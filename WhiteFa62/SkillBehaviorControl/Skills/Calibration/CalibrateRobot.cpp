/**
 * @file CalibrateRobot.cpp
 *
 * @author Lukas Plecher
 */

#include "WhiteFleetSkillBehaviorControl.h"

option((WhiteFleetSkillBehaviorControl) CalibrateRobot,
       args((const CalibrationRequest&) request))
{
  initial_state(execute)
  {
    action
    {
      theCalibrationRequest = request;
    }
  }
}
