/** Fixed full-application commissioning root: no Prepare, Walk, GetUp, or kick. */
#include "WhiteFleetSkillBehaviorControl.h"

option((WhiteFleetSkillBehaviorControl) CommissionPlayDead)
{
  initial_state(inactive)
  {
    action
    {
      LookAtAngles({.pan = JointAngles::off,
                    .tilt = JointAngles::off});
      PlayDead();
    }
  }
}
