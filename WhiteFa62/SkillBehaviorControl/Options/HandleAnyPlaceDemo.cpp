#include "WhiteFleetSkillBehaviorControl.h"

option((WhiteFleetSkillBehaviorControl) HandleAnyPlaceDemo)
{
  initial_state(initial)
  {
    action
    {
      DemoSoccer();
    }
  }
}
