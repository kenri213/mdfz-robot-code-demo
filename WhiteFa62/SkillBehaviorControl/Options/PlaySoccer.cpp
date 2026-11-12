/**
 * @file PlaySoccer.h
 *
 * This file defines the root option of the soccer behavior.
 *
 * @author Thomas Röfer
 */

#include "WhiteFleetSkillBehaviorControl.h"

option((WhiteFleetSkillBehaviorControl) PlaySoccer,
       load((std::vector<cabsl::Cabsl<WhiteFleetSkillBehaviorControl>::OptionInfos::Option>) options))
{
  initial_state(selectOption)
  {
    action
    {
      select_option(options);
    }
  }
}
