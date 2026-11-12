#include "WhiteFleetSkillBehaviorControl.h"

option((WhiteFleetSkillBehaviorControl) HandlePlayingOptions,
       load((std::vector<cabsl::Cabsl<WhiteFleetSkillBehaviorControl>::OptionInfos::Option>) options))
{
  // The default state is "playing".
  initial_state(initial)
  {
    action
    {
      postKickPlayingVisited = true;
      if(!handlePostKickReacquire() && !select_option(options))
      {
        executeRequest();
        handleWalkSpeedAtFieldBorder();
      }
      publishRestartPassIntent();
    }
  }
}
