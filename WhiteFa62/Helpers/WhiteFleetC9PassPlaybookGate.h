#pragma once
namespace WhiteFleetC9PassPlaybook
{
  // Routing ownership is independent of plan availability, request metadata,
  // or valid dependencies. Missing/invalid plans cannot select another kicker.
  inline bool ownsKickoff(bool requested,bool ownKickoff,bool released=false,unsigned releasedEvent=0,unsigned currentEvent=0)
  {return requested&&ownKickoff&&!(released&&releasedEvent>0&&releasedEvent==currentEvent);}
}
