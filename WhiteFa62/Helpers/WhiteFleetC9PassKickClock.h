#pragma once

#include <algorithm>

/** One scoped MotionInfo event channel, not a global clock or timestamp fix.
 * Future raw values are retained as pending and never clamped to now. */
namespace WhiteFleetC9PassKickClock
{
  constexpr unsigned maximumLead=250,maximumWait=250;
  enum State : unsigned {idle,ready,pending,caughtUp,rejected,afterTerminal,caughtUpPending};
  struct Result {State state=idle;bool reject=false;const char* reason="kick_clock_idle";};
  struct Clock
  {
    unsigned value=0,baseline=0,pendingValue=0,pendingAt=0,lastNow=0;
    bool baselineReady=false,observed=false;
    void seed(unsigned now,unsigned raw)
    {
      *this=Clock();lastNow=now;observed=true;
      if(raw<=now){value=baseline=raw;baselineReady=true;}
    }
    Result observe(unsigned now,unsigned raw,bool terminal=false)
    {
      if(terminal)return {afterTerminal,false,"kick_clock_after_terminal_ignored"};
      if(observed&&now<lastNow)return {rejected,true,"local_kick_clock_backwards"};
      lastNow=now;observed=true;
      if(pendingAt&&now-pendingAt>maximumWait)return {rejected,true,"local_kick_wait_expired"};
      if(raw>now&&raw-now>maximumLead)return {rejected,true,"local_kick_gross_future"};
      if(baselineReady&&raw<value)return {rejected,true,"local_kick_regressed"};
      if(pendingAt)
      {
        // The original pending event is not replaced by newer future values.
        // A monotonic newer snapshot corroborates it once its ORIGINAL time
        // becomes past; continuous small producer lead must not starve proof.
        if(now<pendingValue||raw<pendingValue)return {pending,false,"local_kick_pending_reconfirmation"};
        const unsigned accepted=raw<=now?raw:pendingValue;
        if(!baselineReady){baseline=accepted;baselineReady=true;}
        value=accepted;pendingAt=pendingValue=0;
        if(raw>now)
        {pendingAt=now;pendingValue=raw;return {caughtUpPending,false,"local_kick_past_verified_newer_pending"};}
        return {caughtUp,false,"local_kick_time_caught_up"};
      }
      if(raw>now)
      {pendingAt=now;pendingValue=raw;return {pending,false,"local_kick_pending"};}
      if(!baselineReady){baseline=raw;baselineReady=true;}
      value=raw;pendingValue=pendingAt=0;
      return {ready,false,"local_kick_ready"};
    }
  };
}
