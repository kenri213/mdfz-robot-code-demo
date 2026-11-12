#pragma once
namespace WhiteFleetC10Kickoff
{
  struct Phase
  {
    unsigned event=0,last=0;
    bool finished=false,clockFault=false;
    bool exitReported=false,refusalReported=false;
    bool ballDeparted=false;
    unsigned awaySeen=0;
    int lastAction=-1;
    const char* reason="inactive";
    void reset(){*this=Phase();}
    void finish(const char* why){finished=true;reason=why;}
    /** A failed C15 start must not become an unrelated first pass after the
     * referee's timed PLAY transition. Two distinct, fresh own observations
     * outside the centre region retire this refusal, not a kick/event proxy.
     * This records departure only: it is NOT evidence of our foot contact. */
    void observeDeparture(unsigned now,unsigned seen,bool trusted,bool away)
    {
      if(ballDeparted)return;
      if(!trusted||clockFault||!event||seen<=event||seen>now||now-seen>200||!away)
      {awaySeen=0;return;}
      if(awaySeen&&seen>awaySeen&&seen-awaySeen<=200)ballDeparted=true;
      else if(!awaySeen||seen<awaySeen||seen-awaySeen>200)awaySeen=seen;
    }
    bool tick(bool own,bool playing,unsigned nativeEvent,unsigned now,unsigned budget)
    {
      if(!own&&!playing){reset();return false;}
      if(own&&nativeEvent&&nativeEvent!=event){reset();event=nativeEvent;last=now;reason="isolated_kickoff";}
      if(!event)return false;
      if(now<event||now<last){clockFault=true;finish("clock_regressed");return true;}
      last=now;
      if(!finished&&now-event>budget)finish("isolated_total_deadline");
      return !finished;
    }
  };
  inline bool released(bool isolated,bool finished,unsigned event,unsigned ownEvent)
  {return isolated&&finished&&event>0&&event==ownEvent;}
}
