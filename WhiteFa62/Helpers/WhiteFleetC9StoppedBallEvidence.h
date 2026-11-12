#pragma once
#include "WhiteFleetKickoffThroughV8Math.h"

namespace WhiteFleetKickoffThroughV8
{
  /** A zero native velocity is only a candidate, not a stationary witness.
   * Count distinct own observations in their ORIGINAL time base. Never count
   * cached elapsed time or write a synthetic velocity into perception. */
  struct StoppedBallEvidence
  {
    unsigned firstSeen=0,lastSeen=0,samples=0;
    Point anchor{},lastPoint{};float path=0.f;
    const char* reason="empty";
    void reset(){*this=StoppedBallEvidence();}
    unsigned span()const{return lastSeen>=firstSeen?lastSeen-firstSeen:0u;}
    void start(unsigned seen,Point point,const char* why)
    {firstSeen=lastSeen=seen;samples=1;anchor=lastPoint=point;path=0.f;reason=why;}
    bool observe(unsigned now,unsigned seen,Point point)
    {
      if(!seen||seen>now||now-seen>250||!finite(point))
      {reset();reason="invalid_or_stale_observation";return false;}
      if(!firstSeen){start(seen,point,"first_observation");return false;}
      if(seen<lastSeen){start(seen,point,"observation_clock_regressed");return false;}
      if(seen==lastSeen){reason="cached_observation_not_evidence";return false;}
      if(seen-lastSeen>250||distance(point,anchor)>100.f)
      {start(seen,point,"gap_or_displaced_anchor");return false;}
      path+=distance(point,lastPoint);lastSeen=seen;lastPoint=point;++samples;
      if(span()<300||samples<3){reason="await_distinct_observation_span";return false;}
      // Independent observed travel must agree with the parent's <=100 mm/s
      // slow-ball hypothesis. The old 100 mm anchor alone allowed moving balls.
      if(path*1000.f>100.f*static_cast<float>(span()))
      {start(seen,point,"observed_motion_not_stationary");return false;}
      reason="stationary_observation_window";return true;
    }
  };
}
