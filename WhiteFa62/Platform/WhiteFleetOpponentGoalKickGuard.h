#include "WhiteFleetSimContext.h"
#pragma once
#include <atomic>
#include <cmath>

/** Referee-owned restart protection. No SDK, key, pose or ball-model writes. */
namespace WhiteFleetOpponentGoalKickGuard
{
  namespace detail
  {
    struct Shared {std::atomic<bool> enabled{false}, held{false};};
    inline Shared& shared() noexcept {return WhiteFleetSimContext::storage<Shared,111>();}
  }
  inline void enable() noexcept {detail::shared().enabled.store(true);}
  inline bool enabled() noexcept {return detail::shared().enabled.load();}
  inline void publish(bool held) noexcept {detail::shared().held.store(held);}
  inline bool held() noexcept {return enabled() && detail::shared().held.load();}

  struct Input
  {
    unsigned now=0,ballSeen=0;
    bool packetUsable=false,opponentGoalKick=false,stopped=false;
    bool standingStable=false,finiteBall=false,nativeMoved=false;
    float ballSpeed=0.f;
    unsigned secondaryTime=0;
  };
  class State
  {
  public:
    bool active()const noexcept{return active_;}
    bool held()const noexcept{return active_&&!released_;}
    bool releasedByMotion()const noexcept{return active_&&released_;}
    unsigned startedAt()const noexcept{return started_;}
    void reset()noexcept{*this=State{};}
    bool update(const Input& in) noexcept
    {
      if(!active_ && in.opponentGoalKick)
      {active_=true;started_=in.now;lastClock_=in.secondaryTime;}
      // A missing/old/invalid packet is never a Ball Free command.
      if(!in.packetUsable)
      {clearEvidence();return held();}
      if(!in.opponentGoalKick)
      {reset();return false;}
      // STOP or an increased referee restart clock creates a new protected
      // epoch. Expiry of a LOCAL timer or an illegal placement never releases.
      if(in.stopped || wasStopped_ || in.secondaryTime>lastClock_+1u)
      {released_=false;started_=in.now;clearEvidence();}
      wasStopped_=in.stopped;lastClock_=in.secondaryTime;
      if(in.stopped || released_)return held();
      const bool fresh=in.now && in.ballSeen && in.ballSeen<=in.now &&
        in.now-in.ballSeen<=200 && in.ballSeen>started_;
      if(!in.standingStable || !in.finiteBall || !std::isfinite(in.ballSpeed) || in.ballSpeed<0.f || !fresh)
      {clearEvidence();return held();}
      // Re-reading one observation cannot prove movement. The robot must be
      // in real stable Stand: its own approach/turn is not a ball displacement.
      if(in.ballSeen<=lastSeen_)return held();
      if(lastSeen_ && in.ballSeen-lastSeen_>250)clearEvidence();
      lastSeen_=in.ballSeen;
      if(!armed_)
      {
        if(in.ballSpeed<=150.f)
        {
          if(!stationaryCount_)stationarySince_=in.ballSeen;
          ++stationaryCount_;
          if(stationaryCount_>=3 && in.ballSeen-stationarySince_>=200)armed_=true;
        }
        else stationaryCount_=stationarySince_=0;
        return held();
      }
      // Require the existing six-sample native motion proof PLUS multiple
      // fresh, nonzero-speed observations after a stationary-ball anchor.
      if(in.nativeMoved && in.ballSpeed>300.f)
      {
        if(!motionCount_)motionSince_=in.ballSeen;
        ++motionCount_;
        if(motionCount_>=3 && in.ballSeen-motionSince_>=100)released_=true;
      }
      else motionCount_=motionSince_=0;
      return held();
    }
  private:
    void clearEvidence()noexcept
    {armed_=false;lastSeen_=stationarySince_=stationaryCount_=motionSince_=motionCount_=0;}
    bool active_=false,released_=false,wasStopped_=false,armed_=false;
    unsigned started_=0,lastClock_=0,lastSeen_=0,stationarySince_=0,stationaryCount_=0,motionSince_=0,motionCount_=0;
  };
}
