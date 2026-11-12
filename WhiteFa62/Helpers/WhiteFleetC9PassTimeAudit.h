#pragma once

#include <cstring>

/** Read-only same-update evidence. No timestamp clamping, acceptance window,
 * motion command, global clock or team-message field is introduced here. */
namespace WhiteFleetC9PassTimeAudit
{
  enum Reject : unsigned
  {
    eventMissing=1u,
    eventFuture=2u,
    nowBackwards=4u,
    ownSeenFuture=8u,
    localKickFuture=16u,
    sentFrameFuture=32u,
  };

  constexpr unsigned inspect(unsigned now,unsigned previous,unsigned event,unsigned seen,unsigned kick,
                             bool sentPresent,unsigned sentStamp)
  {
    return (!event?eventMissing:0u)|(event>now?eventFuture:0u)|(now<previous?nowBackwards:0u)
        |(seen>now?ownSeenFuture:0u)|(kick>now?localKickFuture:0u)
        |(sentPresent&&sentStamp>now?sentFrameFuture:0u);
  }

  struct Snapshot
  {
    bool inspected=false,sentPresent=false;
    unsigned now=0,previous=0,event=0,seen=0,kick=0,sentStamp=0,mask=0,stageBefore=0;
    unsigned appliedMask=0,deferredMask=0,usableKick=0,pendingKick=0,kickWaitAt=0,kickClockState=0;
  };

  /** Bounded diagnostic selection: first event, each distinct six-bit mask,
   * then stage/request/reason/near-foot transitions. Repeated identical frame
   * data does not print every frame. This latch never affects the session. */
  struct Latch
  {
    unsigned event=0,lastStage=~0u,lastRequest=~0u,lastNear=~0u,lastPublication=~0u,lastPolicy=~0u;
    unsigned long long maskHistory=0;
    const char* lastReason=nullptr;
    bool select(const Snapshot& s,unsigned stageAfter,unsigned request,const char* reason,unsigned nearMask,unsigned publication=0,unsigned policy=0)
    {
      if(!s.inspected||s.mask>63u)return false;
      if(event!=s.event){*this=Latch();event=s.event;}
      const auto bit=1ULL<<s.mask;
      const bool emit=!(maskHistory&bit)||lastStage!=stageAfter||lastRequest!=request||lastNear!=nearMask||lastPublication!=publication||lastPolicy!=policy
          ||!lastReason||std::strcmp(lastReason,reason)!=0;
      maskHistory|=bit;lastStage=stageAfter;lastRequest=request;lastNear=nearMask;lastPublication=publication;lastPolicy=policy;lastReason=reason;
      return emit;
    }
  };
}
