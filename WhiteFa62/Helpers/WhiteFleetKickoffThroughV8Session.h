#pragma once

#include "WhiteFleetKickoffThroughV8Math.h"
#include "WhiteFleetC9PassTimeAudit.h"
#include "WhiteFleetC9PassKickClock.h"
#include "WhiteFleetC9StoppedBallEvidence.h"
#include "WhiteFleetC9CaptureMotionEvidence.h"
#include "WhiteFleetKickoffAimAnchor.h"
#include "WhiteFleetC9ForwardFlank.h"
#include "WhiteFleetC15DefenceAdaptive.h"
#include "WhiteFleetC15RollingRendezvous.h"

namespace WhiteFleetKickoffThroughV8
{
  enum class Action : unsigned {none=0,hold=1,prepare=2,kick=3,run=4,support=5,finish=6,pursue=7,prestart=8,preload=9,gapWalk=10};
  enum class Stage : unsigned {idle,waiting,preparing,armed,flight,capture,settling,ended};
  constexpr float contractTolerance=300.f,captureDistance=650.f,captureTargetRadius=1500.f;
  constexpr unsigned retiredReceiveTail=10000; // unchanged native ordinary-announcement acceptance window
  constexpr unsigned kickCompletionWindow=1500,peerFutureWait=250;
  static_assert(contractTolerance==WhiteFleetC15DefenceAdaptive::wireTolerance);
  static_assert(kickCompletionWindow==WhiteFleetKickoffAimAnchor::maximumAge);

  /** Decoded existing wire fields, all points already transformed to FIELD.
   * healthy is GC-active/upright/valid-pose, NOT a new one-second message TTL.
   * passTarget=-1 + target is a future reception-point response, not a pass
   * back to the sender, successful reception or a new transmitted ACK token.
   */
  struct Peer
  {
    bool present=false,healthy=false;
    int number=-1,passTarget=-1;
    unsigned stamp=0,ballSeen=0,kickProxy=0;
    bool hasTarget=false,nativeOwner=false;
    Point target{},walkingTo{},position{},ball{};
    float speed=0.f;
    float heading=0.f;
  };
  struct Input
  {
    Params params{};Field field{};
    unsigned now=0,event=0,localSeen=0,localKick=0,peerTimeout=10000;
    int self=-1,receiver=-1;
    bool enabled=false,ownKickoff=false,playing=false,rosterValid=false;
    bool active=false,upright=false,poseValid=false,stopped=false,intercept=false;
    bool ballInsidePitch=false,planSafe=false,receiverReachable=false,trackingReady=false,nativeOwner=false;
    Point position{},ball{},ballRelative{};
    // Sender candidate before commitment; afterward only a re-evaluation of
    // the SAME target. Receiver mapping metadata is optional and local only.
    Plan plan{};
    Peer peer{},sent{};
    bool meetingValid=false,ballStoppedShort=false;
    Point meetingTarget{};
    bool timeDiagnostic=false;
    bool c9Coherence=false;
    bool fixedRunToArrival=false;unsigned fixedFootCommandAt=0;
    CaptureOdometry odometry;
    float odometrySpeedBound=0.f,odometryYawBound=0.f;
    float fieldHeading=std::numeric_limits<float>::quiet_NaN();
    float fieldHeadingVariance=std::numeric_limits<float>::quiet_NaN();
  };
  struct Directive
  {
    Action action=Action::none;Stage stage=Stage::idle;
    bool active=false,publishOffer=false,publishEcho=false;
    int sender=-1,receiver=-1;
    unsigned event=0;
    Plan plan{};
    const char* reason="inactive";
    bool suppressLegacyReceive=false;
    bool hasBodyTarget=false;
    Point bodyTarget{};
    bool aimValid=false;
    Point aimTargetOdometry{};
    unsigned aimFrame=0;
    bool gapValid=false;
    unsigned gapSeen=0,gapFrame=0;
    Point gapBallOdometry{},gapTargetOdometry{};
  };

  inline bool sessionFresh(unsigned now,unsigned stamp,unsigned limit)
  {return stamp>0&&now>=stamp&&now-stamp<=limit;}
  inline bool samePoint(Point a,Point b)
  {return finite(a)&&finite(b)&&distance(a,b)<=contractTolerance;}

  /** One instance per robot Strategy, not CABSL vars or a process-global lock.
   * Every deadline is anchored to the event/first stage entry, never to rereads
   * of a retained message. ownKickoff -> native playing is the sole carry-over.
   */
  struct Session
  {
    unsigned event=0,born=0,lastTime=0,committedAt=0,firstOfferSent=0;
    unsigned kickedAt=0,flightAt=0,captureAt=0,settleAt=0,handoffAt=0,releaseAt=0;
    unsigned initialLocalKick=0,initialPeerKick=0,firstKickRequestedAt=0,lastContractPeerStamp=0;
    unsigned kickCompletionAt=0,firstFuturePeerAt=0;
    unsigned shortBallAt=0,shortBallSeen=0;
    unsigned nearFootWaitAt=0,receiverCaptureStamp=0;
    int sender=-1,receiver=-1,self=-1;
    Stage stage=Stage::idle;
    bool hasContract=false,confirmed=false,sawPlaying=false,receiverKickSeen=false,releaseMotionSeen=false,senderWithdrawn=false;
    bool completionFootClosed=false;
    bool receiverCaptureSeen=false;
    Point launch{},captureBall{};
    Point shortBallAnchor{};
    Plan committed{};
    Peer latestPeer{};
    const char* reason="inactive";
    WhiteFleetC9PassTimeAudit::Snapshot timeAudit;
    WhiteFleetC9PassKickClock::Clock kickClock;
    StoppedBallEvidence stoppedBallEvidence;
    CaptureMotionEvidence captureMotion;
    WhiteFleetKickoffAimAnchor aim;
    bool gapArmed=false,gapRetired=false;
    unsigned gapSeen=0,gapLastTime=0;
    Point gapBallOdometry{},gapTargetOdometry{},gapFieldPosition{};
    float gapFieldHeading=0.f;
    CaptureOdometry gapOrigin{},gapLast{};
    unsigned captureReconfirmAt=0,captureReconfirmSeen=0,captureReconfirmGood=0;
    bool captureReconfirmed=false;
    unsigned nativePursuitAt=0,nativeNearSeen=0;
    unsigned fixedFirstFootCommandAt=0;
    unsigned prestartAt=0;
    unsigned preloadAt=0;
    unsigned receiverApproachAt=0,receiverApproachSeen=0,receiverApproachGood=0;

    void reset(){*this=Session();}
    const Plan& committedPlan()const{return committed;}
    Directive update(const Input& input);

  private:
    bool updateGapWalk(const Input& i,bool localVisual,Point& body)
    {
      // This is one fixed ODOM waypoint per genuinely new own observation.
      // Retained BallModel ticks never extend the observation's 450ms lease.
      if(localVisual&&i.localSeen>gapSeen)
      {
        gapSeen=i.localSeen;gapArmed=false;gapRetired=true;
        const float range=distance(i.ballRelative,{0.f,0.f});
        if(i.trackingReady&&finite(i.ballRelative)&&std::isfinite(range)&&range>=2000.f
           &&validCaptureOdometry(i.odometry)&&finite(i.position)&&std::isfinite(i.fieldHeading)
           &&std::isfinite(i.odometrySpeedBound)&&i.odometrySpeedBound>0.f
           &&std::isfinite(i.odometryYawBound)&&i.odometryYawBound>0.f)
        {
          const Point ball=captureRotate(i.ballRelative,i.odometry.heading);
          const float step=std::min(1200.f,range-1500.f);
          const Point way=captureRotate({i.ballRelative.x*step/range,i.ballRelative.y*step/range},i.odometry.heading);
          gapBallOdometry={i.odometry.position.x+ball.x,i.odometry.position.y+ball.y};
          gapTargetOdometry={i.odometry.position.x+way.x,i.odometry.position.y+way.y};
          gapOrigin=gapLast=i.odometry;gapFieldPosition=i.position;gapFieldHeading=i.fieldHeading;gapLastTime=i.now;
          gapArmed=finite(gapBallOdometry)&&finite(gapTargetOdometry);gapRetired=!gapArmed;
        }
      }
      if(!gapArmed||gapRetired)return false;
      const auto reject=[&](){gapRetired=true;return false;};
      if(!i.playing||!i.trackingReady||!i.ballInsidePitch||!finite(i.ball)||!finite(i.ballRelative)
         ||!finite(i.position)||!std::isfinite(i.fieldHeading)||!validCaptureOdometry(i.odometry)
         ||i.localSeen!=gapSeen||i.now<gapSeen||i.now-gapSeen>450u||i.now<gapLastTime
         ||!std::isfinite(i.odometrySpeedBound)||i.odometrySpeedBound<=0.f
         ||!std::isfinite(i.odometryYawBound)||i.odometryYawBound<=0.f)return reject();
      const float dt=static_cast<float>(i.now-gapLastTime)*.001f;
      if(distance(i.odometry.position,gapLast.position)>i.odometrySpeedBound*dt+100.f
         ||std::abs(captureAngle(i.odometry.heading-gapLast.heading))>i.odometryYawBound*dt+.1f)return reject();
      const Point delta=captureRotate({i.odometry.position.x-gapOrigin.position.x,
          i.odometry.position.y-gapOrigin.position.y},-gapOrigin.heading);
      const Point fieldDelta=captureRotate(delta,gapFieldHeading);
      const Point anchoredPosition{gapFieldPosition.x+fieldDelta.x,gapFieldPosition.y+fieldDelta.y};
      const float anchoredHeading=captureAngle(gapFieldHeading+captureAngle(i.odometry.heading-gapOrigin.heading));
      if(!finite(anchoredPosition)||!std::isfinite(anchoredHeading)||distance(anchoredPosition,i.position)>300.f
         ||std::abs(captureAngle(anchoredHeading-i.fieldHeading))>.1745329252f)return reject();
      const Point ball=captureRotate({gapBallOdometry.x-i.odometry.position.x,
          gapBallOdometry.y-i.odometry.position.y},-i.odometry.heading);
      const Point way=captureRotate({gapTargetOdometry.x-i.odometry.position.x,
          gapTargetOdometry.y-i.odometry.position.y},-i.odometry.heading);
      const float remaining=distance(way,{0.f,0.f});
      // One micron permits float SE2 roundoff at the unchanged 1200mm cap.
      if(!finite(ball)||!finite(way)||distance(ball,{0.f,0.f})<2000.f
         ||distance(i.ballRelative,{0.f,0.f})<2000.f||!std::isfinite(remaining)
         ||remaining<=1.f||remaining>1200.001f||way.x*ball.x+way.y*ball.y<=0.f)return reject();
      const Point fieldWay=captureRotate(way,i.fieldHeading);
      body={i.position.x+fieldWay.x,i.position.y+fieldWay.y};
      if(!finite(body)||body.x<i.field.ownGoal+300.f||body.x>i.field.opponentGoal-300.f
         ||std::abs(body.y)>i.field.halfWidth-300.f)return reject();
      gapLast=i.odometry;gapLastTime=i.now;
      return i.now-gapSeen>250u; // Fresh frames still take the original native Action7 below.
    }
    void end(const char* why){stage=Stage::ended;reason=why;}
    Directive output(const Input& i,Action action,bool offer=false,bool echo=false)const
    {
      Directive result{action,stage,action!=Action::none,offer,echo,sender,receiver,event,committed,reason};
      // A tombstone only filters this sender/endpoint's ordinary receive path;
      // it grants no motion, role or ball permission and never renews the event.
      const bool retiredWindow=i.now>=born&&(i.now-born<=i.params.total
          ||i.now-born-i.params.total<=retiredReceiveTail);
      result.suppressLegacyReceive=stage==Stage::ended&&hasContract&&self==receiver&&i.playing
          &&i.event==event&&retiredWindow&&!senderWithdrawn;
      // A local direction aid is not permission for any other action/thread.
      if(i.params.stableKickAim&&action==Action::kick&&aim.valid&&!aim.retired&&aim.lastTime==i.now)
      {result.aimValid=true;result.aimTargetOdometry=aim.targetOdometry;result.aimFrame=i.now;}
      return result;
    }
    Directive terminal(const Input& i)const
    {
      // A cancelled own kickoff must not fall through to the old opposite-side
      // space outlet. The native referee bounds this hold, not an endless lease.
      const bool transactionWait=i.params.playbook&&i.playing&&i.event==event
          &&i.now>=born&&i.now-born<=i.params.total&&!kickedAt&&!captureAt;
      return output(i,(i.ownKickoff&&i.event==event)||transactionWait?Action::hold:Action::none);
    }
    bool captureGeometry(Point position,Point ball)const
    {
      if(!hasContract||!finite(position)||!finite(ball)||distance(position,ball)>captureDistance
         ||distance(ball,committed.target)>captureTargetRadius)return false;
      const float dx=committed.target.x-launch.x,dy=committed.target.y-launch.y,length2=dx*dx+dy*dy;
      return length2>1.f&&((ball.x-launch.x)*dx+(ball.y-launch.y)*dy)/length2>=.5f;
    }
  };

  inline Directive Session::update(const Input& i)
  {
    timeAudit={};
    if(!i.enabled){reset();return {};}
    const bool senderSelf=i.self==i.params.kicker;
    const bool receiverSelf=i.self==i.receiver;
    if(!paramsValid(i.params)||!fieldValid(i.field)||!i.rosterValid||i.params.kicker==i.receiver
       ||i.receiver<=0||(!senderSelf&&!receiverSelf))
    {
      if(event){end("invalid_pair_or_roster");return terminal(i);}
      return {};
    }
    if(event!=i.event)
    {
      if(!i.ownKickoff){if(event)end("new_event");return terminal(i);}
      reset();event=i.event;born=i.event;lastTime=i.now;
      sender=i.params.kicker;receiver=i.receiver;self=i.self;
      initialLocalKick=i.localKick;
      if(i.c9Coherence){kickClock.seed(i.now,i.localKick);initialLocalKick=kickClock.baseline;}
      initialPeerKick=i.peer.present&&i.peer.stamp<=i.now&&i.peer.kickProxy<=i.peer.stamp?i.peer.kickProxy:0;
      stage=Stage::waiting;reason="waiting_contract";
    }
    else if(stage==Stage::idle&&i.ownKickoff)
    {
      born=event=i.event;lastTime=i.now;sender=i.params.kicker;receiver=i.receiver;self=i.self;
      initialLocalKick=i.localKick;
      if(i.c9Coherence){kickClock.seed(i.now,i.localKick);initialLocalKick=kickClock.baseline;}
      initialPeerKick=i.peer.present&&i.peer.stamp<=i.now&&i.peer.kickProxy<=i.peer.stamp?i.peer.kickProxy:0;
      stage=Stage::waiting;reason="waiting_contract";
    }
    if(stage==Stage::idle)return {};
    const unsigned timeReject=WhiteFleetC9PassTimeAudit::inspect(i.now,lastTime,event,i.localSeen,i.localKick,i.sent.present,i.sent.stamp);
    unsigned appliedTimeReject=timeReject,deferredTime=0,localKick=i.localKick;
    WhiteFleetC9PassKickClock::Result kickTime;
    if(i.c9Coherence&&(timeReject&~WhiteFleetC9PassTimeAudit::localKickFuture)==0u)
    {
      kickTime=kickClock.observe(i.now,i.localKick,stage==Stage::ended);
      appliedTimeReject=kickTime.reject?WhiteFleetC9PassTimeAudit::localKickFuture:0u;
      if(kickClock.pendingAt&&stage!=Stage::ended&&!kickTime.reject)
        deferredTime=WhiteFleetC9PassTimeAudit::localKickFuture;
      else if(stage==Stage::ended&&(timeReject&WhiteFleetC9PassTimeAudit::localKickFuture))
        deferredTime=WhiteFleetC9PassTimeAudit::localKickFuture;
      localKick=kickClock.value;
      if(kickClock.baselineReady)initialLocalKick=kickClock.baseline;
    }
    if(i.timeDiagnostic)
    {
      timeAudit={true,i.sent.present,i.now,lastTime,event,i.localSeen,i.localKick,i.sent.stamp,timeReject,static_cast<unsigned>(stage)};
      timeAudit.appliedMask=appliedTimeReject;timeAudit.deferredMask=deferredTime;timeAudit.usableKick=appliedTimeReject?0u:localKick;
      timeAudit.pendingKick=kickClock.pendingValue;timeAudit.kickWaitAt=kickClock.pendingAt;timeAudit.kickClockState=kickTime.state;
    }
    if(appliedTimeReject)
    {
      if(!i.c9Coherence||stage!=Stage::ended)
        end(kickTime.reject?kickTime.reason:"invalid_or_regressed_time");
      return terminal(i);
    }
    lastTime=i.now;
    // A small remote clock lead is not usable evidence. Keep the last valid
    // packet without clamping its replacement or renewing any deadline. Only
    // a genuinely newer valid packet below can finish this one fixed wait.
    const bool futurePeer=i.peer.present&&i.peer.stamp>i.now;
    if(stage!=Stage::ended&&futurePeer)
    {
      if(i.peer.stamp-i.now>peerFutureWait)
      {end("peer_time_gross_future");return terminal(i);}
      if(!firstFuturePeerAt)firstFuturePeerAt=i.now;
    }
    if(stage!=Stage::ended&&firstFuturePeerAt&&i.now-firstFuturePeerAt>peerFutureWait)
    {end("peer_time_future_wait_expired");return terminal(i);}
    if(self!=i.self||sender!=i.params.kicker||receiver!=i.receiver)
    {end("pair_changed");return terminal(i);}
    if(i.stopped||!i.active||!i.upright||!i.poseValid||i.intercept||(!i.ownKickoff&&!i.playing))
    {end("native_protection_or_restart");return output(i,Action::none);}
    if(stage==Stage::ended)
    {
      if(self==receiver&&hasContract&&i.peer.present&&i.peer.number==sender
         &&i.peer.stamp>lastContractPeerStamp&&i.peer.stamp<=i.now
         &&i.peer.healthy&&finite(i.peer.position)&&finite(i.peer.walkingTo)
         &&(!i.peer.hasTarget||finite(i.peer.target)))
      {
        firstFuturePeerAt=0;
        if(i.peer.passTarget==receiver&&(!i.peer.hasTarget||samePoint(i.peer.target,committed.target)))
          lastContractPeerStamp=i.peer.stamp;
        else senderWithdrawn=true;
      }
      return terminal(i);
    }
    if(i.now-born>i.params.total){end("total_deadline");return terminal(i);}
    if(sawPlaying&&i.ownKickoff){end("restart_after_release");return terminal(i);}
    sawPlaying|=i.playing;

    const int other=senderSelf?receiver:sender;
    bool newPeer=false;
    if(i.peer.present&&!futurePeer&&i.peer.number==other&&i.peer.stamp>=event
       &&i.peer.stamp>latestPeer.stamp)
    {
      latestPeer=i.peer;newPeer=true;
      if(receiverSelf&&hasContract)
      {
        if(latestPeer.passTarget==receiver&&(!latestPeer.hasTarget||samePoint(latestPeer.target,committed.target)))
          lastContractPeerStamp=latestPeer.stamp;
        else if(latestPeer.stamp>lastContractPeerStamp)senderWithdrawn=true;
      }
      if(!latestPeer.healthy||!finite(latestPeer.position)||!finite(latestPeer.walkingTo)
         ||latestPeer.ballSeen>latestPeer.stamp||latestPeer.kickProxy>latestPeer.stamp
         ||(latestPeer.hasTarget&&!finite(latestPeer.target)))
      {end("peer_withdrawn_health");return terminal(i);}
      firstFuturePeerAt=0;
    }
    const unsigned liveLimit=std::min(i.peerTimeout,10000u);
    const bool live=latestPeer.present&&latestPeer.healthy&&sessionFresh(i.now,latestPeer.stamp,liveLimit);
    if(hasContract&&latestPeer.present&&!live){end("peer_liveness_expired");return terminal(i);}
    const bool ownFresh=sessionFresh(i.now,i.localSeen,1000)&&i.ballInsidePitch&&finite(i.ball);
    const bool localVisual=sessionFresh(i.now,i.localSeen,250)&&i.ballInsidePitch&&finite(i.ball);
    const bool captureFresh=localVisual&&finite(i.ballRelative)
        &&distance(i.ballRelative,{0.f,0.f})<=captureDistance;

    // Ball Free is permission, not a different first-pass selector. Keep this
    // event's one transaction within its ORIGINAL budgets; no new lease.

    if(senderSelf)
    {
      if(!hasContract&&(i.ownKickoff||(i.params.playbook&&i.playing&&i.now-born<=i.params.maxPrepare))
         &&ownFresh&&validPlan(i.plan)&&i.planSafe
         &&(!i.params.playbook||i.nativeOwner)
         &&(!i.params.playbook||(WhiteFleetC9ForwardFlank::directed(i.ball,i.plan.target,i.plan.side,i.field,i.params)
              &&WhiteFleetC9ForwardFlank::receiverSide(i.ball,latestPeer.position,i.plan.side)))
         &&(!i.params.side||i.params.side==i.plan.side))
      {
        committed=i.plan;launch=i.ball;committedAt=i.now;hasContract=true;
        stage=Stage::preparing;reason="offer_committed";
      }
      // Refresh only locally recomputed flight/length metadata for an immutable
      // endpoint. Neither a moving peer nor a later candidate can drift P.
      if(hasContract&&!kickCompletionAt&&validPlan(i.plan)&&samePoint(i.plan.target,committed.target)
         &&samePoint(i.plan.prepare,committed.prepare))
      {
        const Point target=committed.target,prepare=committed.prepare;
        committed=i.plan;committed.target=target;committed.prepare=prepare;
      }
      if(hasContract&&!firstOfferSent&&i.sent.present&&i.sent.stamp>=committedAt
         &&i.sent.passTarget==receiver&&i.sent.hasTarget&&samePoint(i.sent.target,committed.target))
        firstOfferSent=i.sent.stamp; // serialized locally, NOT delivery proof
      if(hasContract&&live&&firstOfferSent&&latestPeer.stamp>firstOfferSent
         &&latestPeer.passTarget==-1&&latestPeer.hasTarget&&samePoint(latestPeer.target,committed.target)
         &&samePoint(latestPeer.walkingTo,committed.prepare)&&latestPeer.position.x<=0.f
         &&distance(latestPeer.position,committed.prepare)<=600.f&&i.receiverReachable)
      {confirmed=true;if(stage==Stage::preparing){stage=Stage::armed;reason="prepared_point_response";}}
      const bool ownKick=localKick>initialLocalKick&&localKick>=event;
      if(ownKick&&!kickedAt)
      {
        if(i.c9Coherence&&(!kickCompletionAt||localKick<kickCompletionAt))
        {end("unrequested_local_kick_proxy");return terminal(i);}
        if(!confirmed){end("kick_proxy_before_confirmation");return terminal(i);}
        kickedAt=localKick;flightAt=i.now;stage=Stage::flight;reason="own_kick_proxy_support";
      }
      if(!kickedAt)
      {
        if(newPeer&&confirmed&&(!latestPeer.hasTarget||latestPeer.passTarget!=-1
             ||!samePoint(latestPeer.target,committed.target)))
        {end("prepared_response_withdrawn");return terminal(i);}
        if(i.playing&&!releaseAt)releaseAt=i.now;
        if(hasContract&&confirmed)
        {
          const float dx=committed.target.x-launch.x,dy=committed.target.y-launch.y;
          const float length=std::hypot(dx,dy);
          if(localVisual&&i.localSeen>committedAt&&length>1.f)
          {
            const float progress=((i.ball.x-launch.x)*dx+(i.ball.y-launch.y)*dy)/length;
            const float crossTrack=std::abs((i.ball.x-launch.x)*dy-(i.ball.y-launch.y)*dx)/length;
            releaseMotionSeen|=progress>300.f&&crossTrack<=900.f;
          }
        }
        // This is continuity of one fully admitted, near-foot request, NOT
        // proof of contact or that the learned policy started. Its immutable
        // completion window spans ownKickoff -> playing; soft remaining-route
        // ETA/peer destination changes must not abort that same original kick.
        if(kickCompletionAt)
        {
          if(i.now-kickCompletionAt>kickCompletionWindow)
          {end("kick_completion_proxy_timeout");return terminal(i);}
          if(!localVisual||!finite(i.ballRelative)||distance(i.ballRelative,{0.f,0.f})>1300.f)
            completionFootClosed=true;
          if(i.params.stableKickAim&&!aim.observe(i.now,committed.target,i.position,i.fieldHeading,
              i.odometry,i.odometrySpeedBound,i.odometryYawBound))
          {
            completionFootClosed=true;reason="stable_aim_closed_await_proxy";
            return output(i,Action::hold,true); // No reanchor and no renewed foot/epoch budget.
          }
          if(i.params.raceRendezvous&&!releaseMotionSeen&&i.plan.hardRaceConflict)
          {
            completionFootClosed=true;reason="race_close_unreleased_unsafe_foot";
            return output(i,Action::hold,true); // Original completion/epoch deadlines are not renewed.
          }
          if(!completionFootClosed)
          {reason="continue_authorized_foot_request_await_proxy";return output(i,Action::kick,true);}
          reason="foot_request_closed_await_proxy";
          return output(i,Action::hold,true);
        }
        // C9 may finish the SAME admitted transaction after a native early
        // release. Ball Free alone still supplies neither contract nor safety.
        if(i.playing&&!i.params.playbook)
        {
          if(!confirmed){end("release_without_confirmed_contract");return output(i,Action::none);}
          if(i.now-releaseAt>kickCompletionWindow)
          {end("release_proxy_timeout");return output(i,Action::none);}
          reason="released_without_foot_request_await_proxy";
          return output(i,Action::hold,true);
        }
        if(i.now-born>i.params.maxPrepare){end("prepare_deadline_no_kick");return terminal(i);}
        if(!hasContract){reason="waiting_feasible_plan";return output(i,Action::hold);}
        if(i.c9Coherence&&(!kickClock.baselineReady||deferredTime))
        {reason="await_local_kick_time_before_first_request";return output(i,Action::hold,true);}
        if(confirmed&&localVisual&&i.planSafe&&i.receiverReachable
           &&distance(latestPeer.position,committed.prepare)<=600.f&&i.now-committedAt>=i.params.minPrepare)
        {
          if(i.params.stableKickAim)
          {
            // Entry only: wait for a genuinely near, fresh ball and a qualified
            // FIELD/ODOM pair before starting the original first-foot clock.
            if(!captureFresh||!std::isfinite(i.odometrySpeedBound)||i.odometrySpeedBound<=0.f
               ||!std::isfinite(i.odometryYawBound)||i.odometryYawBound<=0.f)
            {reason="stable_aim_wait_fresh_near_foot";return output(i,Action::hold,true);}
            if((!aim.anchored&&!aim.begin(i.now,committed.target,i.position,i.fieldHeading,
                   i.fieldHeadingVariance,i.odometry))
               ||!aim.observe(i.now,committed.target,i.position,i.fieldHeading,
                   i.odometry,i.odometrySpeedBound,i.odometryYawBound))
            {reason="stable_aim_wait_qualified_entry";return output(i,Action::hold,true);}
          }
          if(!firstKickRequestedAt)firstKickRequestedAt=i.now;
          if(finite(i.ballRelative)&&distance(i.ballRelative,{0.f,0.f})<=1300.f)
            kickCompletionAt=i.now;
          reason="confirmed_safe_kick";return output(i,Action::kick,true);
        }
        reason=confirmed?"hold_current_ball_or_safety":"waiting_point_response";
        return output(i,Action::hold,true);
      }

      // A shot proxy is not physical reception/contact. Keep the sender out of
      // the channel until a bounded handoff is observable or its fixed limit.
      const bool peerBall=live&&latestPeer.ballSeen<=latestPeer.stamp
          &&sessionFresh(latestPeer.stamp,latestPeer.ballSeen,250)&&finite(latestPeer.ball);
      if(i.c9Coherence)
      {
        if(peerBall&&captureGeometry(latestPeer.position,latestPeer.ball)&&latestPeer.nativeOwner)
        {receiverCaptureSeen=true;if(!receiverCaptureStamp)receiverCaptureStamp=latestPeer.stamp;}
        const bool echo=live&&latestPeer.passTarget==-1&&latestPeer.hasTarget&&samePoint(latestPeer.target,committed.target);
        const bool proxy=live&&latestPeer.kickProxy>initialPeerKick&&latestPeer.kickProxy>kickedAt
            &&latestPeer.kickProxy<=latestPeer.stamp;
        // Neither a proxy nor native role alone withdraws an ongoing contract.
        if(proxy&&receiverCaptureSeen)receiverKickSeen=true;
        if(newPeer&&!echo)
        {
          if(!receiverCaptureSeen){end(i.params.nativeReceive?"receiver_native_pursuit_released":"receiver_withdrew_without_capture");return output(i,Action::none);}
          if(!handoffAt)handoffAt=i.now;
        }
        if(handoffAt)
        {
          if(i.now-handoffAt>=500){end("receiver_causal_handoff_observed");return output(i,Action::none);}
          reason="receiver_causal_handoff_tail";return output(i,Action::support);
        }
        if(i.now-flightAt>i.params.flight&&i.now-flightAt-i.params.flight>i.params.finish)
        {end("support_deadline");return output(i,Action::none);}
        reason=proxy&&!receiverCaptureSeen?"unqualified_peer_proxy_keep_contract":"sender_keep_receiver_contract";
        return output(i,Action::support,true);
      }
      if(peerBall&&captureGeometry(latestPeer.position,latestPeer.ball)&&latestPeer.nativeOwner&&!handoffAt)
        handoffAt=i.now;
      if(live&&latestPeer.kickProxy>initialPeerKick&&latestPeer.kickProxy>kickedAt
         &&latestPeer.kickProxy<=latestPeer.stamp)
      {receiverKickSeen=true;if(!handoffAt)handoffAt=i.now;}
      const bool movedAfterShot=receiverKickSeen&&ownFresh&&distance(i.ball,committed.target)>captureTargetRadius;
      if(handoffAt&&i.now-handoffAt>=500&&(!i.nativeOwner||movedAfterShot))
      {end("receiver_handoff_observed");return output(i,Action::none);}
      if(i.now-flightAt>i.params.flight&&i.now-flightAt-i.params.flight>i.params.finish)
      {end("support_deadline");return output(i,Action::none);}
      reason=receiverKickSeen?"receiver_kick_proxy_support_tail":"sender_yield_support";
      return output(i,Action::support,!handoffAt&&!receiverKickSeen);
    }

    const bool offer=live&&latestPeer.passTarget==receiver&&latestPeer.hasTarget&&finite(latestPeer.target);
    if(!hasContract&&(i.ownKickoff||(i.params.playbook&&i.playing&&i.now-born<=i.params.maxPrepare))
       &&offer&&(!i.params.playbook||latestPeer.nativeOwner))
    {
      const int side=i.params.directCenter?i.params.side:latestPeer.target.y<0.f?-1:1;
      const bool observedAtSend=latestPeer.ballSeen<=latestPeer.stamp
          &&sessionFresh(latestPeer.stamp,latestPeer.ballSeen,1000);
      bool region=false;
      for(Point candidate:WhiteFleetC9ForwardFlank::candidates(i.field,i.params,side))region|=samePoint(candidate,latestPeer.target);
      region|=WhiteFleetC15DefenceAdaptive::domain(latestPeer.ball,latestPeer.target,i.field,i.params,side,contractTolerance);
      region|=WhiteFleetC15RollingRendezvous::domain(latestPeer.ball,latestPeer.target,i.field,i.params,contractTolerance);
      if(observedAtSend&&region&&(!i.params.side||i.params.side==side)
         &&targetSafe(latestPeer.ball,latestPeer.target,i.field,i.params,side,i.params.centerFast?contractTolerance:0.f)
         &&(!i.params.playbook||(WhiteFleetC9ForwardFlank::directed(latestPeer.ball,latestPeer.target,side,i.field,i.params,i.params.centerFast?contractTolerance:0.f)
              &&WhiteFleetC9ForwardFlank::receiverSide(latestPeer.ball,i.position,side))))
      {
        committed=i.plan;
        if(!validPlan(committed)||!samePoint(committed.target,latestPeer.target))
        {committed=Plan();committed.reason="wire_geometry_only";}
        committed.target=latestPeer.target;committed.prepare=preparePoint(i.field,i.params,side);committed.side=side;
        launch=latestPeer.ball;hasContract=true;committedAt=i.now;
        lastContractPeerStamp=latestPeer.stamp;
        stage=Stage::preparing;reason="received_future_point_contract";
      }
    }
    if(!hasContract)
    {
      if(i.now-born>i.params.maxPrepare){end("prepare_deadline_no_offer");return terminal(i);}
      if((i.params.raceRendezvous||i.params.directCenter)&&i.ownKickoff)
      {
        const auto points=i.params.directCenter?std::vector<Point>{directCenterTarget(i.field,i.params)}:
            WhiteFleetC15RollingRendezvous::targets(i.field,i.params);
        if(!points.empty())
        {
          reason="race_prepare_without_offer_no_ack";
          auto d=output(i,Action::prepare);d.hasBodyTarget=true;d.bodyTarget=preparePoint(i.field,i.params,-1);
          d.plan.prepare=d.bodyTarget;d.plan.target=points.front();d.plan.side=-1;return d;
        }
      }
      reason="waiting_sender_offer";return output(i,Action::hold);
    }
    if(newPeer&&stage!=Stage::capture&&stage!=Stage::settling
       &&(!offer||!samePoint(latestPeer.target,committed.target)))
    {end("sender_offer_withdrawn");return terminal(i);}
    if(i.ownKickoff)
    {
      if(i.now-born>i.params.maxPrepare){end("prepare_deadline_no_release");return terminal(i);}
      if(i.params.raceRendezvous||i.params.centerFast)
      {
        // Keep the original native standing ball-motion detector usable.
        // Do not walk/preload while it is collecting genuine observations;
        // no GameState/IllegalAreas/BallFree bit is fabricated or cleared.
        reason="stand_observe_native_release";return output(i,Action::prepare,false,true);
      }
      if(i.params.c14EarlyStart&&(i.params.nativeReceive||i.params.directCenter)&&i.params.isolated&&i.params.playbook)
      {
        if(!prestartAt&&localVisual&&i.trackingReady&&finite(i.ballRelative)
           &&i.localSeen>committedAt&&distance(i.ball,launch)>300.f)prestartAt=i.now;
        if(prestartAt)
        {
          if(!localVisual||!i.trackingReady||!finite(i.ballRelative))
          {reason="prestart_wait_fresh_ball";return output(i,Action::hold,false,true);}
          reason="legal_own_half_prestart";
          auto d=output(i,Action::prestart,false,true);d.hasBodyTarget=true;
          d.bodyTarget=WhiteFleetC9ForwardFlank::prestartPoint(committed.prepare,committed.target);
          if(i.params.directCenter)
          {
            // Stay beyond native large-distance hysteresis while ownKickOff.
            d.bodyTarget.x=-700.f;
            d.bodyTarget.y=committed.prepare.y+(d.bodyTarget.x-committed.prepare.x)*
                (committed.target.y-committed.prepare.y)/(committed.target.x-committed.prepare.x);
          }
          return d;
        }
      }
      // Pre-foot motion is only a legal own-half preparation, never touch
      // evidence. A fresh, aligned near-ball passer still may cancel the kick.
      const float ray=std::atan2(committed.target.y-launch.y,committed.target.x-launch.x);
      const bool aligned=std::isfinite(latestPeer.heading)&&
          std::abs(captureAngle(ray-latestPeer.heading))<=.61086524f;
      if(!preloadAt&&i.params.legalPreload&&i.params.c14EarlyStart&&i.params.nativeReceive&&i.params.isolated&&
         localVisual&&i.trackingReady&&live&&sessionFresh(i.now,latestPeer.stamp,600)&&
         sessionFresh(latestPeer.stamp,latestPeer.ballSeen,250)&&latestPeer.nativeOwner&&
         finite(latestPeer.ball)&&distance(latestPeer.position,latestPeer.ball)<=650.f&&
         samePoint(latestPeer.ball,launch)&&aligned&&distance(i.position,committed.prepare)<=600.f)
      {
        const float x=std::min(-750.f,committed.prepare.x+200.f);
        const float dx=committed.target.x-committed.prepare.x;
        if(dx>0.f&&x>committed.prepare.x)
        {
          const Point body{x,committed.prepare.y+(x-committed.prepare.x)*(committed.target.y-committed.prepare.y)/dx};
          if(distance(body,committed.prepare)<=contractTolerance&&i.position.x<=body.x+50.f)
          {
            preloadAt=i.now;
            reason="legal_own_half_preload_not_ball_motion";
            auto d=output(i,Action::preload,false,true);d.hasBodyTarget=true;d.bodyTarget=body;return d;
          }
        }
      }
      if(preloadAt)
      {
        if(!localVisual||!i.trackingReady){reason="preload_wait_fresh_ball";return output(i,Action::hold,false,true);}
        const float x=std::min(-750.f,committed.prepare.x+200.f);
        const float dx=committed.target.x-committed.prepare.x;
        reason="continue_own_half_preload_not_ball_motion";
        auto d=output(i,Action::preload,false,true);d.hasBodyTarget=true;
        d.bodyTarget={x,committed.prepare.y+(x-committed.prepare.x)*(committed.target.y-committed.prepare.y)/dx};return d;
      }
      reason="legal_own_half_preparation";return output(i,Action::prepare,false,true);
    }
    if(!flightAt){flightAt=i.now;stage=Stage::flight;reason="native_playing_release";}
    if(i.now-flightAt>i.params.flight&&stage!=Stage::capture&&stage!=Stage::settling)
    {end("flight_deadline");return output(i,Action::none);}
    if((i.params.raceRendezvous||i.params.directCenter)&&i.params.isolated&&i.params.playbook)
    {
      // The rolling planner qualifies a route to the locked meeting point,
      // not an inward chase of the current ball with a shooting executor.
      // Native playing alone is not a touch: retain the genuine fresh-motion
      // witness before starting that route. Capture/odometry/finish below are
      // the ORIGINAL gates, with their original clocks and thresholds.
      if(!nativePursuitAt&&localVisual&&i.trackingReady&&finite(i.ballRelative)
         &&i.localSeen>committedAt&&distance(i.ball,launch)>300.f)nativePursuitAt=i.now;
      if(!nativePursuitAt)
      {reason="race_meeting_wait_observed_motion";return output(i,Action::hold,false,true);}
    }
    if((i.params.nativeReceive||i.params.receiverEarlyNative)&&i.params.isolated&&i.params.playbook&&!i.params.raceRendezvous)
    {
      // P remains the immutable first-pass contract, not a leash on the real
      // ball. Observe actual ball motion after legal release, not a predicted
      // pass. A deflection must not make a fresh real ball unchaseable.
      // The explicit early-native receiver profile uses this same bounded
      // pursuit before any P-region capture; it does not claim captureAt or
      // bypass native KickAtGoal, fresh-ball, referee or collision permissions.
      if(!nativePursuitAt&&localVisual&&i.trackingReady&&finite(i.ballRelative)
         &&i.localSeen>committedAt&&distance(i.ball,launch)>300.f)
        nativePursuitAt=i.now;
      if(nativePursuitAt)
      {
        Point gapBody{};
        const bool predictiveOnly=i.params.firstPassPrepareAdvance==250.f;
        const bool gapReady=!predictiveOnly&&i.params.receiverGapWalk&&updateGapWalk(i,localVisual,gapBody);
        // Close observation + a new native kick proxy are distinct from
        // physical reception. Do not set captureAt or renew the old deadline.
        // Walking contact can update MotionInfo.lastKickTimestamp. In the
        // explicit fixed RUN it cannot retire ownership before an actual
        // previous final native-foot output (only issued after ARRIVED).
        if(i.fixedRunToArrival&&i.params.firstPassPrepareAdvance==250.f&&i.fixedFootCommandAt>i.now)
        {end("fixed_foot_command_future");return output(i,Action::none);}
        if(i.fixedRunToArrival&&i.params.firstPassPrepareAdvance==250.f&&!fixedFirstFootCommandAt
           &&i.fixedFootCommandAt>=nativePursuitAt&&i.fixedFootCommandAt<=i.now)
          fixedFirstFootCommandAt=i.fixedFootCommandAt;
        const bool footQualified=!i.fixedRunToArrival||i.params.firstPassPrepareAdvance!=250.f
            ||(fixedFirstFootCommandAt&&localKick>=fixedFirstFootCommandAt);
        if(footQualified&&localKick>initialLocalKick&&localKick>=nativePursuitAt&&nativeNearSeen
           &&localKick>=nativeNearSeen&&localKick-nativeNearSeen<=1000u)
        {settleAt=i.now;end("native_receive_kick_proxy");return output(i,Action::none);}
        if(captureFresh&&(!i.fixedRunToArrival||i.params.firstPassPrepareAdvance!=250.f||fixedFirstFootCommandAt))nativeNearSeen=i.localSeen;
        if(predictiveOnly)
        {
          // Exclusive receiver executor: neither the old gap waypoint nor
          // ordinary native pursuit may take over when a prediction is missing.
          // SBC independently limits stale walking and all fresh-foot output.
          // Loss of tracking waits here; it does not retire into old play at650.
          reason=i.fixedRunToArrival?"fixed_run_session_current":i.trackingReady?"predictive_receiver_exclusive":"predictive_receiver_reacquire_wait";
          return output(i,Action::pursue,false,true);
        }
        if(!i.trackingReady){end("native_receive_lost_local_ball");return output(i,Action::none);}
        if(localVisual&&finite(i.ballRelative))
        {reason="native_real_ball_pursuit_not_capture";return output(i,Action::pursue,false,true);}
        if(gapReady)
        {
          reason="receiver_short_gap_odometry_walk_not_kick";
          auto d=output(i,Action::gapWalk,false,true);d.hasBodyTarget=true;d.bodyTarget=gapBody;
          d.gapValid=true;d.gapSeen=gapSeen;d.gapFrame=i.now;
          d.gapBallOdometry=gapBallOdometry;d.gapTargetOdometry=gapTargetOdometry;return d;
        }
        reason="native_receive_wait_fresh_ball";return output(i,Action::hold,false,true);
      }
      reason="await_observed_ball_motion";return output(i,Action::hold,false,true);
    }
    if(i.c9Coherence&&(stage==Stage::flight||stage==Stage::capture))
    {
      const bool nearStale=i.localSeen>0&&i.localSeen<=i.now&&!localVisual&&finite(i.ballRelative)
          &&distance(i.ballRelative,{0.f,0.f})<=captureDistance;
      if(localVisual)nearFootWaitAt=0;
      else if(nearStale||nearFootWaitAt)
      {
        if(!nearFootWaitAt)nearFootWaitAt=i.now;
        if(i.now-nearFootWaitAt>250){end("near_foot_fresh_wait_expired");return output(i,Action::none);}
        reason="near_foot_wait_fresh";return output(i,Action::hold,false,true);
      }
    }
    bool approachCaptureReady=true;
    if(i.params.receiverBallApproach&&stage==Stage::flight)
    {
      const auto approach=receiverBallApproach(launch,committed.target,i.ball,i.position,i.field,i.params);
      if(!receiverApproachAt&&(i.params.receiverFollowFromRelease||(localVisual&&i.trackingReady&&approach.valid)))
        receiverApproachAt=i.now;
      // An early movement-only approach is not a reception. Require two real,
      // distinct close observations with the ball IN FRONT before entering the
      // original odometry/capture/reconfirmation path. Never renew a clock.
      const bool candidate=localVisual&&i.trackingReady&&captureFresh
          &&captureGeometry(i.position,i.ball)&&receiverBallInFront(i.ballRelative);
      if(!candidate){receiverApproachGood=0;receiverApproachSeen=0;}
      else if(i.localSeen>receiverApproachSeen)
      {receiverApproachSeen=i.localSeen;receiverApproachGood=std::min(2u,receiverApproachGood+1);}
      approachCaptureReady=receiverApproachGood>=2;
      if(receiverApproachAt&&(!localVisual||!i.trackingReady||!approach.valid)
         &&(!i.params.receiverFollowFromRelease||!approachCaptureReady))
      {reason="receiver_approach_wait_current_ball";return output(i,Action::hold,false,true);}
    }
    if(stage==Stage::flight&&captureFresh&&captureGeometry(i.position,i.ball)&&approachCaptureReady)
    {
      if(i.params.playbook&&!captureMotion.begin(i.now,i.odometry,i.ballRelative))
      {reason="capture_wait_native_odometry";return output(i,Action::hold,false,true);}
      stage=Stage::capture;captureAt=i.now;captureBall=i.ball;reason="local_capture_candidate";
    }
    if(i.params.adaptiveMeet&&stage==Stage::flight)
    {
      if(i.params.playbook)
      {
        if(localVisual&&i.ballStoppedShort)
        {
          if(stoppedBallEvidence.observe(i.now,i.localSeen,i.ball))
          {end("own_ball_stopped_before_region");return output(i,Action::none);}
        }
        else stoppedBallEvidence.reset();
      }
      else if(localVisual&&i.ballStoppedShort)
      {
        if(!shortBallAt||distance(i.ball,shortBallAnchor)>100.f)
        {shortBallAt=i.now;shortBallSeen=i.localSeen;shortBallAnchor=i.ball;}
        else if(i.localSeen>shortBallSeen&&i.now-shortBallAt>=300)
        {end("own_ball_stopped_before_region");return output(i,Action::none);}
      }
      else {shortBallAt=0;shortBallSeen=0;}
      if(!i.params.raceRendezvous&&!i.params.directCenter&&localVisual&&i.trackingReady&&i.meetingValid&&finite(i.meetingTarget)
         &&distance(i.meetingTarget,committed.target)<=1200.f)
      {
        reason="track_own_roll_region_meeting";
        auto d=output(i,Action::run,false,true);d.hasBodyTarget=true;d.bodyTarget=i.meetingTarget;return d;
      }
    }
    if(stage==Stage::capture)
    {
      if(localKick>initialLocalKick&&localKick>=captureAt)
      {stage=Stage::settling;settleAt=i.now;reason="receiver_next_kick_proxy";}
      else if(i.now-captureAt>i.params.finish)
      {end("capture_finish_deadline");return output(i,Action::none);}
      else
      {
        if(i.params.playbook)
        {
          const bool known=localVisual&&captureMotion.observe(i.now,i.odometry,i.ballRelative,
              i.odometrySpeedBound,i.odometryYawBound);
          if(known&&captureMotion.motionDistance>800.f)
          {end("post_capture_ball_moved");return output(i,Action::none);}
          // Current native field geometry remains mandatory: odometry cannot
          // rebase P or turn a 1.94m field estimate into a <=1.5m capture.
          const bool geometry=captureFresh&&captureGeometry(i.position,i.ball);
          if(!known||!geometry)
          {
            if(!captureReconfirmAt)captureReconfirmAt=i.now;
            captureReconfirmSeen=i.localSeen;captureReconfirmGood=0;captureReconfirmed=false;
            if(i.now-captureReconfirmAt>250)
            {end("capture_frame_not_reconfirmed");return output(i,Action::none);}
            reason=known?"capture_wait_current_field_geometry":"capture_wait_motion_evidence";
            return output(i,Action::hold,false,true);
          }
          if(captureReconfirmAt&&!captureReconfirmed)
          {
            if(i.now-captureReconfirmAt>250)
            {end("capture_frame_not_reconfirmed");return output(i,Action::none);}
            if(i.localSeen>captureReconfirmSeen)
            {captureReconfirmSeen=i.localSeen;++captureReconfirmGood;}
            if(captureReconfirmGood<2)
            {reason="capture_reconfirm_distinct_observations";return output(i,Action::hold,false,true);}
            captureReconfirmed=true; // Original first pause, captureAt/P/lease are NEVER reset.
          }
        }
        if(!i.params.isolated&&captureFresh&&i.nativeOwner&&i.now-captureAt>=250)
        {end("receiver_native_role_handoff");return output(i,Action::none);}
        else if(!i.params.playbook&&captureFresh&&distance(i.ball,captureBall)>800.f)
        {end("post_capture_ball_moved");return output(i,Action::none);}
        else if(captureFresh&&captureGeometry(i.position,i.ball))
        {reason="fresh_capture_native_shoot";return output(i,Action::finish,false,true);}
      }
    }
    if(stage==Stage::settling)
    {
      if(i.now-settleAt>=500){end("receiver_proxy_settled");return output(i,Action::none);}
      return output(i,Action::hold);
    }
    reason=i.trackingReady?(i.params.raceRendezvous?"race_run_locked_meeting_before_finish":
        "running_committed_future_point"):"bounded_hold_track_ball";
    if(i.params.receiverBallApproach&&receiverApproachAt&&stage==Stage::flight&&i.trackingReady&&localVisual)
    {
      const auto approach=receiverBallApproach(launch,committed.target,i.ball,i.position,i.field,i.params);
      if(approach.valid)
      {
        reason="receiver_track_real_ball_not_capture";
        auto d=output(i,Action::run,false,true);d.hasBodyTarget=true;d.bodyTarget=approach.body;return d;
      }
      if(i.params.receiverFollowFromRelease)
      {reason="receiver_follow_wait_legal_current_ball";return output(i,Action::hold,false,true);}
    }
    if(i.params.centerFast&&i.trackingReady&&localVisual)
    {
      auto d=output(i,Action::run,false,true);d.hasBodyTarget=true;
      d.bodyTarget=centerMeetingBody(launch,committed.target,i.ball,i.params);return d;
    }
    return output(i,i.trackingReady?Action::run:Action::hold,false,true);
  }
}
