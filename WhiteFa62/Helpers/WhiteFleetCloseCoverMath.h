#pragma once
#include "WhiteFleetDefenseCounterMath.h"
#include "Tools/BehaviorControl/TeamActivityPolicy.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

/** Positioning only. No ownership, referee authority or synthetic ball evidence. */
namespace WhiteFleetCloseCoverMath
{
  using WhiteFleetDefenseCounterMath::Point;
  using WhiteFleetDefenseCounterMath::finite;
  using WhiteFleetDefenseCounterMath::distance;
  struct Field {float ownGoal, opponentGoal, ownPenalty, halfWidth, goalHalfWidth;};
  inline bool fresh(unsigned now,unsigned stamp,unsigned limit)
  {return stamp>0 && now>=stamp && now-stamp<=limit;}
  // Event-driven messages have no 1.2 s heartbeat. This is native liveness,
  // NOT freshness of the ball observation or proof of arrival.
  inline bool peerAvailable(unsigned now,unsigned stamp,unsigned configuredTimeout)
  {return fresh(now,stamp,std::min(10000u,TeamActivityPolicy::boundedTimeout(configuredTimeout)));}
  inline float predictionAllowance(Point reported,Point predicted)
  {return finite(reported)&&finite(predicted)?300.f+std::min(600.f,.1f*distance(reported,predicted)):100000.f;}
  inline bool receiverReady(Point reported,Point predicted,Point target)
  {return finite(target)&&distance(predicted,target)+predictionAllowance(reported,predicted)<=900.f;}
  enum class RestartFault {none,context,roster,ballObservation,ballGeometry,noTarget,deadline,attemptLimit};
  inline const char* faultName(RestartFault f)
  {
    switch(f)
    {
      case RestartFault::context:return "context";
      case RestartFault::roster:return "roster";
      case RestartFault::ballObservation:return "ball_observation";
      case RestartFault::ballGeometry:return "ball_geometry";
      case RestartFault::noTarget:return "no_target";
      case RestartFault::deadline:return "gc_deadline";
      case RestartFault::attemptLimit:return "attempt_limit";
      default:return "none";
    }
  }
  struct RestartAttempt
  {
    unsigned epoch=0,started=0;int kind=-1,kicker=-1,receiver=-1;bool cancelled=false;
    bool paused=false,fallbackAllowed=false;
    unsigned firstFaultAt=0,pauseStarted=0,goodSince=0,lastGoodObservation=0,goodSamples=0;
    RestartFault firstFault=RestartFault::none;
    void reset(){*this=RestartAttempt();}
    int status()const{return cancelled?2:paused?1:0;}
    void fail(unsigned now,RestartFault why,bool temporary,bool allowFallback)
    {
      if(cancelled)return;
      if(firstFault==RestartFault::none){firstFault=why;firstFaultAt=now;}
      if(!pauseStarted)pauseStarted=now;
      goodSince=lastGoodObservation=goodSamples=0;
      fallbackAllowed=allowFallback;
      cancelled=!temporary||now<pauseStarted||now-pauseStarted>=1000;
      paused=!cancelled;
    }
    bool enter(unsigned now,unsigned e,int type,int k,int r)
    {
      if(epoch!=e||kind!=type||kicker!=k||receiver!=r)
        *this={e,now,type,k,r,false};
      if(now<started||now-started>20000)fail(now,RestartFault::attemptLimit,false,true);
      return !cancelled;
    }
    bool confirm(unsigned now,unsigned observation)
    {
      if(cancelled)return false;
      if(!paused)return true;
      if(now<pauseStarted||now-pauseStarted>=1000)
      {cancelled=true;paused=false;return false;}
      if(fresh(now,observation,1000)&&observation>=pauseStarted&&observation>lastGoodObservation)
      {
        if(!goodSince)goodSince=now;
        lastGoodObservation=observation;++goodSamples;
      }
      if(goodSamples>=3 && now>=goodSince && now-goodSince>=120)
      {paused=false;pauseStarted=goodSince=lastGoodObservation=goodSamples=0;return true;}
      return false;
    }
  };
  struct FinalTargetSession
  {
    bool active=false,executing=false,retired=false;
    unsigned epoch=0,born=0,changed=0,waitStarted=0,blockedSince=0,revision=0;
    int receiver=-1;Point target{};
    const char* endReason=nullptr;
    void reset(){*this=FinalTargetSession();}
    void bind(unsigned now,unsigned e,int r)
    {if(epoch!=e||receiver!=r){reset();epoch=e;receiver=r;born=now;}}
    bool same(unsigned e,int r)const{return active&&epoch==e&&receiver==r;}
    bool commit(unsigned now,unsigned e,int r,Point point)
    {
      bind(now,e,r);active=true;
      const bool changedTarget=revision==0||distance(point,target)>1.f;
      if(changedTarget){target=point;changed=now;waitStarted=0;++revision;}
      blockedSince=0;
      return changedTarget;
    }
    void wait(unsigned now){if(!waitStarted)waitStarted=now;executing=false;}
    bool expired(unsigned now)const{return retired||now<born||now-born>20000;}
    bool blocked(unsigned now)
    {if(!blockedSince)blockedSince=now;return now<blockedSince||now-blockedSince>=800;}
    bool finish(const char* why)
    {if(retired)return false;retired=true;endReason=why;return true;}
  };
  inline bool committedRaySafe(Point ball,Point committed,Point evaluated,float low,float high,bool valid,bool free)
  {
    if(!valid||!free||!finite(ball)||!finite(committed)||!finite(evaluated)||!std::isfinite(low)||!std::isfinite(high))
      return false;
    const float delta=std::remainder(std::atan2(committed.y-ball.y,committed.x-ball.x)-
                                    std::atan2(evaluated.y-ball.y,evaluated.x-ball.x),6.28318530718f);
    return delta>=low-1e-5f&&delta<=high+1e-5f;
  }
  inline bool urgentWalk(bool scoped,bool keeper,bool stopped,bool poseGood,bool ballFresh,float targetDistance,float ballDistance)
  {return scoped&&!keeper&&!stopped&&poseGood&&ballFresh&&std::isfinite(targetDistance)&&std::isfinite(ballDistance)
       &&targetDistance>1500.f&&ballDistance>1200.f;}
  inline bool passAllowed(bool valid,bool angleFree,bool freeKick,bool closeRequest)
  {return valid&&(angleFree||(!closeRequest&&freeKick));}
  inline bool fieldValid(Field f)
  {return std::isfinite(f.ownGoal)&&std::isfinite(f.opponentGoal)&&std::isfinite(f.ownPenalty)&&std::isfinite(f.halfWidth)&&std::isfinite(f.goalHalfWidth)
      && f.ownGoal<f.ownPenalty&&f.ownPenalty<0&&f.opponentGoal>0&&f.halfWidth>1800&&f.goalHalfWidth>100;}
  inline bool restartBallGeometry(Point ball,Field f)
  {return fieldValid(f)&&finite(ball)&&ball.x>=0&&std::abs(ball.y)<=f.halfWidth+250.f;}
  inline bool inside(Point p,Field f,float margin=800.f)
  {return finite(p)&&fieldValid(f)&&p.x>=f.ownGoal+margin&&p.x<=f.opponentGoal-margin&&std::abs(p.y)<=f.halfWidth-margin;}
  inline float segmentDistance(Point p,Point a,Point b)
  {
    const float x=b.x-a.x,y=b.y-a.y,l=x*x+y*y;
    if(l<1.f)return distance(p,a);
    const float t=std::clamp(((p.x-a.x)*x+(p.y-a.y)*y)/l,0.f,1.f);
    return distance(p,{a.x+t*x,a.y+t*y});
  }
  inline bool closeTargetValid(Point p,Point ball,Point receiver,Field f,const std::vector<Point>& opponents)
  {
    if(!inside(p,f)||!finite(ball)||!finite(receiver)||ball.x<0||distance(p,ball)<2400.f||distance(p,ball)>4200.f)return false;
    for(Point o:opponents)
      if(finite(o)&&(distance(o,p)<900.f||segmentDistance(o,ball,p)<500.f))return false;
    return true;
  }
  struct RestartPlan
  {
    bool valid=false;unsigned epoch=0,changed=0;int kind=0,kicker=-1,receiver=-1;
    Point target{},ball{};
    void reset(){*this=RestartPlan();}
  };
  inline bool planRestart(RestartPlan& plan,unsigned now,unsigned epoch,int kind,int kicker,int receiver,
                          Point ball,Point here,Field f,const std::vector<Point>& opponents,unsigned timeLeft)
  {
    if(!fieldValid(f)||!finite(ball)||!finite(here)||ball.x<0||std::abs(ball.y)>f.halfWidth+250.f
       ||kicker<=0||receiver<=0||kicker==receiver||!epoch||now<epoch||timeLeft<2000)
    {plan.reset();return false;}
    const bool same=plan.valid&&plan.epoch==epoch&&plan.kind==kind&&plan.kicker==kicker&&plan.receiver==receiver&&now>=plan.changed;
    if(same&&now-plan.changed<1200&&distance(ball,plan.ball)<350.f&&closeTargetValid(plan.target,ball,here,f,opponents))return true;
    const float theta=std::atan2(-ball.y,f.opponentGoal-ball.x);
    const float side=ball.y<0?-1.f:1.f;
    float best=-std::numeric_limits<float>::infinity();Point chosen{};bool found=false;
    for(float r:{2500.f,3250.f,4000.f})
      for(float offset:{0.f,.26f,-.26f,.52f,-.52f,.78f,-.78f,1.04f,-1.04f})
      {
        const float a=theta+side*offset;Point p{ball.x+r*std::cos(a),ball.y+r*std::sin(a)};
        if(!closeTargetValid(p,ball,here,f,opponents))continue;
        const float arrival=distance(p,here)/1000.f;
        if(arrival*1000.f+2000.f>timeLeft)continue;
        float clearance=3000.f;for(Point o:opponents)if(finite(o))clearance=std::min(clearance,distance(o,p));
        const float goalDistance=distance(p,{f.opponentGoal,0});
        const float aperture=std::abs(std::atan2(f.goalHalfWidth-p.y,f.opponentGoal-p.x)-std::atan2(-f.goalHalfWidth-p.y,f.opponentGoal-p.x));
        const float nextShotTurn=std::abs(std::remainder(std::atan2(ball.y-p.y,ball.x-p.x)-std::atan2(-p.y,f.opponentGoal-p.x),6.28318530718f));
        const float score=1800.f*aperture-.22f*goalDistance-.35f*distance(p,here)-.10f*r+.15f*clearance-400.f*nextShotTurn
                          +(same&&distance(plan.target,p)<450.f?350.f:0.f);
        if(score>best){best=score;chosen=p;found=true;}
      }
    if(!found){plan.reset();return false;}
    plan={true,epoch,now,kind,kicker,receiver,chosen,ball};return true;
  }
  /** Strong possession proxy, not proof of touch. Unknown is deliberately separate. */
  enum class Possession{unknown,opponent,own};
  inline Possession possession(float ours,float theirs)
  {
    if(!std::isfinite(ours)||!std::isfinite(theirs)||ours<0||theirs<0)return Possession::unknown;
    if(theirs<=1000.f&&ours>=theirs+300.f)return Possession::opponent;
    if(ours<=900.f&&theirs>=ours+300.f)return Possession::own;
    return Possession::unknown;
  }
  struct CoverState
  {
    bool defending=false;unsigned lastOpponent=0,ownSince=0,lastTime=0;int actor=-1;
    void reset(){*this=CoverState();}
  };
  struct ReceiveSession
  {
    bool active=false;unsigned epoch=0,lastRestart=0;int passer=-1;Point launch{},target{};
    unsigned started=0,lastMessage=0;bool retired=false;
    void reset(){*this=ReceiveSession();}
    void suspend(){if(active){active=false;retired=true;}}
  };
  inline bool receiveTarget(ReceiveSession& s,unsigned now,unsigned epoch,int passer,
                            bool restart,bool ordinaryPlay,bool messageValid,Point ball,Point target,Field f)
  {
    if(!messageValid||passer<=0||!inside(target,f,750.f)||(!restart&&!ordinaryPlay))
    {s.suspend();return false;}
    if(restart)
    {
      if(!epoch||now<epoch||!finite(ball)||ball.x<0||distance(ball,target)<1000.f||distance(ball,target)>6500.f)
      {s.suspend();return false;}
      if(s.epoch!=epoch||s.passer!=passer)s={true,epoch,now,passer,ball,target,now,0,false};
      if(s.retired||now<s.started||now-s.started>20000){s.suspend();return false;}
      s.lastRestart=now;s.target=target;return true;
    }
    // Keep the actual announced endpoint briefly after Ball Free. Never push
    // it four metres away from the now-moving ball or carry it into a new pass.
    if(!s.active||s.passer!=passer||now<s.lastRestart||now-s.lastRestart>1500
       ||distance(s.launch,target)<1000.f||distance(s.launch,target)>6500.f)
    {s.suspend();return false;}
    s.target=target;return true;
  }
  inline bool defend(CoverState& s,unsigned now,int actor,Possession evidence)
  {
    if(actor<=0||now<s.lastTime){s.reset();return false;}
    if(actor!=s.actor)s.reset();
    s.actor=actor;s.lastTime=now;
    if(evidence==Possession::opponent){s.defending=true;s.lastOpponent=now;s.ownSince=0;return true;}
    if(evidence==Possession::own)
    {
      if(!s.ownSince)s.ownSince=now;
      if(now-s.ownSince>=250){s.defending=false;return false;}
    }
    else s.ownSince=0;
    if(s.defending&&now>=s.lastOpponent&&now-s.lastOpponent<=800)return true;
    s.defending=false;return false;
  }
  inline bool coverTarget(Point ball,Field f,Point keeper,const std::vector<Point>& mates,Point& target)
  {
    if(!finite(ball)||!fieldValid(f)||!inside(ball,f,0.f))return false;
    const float dx=f.ownGoal-ball.x,dy=-ball.y,l=std::hypot(dx,dy);
    if(l<2200.f)return false;
    float depth=std::clamp(.35f*l,1800.f,3600.f);
    // Do not send the sole cover player into the opponent half while they
    // control the ball. Keep a layer ahead of the keeper, on the SAME ray.
    const float defensiveFront=f.ownPenalty*.75f;
    if(ball.x>defensiveFront)
      depth=std::max(depth,(ball.x-defensiveFront)*l/(ball.x-f.ownGoal));
    target={ball.x+depth*dx/l,ball.y+depth*dy/l};
    // Preserve the keeper's close-goal space; do not force an outside-segment clamp.
    if(!inside(target,f,650.f)||target.x<f.ownPenalty+450.f||target.x>=ball.x)return false;
    if(finite(keeper)&&distance(target,keeper)<1300.f)return false;
    for(Point p:mates)if(finite(p)&&distance(target,p)<1000.f)return false;
    return true;
  }
}
