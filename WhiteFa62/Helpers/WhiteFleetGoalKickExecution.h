#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include "WhiteFleetMatchTacticalRepair.h"

/** Same goal-kick contract only. No new kick, peer or referee permission. */
namespace WhiteFleetGoalKickExecution
{
  inline std::optional<WhiteFleetMatchTacticalRepair::Point> meeting(WhiteFleetMatchTacticalRepair::Point ball,WhiteFleetMatchTacticalRepair::Point peer)
  {
    using namespace WhiteFleetMatchTacticalRepair;
    if(!finite(ball)||!finite(peer)||peer.x<=ball.x+2500.f||distance(ball,peer)<3000.f)return std::nullopt;
    return farReceiverMeeting(ball,peer); // A genuine receiver at (0,0) remains valid.
  }
  inline bool contract(bool valid,bool preparing,bool keeper,int passer,int self,int receiver,int argument,
                       unsigned now,unsigned epoch,WhiteFleetMatchTacticalRepair::Point requested,WhiteFleetMatchTacticalRepair::Point committed)
  {
    return valid&&!preparing&&keeper&&self>0&&passer==self&&receiver>0&&receiver!=self&&receiver==argument&&
      epoch&&now>=epoch&&now-epoch<WhiteFleetMatchTacticalRepair::goalKickContractBudget&&
      WhiteFleetMatchTacticalRepair::finite(requested)&&WhiteFleetMatchTacticalRepair::finite(committed)&&
      WhiteFleetMatchTacticalRepair::distance(requested,committed)<=1.f;
  }
  inline bool effectiveCommand(unsigned expectedAt,unsigned actualAt,bool strongKick,
                               float expectedDirection,float actualDirection,float expectedLength,float actualLength)
  {
    return expectedAt&&actualAt==expectedAt&&strongKick&&std::isfinite(expectedDirection)&&std::isfinite(actualDirection)&&
      std::isfinite(expectedLength)&&std::isfinite(actualLength)&&expectedLength>=500.f&&expectedLength<=12000.f&&
      std::abs(std::remainder(expectedDirection-actualDirection,6.283185307f))<=.02f&&std::abs(expectedLength-actualLength)<=10.f;
  }
  inline bool geometryReady(float distance,float ballAngle,float targetAngle,float poseError,float ballSpeed)
  {
    constexpr float angle=.34906585f; // 20 degrees; not a model-entry threshold.
    return std::isfinite(distance)&&std::isfinite(ballAngle)&&std::isfinite(targetAngle)&&
      std::isfinite(poseError)&&std::isfinite(ballSpeed)&&distance>=150.f&&distance<=650.f&&
      std::abs(ballAngle)<=angle&&std::abs(targetAngle)<=angle&&poseError>=0.f&&poseError<=250.f&&
      ballSpeed>=0.f&&ballSpeed<=650.f;
  }
  struct StrikeClock
  {
    unsigned last=0,elapsed=0,alignedSince=0;
    bool active=false;
    void beginFrame(unsigned now,bool effective)
    {
      if(last&&now<last)elapsed=WhiteFleetMatchTacticalRepair::strikeBudget;
      else if(last&&active&&effective)
        elapsed=std::min(WhiteFleetMatchTacticalRepair::strikeBudget,elapsed+std::min(now-last,WhiteFleetMatchTacticalRepair::strikeBudget));
      // A delayed frame conservatively spends the old permission, not another
      // free attempt; it is NOT evidence of an actual network or foot contact.
      if(last&&now-last>200u)alignedSince=0;
      last=now;active=false;
    }
    void beginFrame(unsigned now){beginFrame(now,true);} // Historical clock fixtures; production passes actual feedback.
    void suspend(){active=false;alignedSince=0;} // Does NOT replenish spent strike time.
    bool admit(unsigned now,bool geometry)
    {
      if(!geometry){suspend();return false;}
      if(!alignedSince)alignedSince=now;
      active=now>=alignedSince&&now-alignedSince>=120u&&elapsed<WhiteFleetMatchTacticalRepair::strikeBudget;
      return active;
    }
    bool exhausted()const{return elapsed>=WhiteFleetMatchTacticalRepair::strikeBudget;}
  };
  struct ObservationSearch
  {
    unsigned epoch=0,lostSince=0,bodyStartedAt=0;
    struct Decision{bool active=false,body=false,otherSide=false;float rotation=0.f;};
    Decision update(unsigned now,unsigned currentEpoch,bool allowed,bool currentBall,float heading)
    {
      if(!currentEpoch||now<currentEpoch||now-currentEpoch>=WhiteFleetMatchTacticalRepair::goalKickContractBudget)
      {*this=ObservationSearch{};return {};}
      if(epoch!=currentEpoch){*this=ObservationSearch{};epoch=currentEpoch;}
      if(currentBall){lostSince=0;return {};}
      if(!allowed||!std::isfinite(heading))return {}; // A fault/pause cannot replenish this epoch's body window.
      if(!lostSince)lostSince=now;
      if(now<lostSince){*this=ObservationSearch{};return {};}
      const unsigned age=now-lostSince;
      const float turn=std::remainder(-heading,6.283185307f);
      Decision d;d.active=true;d.otherSide=(age/1500u)%2u;
      if(age>=400u&&std::abs(turn)>.34906585f&&!bodyStartedAt)bodyStartedAt=now;
      d.body=bodyStartedAt&&now>=bodyStartedAt&&now-bodyStartedAt<6000u&&std::abs(turn)>.34906585f;
      d.rotation=d.body?(turn>0.f?.3f:-.3f):0.f;
      return d;
    }
  };
  inline bool blocksLane(WhiteFleetMatchTacticalRepair::Point ball,WhiteFleetMatchTacticalRepair::Point target,
                         WhiteFleetMatchTacticalRepair::Point position)
  {
    using namespace WhiteFleetMatchTacticalRepair;
    if(!finite(ball)||!finite(target)||!finite(position))return false;
    const float length=distance(ball,target);
    if(length<500.f)return false;
    const float along=((position.x-ball.x)*(target.x-ball.x)+(position.y-ball.y)*(target.y-ball.y))/length;
    const float cross=std::abs((position.x-ball.x)*(target.y-ball.y)-(position.y-ball.y)*(target.x-ball.x))/length;
    return along>=-300.f&&along<=length&&cross<1000.f;
  }
  inline WhiteFleetMatchTacticalRepair::Point coverPoint(WhiteFleetMatchTacticalRepair::Point ball,float ownGoalX,
                                               float halfWidth)
  {
    using namespace WhiteFleetMatchTacticalRepair;
    if(!finite(ball)||!std::isfinite(ownGoalX)||!std::isfinite(halfWidth)||halfWidth<1000.f)return {};
    const float length=std::hypot(ownGoalX-ball.x,-ball.y);
    if(length<1.f)return {ownGoalX+600.f,std::clamp(ball.y,-halfWidth+600.f,halfWidth-600.f)};
    return {std::max(ownGoalX+600.f,ball.x+(ownGoalX-ball.x)*2000.f/length),
      std::clamp(ball.y-ball.y*2000.f/length,-halfWidth+600.f,halfWidth-600.f)};
  }
}
