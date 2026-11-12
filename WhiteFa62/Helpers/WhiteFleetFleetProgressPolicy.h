#pragma once
#include "WhiteFleetMatchTacticalRepair.h"
#include <vector>
#include <limits>

/** Local tactical preferences, never ownership, ball permission or motion limits. */
namespace WhiteFleetFleetProgressPolicy
{
  using Point=WhiteFleetMatchTacticalRepair::Point;
  using WhiteFleetMatchTacticalRepair::finite;
  using WhiteFleetMatchTacticalRepair::distance;
  struct Half
  {
    int value=0,pending=0;unsigned since=0,lastSeen=0,lastNow=0;
    void reset(){*this=Half{};}
    int update(unsigned now,unsigned seen,float x,bool valid)
    {
      if(!valid||!now||!seen||seen>now||now-seen>650||!std::isfinite(x)||now<lastNow||seen<lastSeen)
      {reset();return 0;}
      if(lastNow&&now-lastNow>1000){value=pending=0;since=lastSeen=0;}
      lastNow=now;
      const int wanted=x>300.f?1:x< -300.f?-1:0;
      if(!wanted){pending=0;since=0;lastSeen=seen;return value;}
      if(wanted==value){pending=0;since=0;lastSeen=seen;return value;}
      if(pending!=wanted){pending=wanted;since=seen;}
      if(seen>lastSeen&&seen>=since&&seen-since>=120){value=wanted;pending=0;since=0;}
      lastSeen=seen;return value;
    }
  };
  inline bool frontSupply(Point ball,Point peer,float goalX,float halfWidth)
  {
    const float d=distance(ball,peer);
    return finite(ball)&&finite(peer)&&std::isfinite(goalX)&&std::isfinite(halfWidth)&&ball.x<0.f&&
      peer.x>=500.f&&peer.x>ball.x+500.f&&peer.x<goalX-600.f&&std::abs(peer.y)<halfWidth-600.f&&
      d>=4000.f&&d<=12000.f;
  }
  inline Point sideTarget(int half,Point ball,Point opponent,Point nativeTarget,
                          float ownGoalX,float ownGoalAreaX,float opponentPenaltyX,float halfWidth,int lateralSide)
  {
    if(!half||!finite(ball)||!finite(opponent)||!finite(nativeTarget)||
       !std::isfinite(ownGoalX)||!std::isfinite(ownGoalAreaX)||!std::isfinite(opponentPenaltyX)||!std::isfinite(halfWidth)||
       ownGoalX>=ownGoalAreaX||ownGoalAreaX>=0.f||opponentPenaltyX<=0.f||halfWidth<2000.f)return {NAN,NAN};
    Point p{std::clamp(opponent.x+half*750.f,ownGoalAreaX+300.f,opponentPenaltyX-300.f),opponent.y};
    if(half<0&&ball.x>ownGoalX)
      p.y=ball.y*std::clamp((p.x-ownGoalX)/(ball.x-ownGoalX),0.f,1.f);
    else p.y=.5f*(opponent.y+nativeTarget.y);
    p.y+=lateralSide*600.f;
    p.y=std::clamp(p.y,-halfWidth+700.f,halfWidth-700.f);
    // Clipping cannot silently invert the requested front/back relationship.
    if((p.x-opponent.x)*half<300.f)return {NAN,NAN};
    return p;
  }
}
