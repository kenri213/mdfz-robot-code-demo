#pragma once
#include <algorithm>
#include <cmath>

namespace WhiteFleetDefenseCounterMath
{
  inline bool active(bool flag,bool simulated,int team) { return flag&&simulated&&team==5; }
  struct Point { float x, y; };
  inline float distance(Point a, Point b) { return std::hypot(a.x-b.x,a.y-b.y); }
  inline bool finite(Point p) { return std::isfinite(p.x)&&std::isfinite(p.y); }
  inline float crossingY(Point ball, Point velocity, float plane)
  { return ball.y + velocity.y * ((plane-ball.x)/velocity.x); }
  inline Point keeper(Point ball, Point velocity, float ownGoalX, float halfGoal, bool& prediction)
  {
    const float depth=std::max(150.f,ball.x-ownGoalX);
    const float x=ownGoalX+std::clamp(depth*.20f,350.f,850.f);
    float y=ball.y*(x-ownGoalX)/depth;
    prediction=false;
    if(velocity.x < -300.f)
    {
      const float t=(x-ball.x)/velocity.x;
      const float atGoal=crossingY(ball,velocity,ownGoalX);
      if(t>0.f && t<2.f && std::abs(atGoal)<halfGoal+350.f)
      { y=crossingY(ball,velocity,x);prediction=true; }
    }
    return {x,std::clamp(y,-halfGoal+120.f,halfGoal-120.f)};
  }
  inline Point screen(Point ball, Point velocity, float ownGoalX, float ownPenaltyX,
                      float halfWidth, bool kickoff)
  {
    (void)ownPenaltyX;
    const float x=kickoff ? -1800.f : std::clamp(ball.x-800.f,ownGoalX+750.f,-1600.f);
    float y=ball.y*(x-ownGoalX)/std::max(200.f,ball.x-ownGoalX);
    if(velocity.x < -250.f && ball.x > x+100.f)
    {
      const float t=(x-ball.x)/velocity.x;
      if(t>0.f && t<2.f) y=crossingY(ball,velocity,x);
    }
    return {x,std::clamp(y,-halfWidth+650.f,halfWidth-650.f)};
  }
  inline bool keeperMayClaim(Point ball, Point velocity, Point keeperPose,
                             float ownPenaltyX, float penaltyHalfWidth)
  {
    // Native goalkeeper interception remains available; this gates chasing/clearing only.
    return ball.x<ownPenaltyX+150.f && std::abs(ball.y)<penaltyHalfWidth-100.f &&
           (std::hypot(velocity.x,velocity.y)<650.f || distance(ball,keeperPose)<450.f);
  }
  inline bool shouldScreen(Point self, Point ball, Point velocity, float opponentDistance, bool holding)
  {
    const float d=distance(self,ball);
    if(d<(holding?700.f:1100.f) || ball.x>600.f) return false;
    (void)velocity;
    // Recover only after losing the goal side to an actually observed opponent.
    // A loose ball rolling backwards after our kick is not opponent possession.
    return self.x>ball.x+150.f && opponentDistance<700.f && opponentDistance+250.f<d;
  }
  inline Point outlet(Point ball, float opponentGoalX, float halfWidth, int side)
  { return {std::min(opponentGoalX-900.f,ball.x+2200.f),std::clamp(ball.y+side*950.f,-halfWidth+600.f,halfWidth-600.f)}; }
}
