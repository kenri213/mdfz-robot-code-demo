#pragma once
// Back-field outlet under pressure: when the ball is in our half and an opponent is
// close, do not walk around the ball to aim at the opponent goal; play it at once in
// a direction chosen by one of the tested variants (see VERSIONS.md, backfield drill).
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>
#include "Math/Eigen.h"
#include "Platform/SystemCall.h"

namespace WhiteFleetBackfieldOutlet
{
  enum Variant {off=0,quick=1,wing=2,outlet=3,away=4,hybrid=5};
  inline constexpr int robotDefault=off; // chosen variant for the physical robots
  inline int variant()
  {
    if(SystemCall::getMode()!=SystemCall::simulatedRobot)return robotDefault;
    const char* e=std::getenv("MDFZ_BACKFIELD_VARIANT"); // read live: the scenario drill rotates it per round
    return e?std::atoi(e):int(robotDefault);
  }
  struct Input
  {
    Vector2f ball,robot;                 // field coordinates
    std::vector<Vector2f> opponents;     // field coordinates
    bool frontKnown=false;Vector2f front=Vector2f::Zero(); // front forward (field)
    float halfWidth=7000.f;
  };
  inline float nearestOpponent(const Input& in)
  {
    float d=1e9f;
    for(const auto& o:in.opponents)d=std::min(d,(o-in.ball).norm());
    return d;
  }
  inline bool active(const Input& in){return in.ball.x()<0.f&&nearestOpponent(in)<3000.f;}
  inline float clampForward(float a){constexpr float lim=1.309f;return std::clamp(std::remainder(a,6.2831853f),-lim,lim);} // +-75 deg
  /** Field direction of the kick and its length [mm]. */
  inline float direction(const Input& in,int v,float& length)
  {
    const float approach=std::atan2(in.ball.y()-in.robot.y(),in.ball.x()-in.robot.x());
    // side away from the nearest opponent
    const Vector2f* near=nullptr;float nd=1e9f;
    for(const auto& o:in.opponents){const float d=(o-in.ball).norm();if(d<nd){nd=d;near=&o;}}
    const float side=near?((near->y()>in.ball.y())?-1.f:1.f):(in.ball.y()>=0.f?1.f:-1.f);
    const auto wingDir=[&]{const Vector2f t(in.ball.x()+4000.f,side*(in.halfWidth-1000.f));return std::atan2(t.y()-in.ball.y(),t.x()-in.ball.x());};
    length=9000.f;
    switch(v)
    {
      case quick: return clampForward(approach);
      case wing: return clampForward(wingDir());
      case outlet:
        if(in.frontKnown&&in.front.x()>in.ball.x()+1500.f)
        {
          const Vector2f t=in.front+Vector2f(1500.f,0.f);
          length=std::clamp((t-in.ball).norm(),4000.f,10000.f);
          return clampForward(std::atan2(t.y()-in.ball.y(),t.x()-in.ball.x()));
        }
        return clampForward(wingDir());
      case hybrid:
        // counter-attack when the lane to the front forward is open, else the safest free angle
        if(in.frontKnown&&in.front.x()>in.ball.x()+1500.f)
        {
          const Vector2f t=in.front+Vector2f(1500.f,0.f);
          const float a=clampForward(std::atan2(t.y()-in.ball.y(),t.x()-in.ball.x()));
          float clear=3.1416f;
          for(const auto& o:in.opponents)
          {
            const Vector2f r=o-in.ball;
            if(r.norm()>(t-in.ball).norm())continue;
            clear=std::min(clear,std::abs(std::remainder(std::atan2(r.y(),r.x())-a,6.2831853f)));
          }
          if(clear>=0.4363f) // 25 deg
          {length=std::clamp((t-in.ball).norm(),4000.f,10000.f);return a;}
        }
        [[fallthrough]];
      case away:
      default:
      {
        float best=-1e9f,bestA=0.f;
        for(int i=-15;i<=15;++i)
        {
          const float a=i*0.0872665f; // 5 deg steps in [-75,75]
          float clear=1.0472f;        // 60 deg cap
          for(const auto& o:in.opponents)
          {
            const Vector2f r=o-in.ball;
            if(r.norm()>5000.f)continue;
            clear=std::min(clear,std::abs(std::remainder(std::atan2(r.y(),r.x())-a,6.2831853f)));
          }
          const float score=clear+0.3f*std::cos(a)-0.2f*std::abs(std::remainder(a-approach,6.2831853f));
          if(score>best){best=score;bestA=a;}
        }
        return bestA;
      }
    }
  }
}
