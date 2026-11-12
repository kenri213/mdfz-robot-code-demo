#pragma once

#include "WhiteFleetKickoffThroughV8Math.h"

/** Receiver-only navigation estimates. These are not capture/kick certificates.
 * The caller supplies native BallPhysics and native obstacle-route estimates.
 * No GT, altered friction, fixed field destination, or observation renewal. */
namespace WhiteFleetC15ReceiverRoute
{
  using WhiteFleetKickoffThroughV8::Point;
  using WhiteFleetKickoffThroughV8::finite;
  using WhiteFleetKickoffThroughV8::distance;
  constexpr float pi=3.14159265358979323846f;
  struct Circle {Point center{};float radius=0.f;bool clockwise=false;};
  struct Route {bool valid=false;float length=0.f,initialTurn=0.f,arcTurn=0.f;};
  struct Choice {bool valid=false;Point ball{},body{};float eta=0.f,ballTime=0.f,length=0.f;};

  inline float angle(float a){return std::atan2(std::sin(a),std::cos(a));}
  inline Point add(Point a,Point b){return {a.x+b.x,a.y+b.y};}
  inline Point sub(Point a,Point b){return {a.x-b.x,a.y-b.y};}
  inline Point mul(Point a,float t){return {a.x*t,a.y*t};}

  // Exact tangent/arc length for the native ordered circle route. Overlapping
  // control circles or a start/target inside one are not a certified route.
  inline Route measure(Point target,const std::vector<Circle>& obstacles)
  {
    Route out;
    if(!finite(target)||distance(target,{})<=1.f||obstacles.size()>16)return out;
    std::vector<Circle> nodes{{{},0.f,false}};
    for(const auto& c:obstacles)
    {
      if(!finite(c.center)||!std::isfinite(c.radius)||c.radius<=0.f)return out;
      nodes.push_back(c);
    }
    nodes.push_back({target,0.f,false});
    std::vector<Point> entry(nodes.size()),exit(nodes.size());
    for(size_t n=0;n+1<nodes.size();++n)
    {
      const auto& a=nodes[n];const auto& b=nodes[n+1];
      const Point d=sub(b.center,a.center);const float length=distance(d,{});
      const float ra=a.clockwise?-a.radius:a.radius,rb=b.clockwise?-b.radius:b.radius;
      if(!std::isfinite(length)||length<=std::abs(ra-rb)+1.f)return out;
      const Point u=mul(d,1.f/length);const float cosine=(ra-rb)/length;
      const float sine=std::sqrt(std::max(0.f,1.f-cosine*cosine));
      const Point normal{cosine*u.x+sine*u.y,cosine*u.y-sine*u.x};
      exit[n]=add(a.center,mul(normal,ra));entry[n+1]=add(b.center,mul(normal,rb));
      const Point segment=sub(entry[n+1],exit[n]);
      out.length+=distance(segment,{});
      if(n==0)out.initialTurn=std::abs(std::atan2(segment.y,segment.x));
    }
    for(size_t n=1;n+1<nodes.size();++n)
    {
      const Point a=sub(entry[n],nodes[n].center),b=sub(exit[n],nodes[n].center);
      float arc=angle(std::atan2(b.y,b.x)-std::atan2(a.y,a.x));
      if(nodes[n].clockwise)arc=-arc;
      if(arc<0.f)arc+=2.f*pi;
      out.length+=arc*nodes[n].radius;out.arcTurn+=arc;
    }
    out.valid=std::isfinite(out.length)&&out.length>=distance(target,{})-.01f;
    return out;
  }

  template<class Predict,class Assess>
  Choice choose(Point robot,Point contract,const WhiteFleetKickoffThroughV8::Field& field,
                const WhiteFleetKickoffThroughV8::Params& p,float maxForward,float maxRotation,
                float remaining,Predict predict,Assess assess)
  {
    Choice best;
    const float speed=std::min(maxForward*p.receiverSpeedFactor,p.empiricalReceiverSpeed);
    const float yaw=std::min(maxRotation,1.5f);
    if(!WhiteFleetKickoffThroughV8::paramsValid(p)||!WhiteFleetKickoffThroughV8::fieldValid(field)
       ||!std::isfinite(maxForward)||maxForward<=0.f||!std::isfinite(maxRotation)||maxRotation<=0.f
       ||!finite(robot)||!finite(contract)||!std::isfinite(speed)||speed<=0.f
       ||!std::isfinite(yaw)||yaw<=0.f||!std::isfinite(remaining)||remaining<=0.f)return best;
    unsigned routeQueries=0;bool cached=false;Point cachedBody{};Route cachedRoute;
    // Finite future trajectory, bounded by both the original flight budget
    // and the original 1500mm receiving region around the locked contract.
    for(unsigned sample=1;sample<=32;++sample)
    {
      const float t=sample*.25f;
      if(t+p.arrivalMargin>remaining)break;
      const Point ball=predict(t);
      if(!finite(ball)||distance(ball,contract)>1500.f||ball.x<field.ownGoal+300.f
         ||ball.x>field.opponentGoal-300.f||std::abs(ball.y)>field.halfWidth-300.f)continue;
      const Point ray=sub(ball,robot);const float length=distance(ray,{});
      if(!std::isfinite(length)||length<=1500.f)continue;
      // Enter the near side of the existing 650mm capture disc, not its far
      // side or the exact contract point. Fresh near-ball logic takes over
      // before this body destination is reached.
      const Point body=sub(ball,mul(ray,650.f/length));
      // Cheap necessary bound before invoking the stateful native planner.
      // Eight route queries at most per genuine replan; never an unbounded
      // candidate scan on the real-time Cognition thread. Stopped-ball repeats
      // reuse only the same candidate's geometry, not an old motion request.
      if((length-650.f)/speed+p.arrivalMargin>remaining)continue;
      if(!cached||distance(body,cachedBody)>.001f)
      {
        if(routeQueries>=8)break;
        ++routeQueries;cachedRoute=assess(body);cachedBody=body;cached=true;
      }
      const Route route=cachedRoute;
      if(!route.valid)continue;
      const float eta=route.length/speed+(route.initialTurn+route.arcTurn)/yaw;
      // This is navigation into the receiving region, not a certificate that
      // we intercept the still-rolling ball at t. Late arrival must not switch
      // control back to the old chase. Keep the original mission budget.
      if(!std::isfinite(eta)||eta+p.arrivalMargin>remaining)continue;
      if(!best.valid||eta<best.eta-.001f)
        best={true,ball,body,eta,t,route.length};
    }
    return best;
  }

  inline Point boundedGapStep(Point route,Point snapshotBall,Point currentBall)
  {
    if(!finite(route)||!finite(snapshotBall)||!finite(currentBall))return {};
    const float length=distance(route,{});
    const float step=std::min({length,1200.f,distance(snapshotBall,{})-1500.f,
                                             distance(currentBall,{})-1500.f});
    // Triangle inequality keeps the ENTIRE bounded step outside both old and
    // current ball estimates' original1500mm front margin, not just its end.
    if(!std::isfinite(length)||!std::isfinite(step)||step<=1.f||length<=1.f)return {};
    return mul(route,step/length);
  }

  struct State
  {
    unsigned event=0,seen=0,plannedAt=0,arrivesAt=0,diagnostics=0;
    bool valid=false,near=false;
    Point targetOdometry{},ballOdometry{},expectedBallOdometry{},originField{},originOdometry{},lastOdometry{};
    float originHeading=0.f,originOdoHeading=0.f;
    float lastOdoHeading=0.f;
    unsigned lastFrame=0;
    void begin(unsigned e){if(event!=e){*this=State();event=e;}}
    void invalidate(){valid=false;}
    bool coherent(unsigned now,Point odo,float heading,Point field,float fieldHeading,float speed,float yaw)const
    {
      if(!valid||!lastFrame||now<lastFrame||!finite(odo)||!finite(field)
         ||!std::isfinite(heading)||!std::isfinite(fieldHeading)
         ||!std::isfinite(speed)||speed<=0.f||!std::isfinite(yaw)||yaw<=0.f)return false;
      const float dt=(now-lastFrame)*.001f;
      const float r=originHeading-originOdoHeading;
      const Point d=sub(odo,originOdometry);
      const Point anchored=add(originField,{std::cos(r)*d.x-std::sin(r)*d.y,
                                            std::sin(r)*d.x+std::cos(r)*d.y});
      return distance(odo,lastOdometry)<=speed*dt+100.f&&std::abs(angle(heading-lastOdoHeading))<=yaw*dt+.1f
          &&distance(anchored,field)<=300.f
          &&std::abs(angle(originHeading+heading-originOdoHeading-fieldHeading))<=.1745329252f;
    }
  };
}
