#pragma once
#include "WhiteFleetC15ReceiverRoute.h"
#include <array>
#include <cstring>
#include <utility>

/** Tactical off-ball corridors, not a collision oracle or capture certificate.
 * No simulation interfaces, GT, saved opponent trajectories, or actuator writes. */
namespace WhiteFleetC15EarlyChannel
{
  using WhiteFleetC15ReceiverRoute::Point;using WhiteFleetC15ReceiverRoute::Circle;using WhiteFleetC15ReceiverRoute::Route;
  using WhiteFleetC15ReceiverRoute::finite;using WhiteFleetC15ReceiverRoute::distance;using WhiteFleetC15ReceiverRoute::angle;
  using WhiteFleetC15ReceiverRoute::add;using WhiteFleetC15ReceiverRoute::sub;using WhiteFleetC15ReceiverRoute::mul;
  using WhiteFleetC15ReceiverRoute::measure;using WhiteFleetC15ReceiverRoute::boundedGapStep;
  struct NativePath:Route{std::vector<Point> samples;bool redirected=false;};
  inline NativePath nativePath(Point target,const std::vector<Circle>& circles)
  {
    NativePath p;static_cast<Route&>(p)=measure(target,circles);if(!p.valid)return p;
    std::vector<Circle> nodes{{{},0,false}};for(auto c:circles)nodes.push_back(c);nodes.push_back({target,0,false});
    std::vector<Point> enter(nodes.size()),leave(nodes.size());
    for(size_t n=0;n+1<nodes.size();++n)
    {
      const auto& a=nodes[n];const auto& b=nodes[n+1];const Point d=sub(b.center,a.center);const float length=distance(d,{});
      const float ra=a.clockwise?-a.radius:a.radius,rb=b.clockwise?-b.radius:b.radius,c=(ra-rb)/length;
      const Point u=mul(d,1.f/length);const float s=std::sqrt(std::max(0.f,1.f-c*c));
      const Point normal{c*u.x+s*u.y,c*u.y-s*u.x};leave[n]=add(a.center,mul(normal,ra));enter[n+1]=add(b.center,mul(normal,rb));
    }
    p.samples.push_back({});
    for(size_t n=1;n<nodes.size();++n)
    {
      p.samples.push_back(enter[n]);if(n+1==nodes.size())break;
      const Point a=sub(enter[n],nodes[n].center),b=sub(leave[n],nodes[n].center);
      const float start=std::atan2(a.y,a.x);float arc=angle(std::atan2(b.y,b.x)-start);
      if(nodes[n].clockwise)arc=-arc;if(arc<0)arc+=2*WhiteFleetC15ReceiverRoute::pi;
      const unsigned steps=std::max(1u,static_cast<unsigned>(std::ceil(arc/.04f)));
      for(unsigned k=1;k<=steps;++k){const float t=start+(nodes[n].clockwise?-1.f:1.f)*arc*k/steps;
        p.samples.push_back(add(nodes[n].center,{nodes[n].radius*std::cos(t),nodes[n].radius*std::sin(t)}));}
    }
    return p;
  }
  inline Point along(const std::vector<Point>& points,float fraction)
  {
    float total=0;for(size_t k=1;k<points.size();++k)total+=distance(points[k],points[k-1]);
    float wanted=total*fraction;
    for(size_t k=1;k<points.size();++k){float d=distance(points[k],points[k-1]);if(wanted<=d&&d>0)return add(points[k-1],mul(sub(points[k],points[k-1]),wanted/d));wanted-=d;}
    return points.empty()?Point{}:points.back();
  }
  struct Obstacle
  {
    Point center{},velocity{};
    float radius=600.f,uncertainty=300.f,speedBound=2200.f;
    unsigned seen=0;
    bool velocityKnown=false,local=true,opponent=true;
    bool identifiedOpponent=false; // Exact native classification, not "not a teammate".
  };
  struct Input
  {
    unsigned now=0,event=0,seen=0;
    Point robot{},odometry{},ball{},relativeBall{},velocity{};
    float heading=0.f,odoHeading=0.f,estimatedSpeed=0.f,maxForward=0.f,maxSide=0.f,maxYaw=0.f;
    float positionSigma=1e9f,headingSigma=1e9f;
    bool estimatedSpeedKnown=false;unsigned unknownOpponents=0;
    bool localStep=false; // Explicit isolated policy; false is the exact b872 decision path.
    bool shortest=false;
    bool fixedDirect=false;Point fixedBall{6940.f,-1050.f};
    bool fixedRunToArrival=false,fixedOwnFresh=false,fixedReleaseWitness=false;
    bool boundedFastRun=false;
    unsigned fixedContextFrame=0,fixedContextRelease=0;int fixedContextKicker=0,fixedContextReceiver=0;
    float preferredMinX=6650.f,preferredMaxX=7250.f,preferredMinY=-1300.f,preferredMaxY=-800.f;
    bool lowerChannel=false,lowerLocked=false,lowerPassed=false,lowerDefenderKnown=false;
    Obstacle lowerDefender{}; // Assigned native local observation, never a fabricated global identity.
    WhiteFleetKickoffThroughV8::Field field{};WhiteFleetKickoffThroughV8::Params params{};
    Point contract{};
    std::vector<Obstacle> obstacles;
  };
  struct Plan
  {
    bool valid=false,uncertain=false;
    bool localStepOnly=false; // A safe first leg is NOT a full-route/capture certificate.
    bool fixedDirect=false;
    bool shortest=false,shortestHasGoal=false,shortestPreferred=false,shortestTimely=false;Point tangent{};
    float shortestLength=0.f,shortestEta=0.f,shortestTurn=0.f;
    bool lower=false,lowerFutureRisk=false;unsigned lane=0,defenderSeen=0;Point defender{},entry{},exit{};float defenderRadius=0.f,defenderUncertainty=300.f;
    int side=0;
    std::array<Point,3> points{};unsigned size=0;
    Point ball{};
    float eta=0.f,ballTime=0.f,length=0.f,clearance=1e9f,softClearance=1e9f,opponentEta=1e9f,score=1e9f;
    float firstLegEta=0.f,futureClearance=1e9f;
    unsigned samplesTested=0,routesTested=0,nativeRejected=0,occupiedRejected=0,illegalRejected=0,budgetRejected=0;
    unsigned lowerRejected=0;
    const char* reason="no_candidate";
  };
  inline Point rotate(Point p,float a)
  {return {std::cos(a)*p.x-std::sin(a)*p.y,std::sin(a)*p.x+std::cos(a)*p.y};}
  inline Point toOdo(Point p,const Input& i)
  {return add(i.odometry,rotate(sub(p,i.robot),i.odoHeading-i.heading));}
  inline Point toField(Point p,const Input& i)
  {return add(i.robot,rotate(sub(p,i.odometry),i.heading-i.odoHeading));}
  inline bool inside(Point p,const Input& i)
  {return finite(p)&&p.x>=i.field.ownGoal+300.f&&p.x<=i.field.opponentGoal-300.f&&std::abs(p.y)<=i.field.halfWidth-300.f;}

  struct Memory
  {
    struct Track{Point point{},velocity{};unsigned seen=0,time=0;bool known=false;};
    std::array<Track,8> tracks{};
    void update(Obstacle& o,unsigned now)
    {
      if(!o.local||!o.seen||o.seen>now||now-o.seen>650u)return;
      Track* best=nullptr;float d1=1e9f,d2=1e9f;
      for(auto& t:tracks)if(t.time&&now>=t.time&&now-t.time<=650u)
      {
        const float d=distance(o.center,t.point);
        if(d<d1){d2=d1;d1=d;best=&t;}else d2=std::min(d2,d);
      }
      if(best&&d1<=o.speedBound*(now-best->time)*.001f+300.f&&d2-d1>300.f)
      {
        if(o.seen>best->seen&&now>best->time)
        {
          const Point v=mul(sub(o.center,best->point),1000.f/(now-best->time));
          best->known=finite(v)&&distance(v,{})<=o.speedBound;
          best->velocity=v;best->point=o.center;best->time=now;best->seen=o.seen;
        }
        o.velocityKnown=best->known&&now-o.seen<=250u;
        if(o.velocityKnown)o.velocity=best->velocity;
      }
      else
      {
        auto* slot=&tracks[0];for(auto& t:tracks)if(t.time<slot->time)slot=&t;
        *slot={o.center,{},o.seen,now,false};
      }
    }
  };

  inline float travel(float length,float speed,const Input& i)
  {
    const float cap=std::min(i.maxForward*i.params.receiverSpeedFactor,i.params.empiricalReceiverSpeed);
    if(!std::isfinite(cap)||cap<=0.f)return 1e9f;
    // Measured/estimated current motion affects startup cost, not a claim that
    // an upper speed is already attained or that odometry is physical GT.
    return length/cap+i.params.startDelay*(1.f-std::clamp(speed/cap,0.f,1.f));
  }
  template<class Native,class Legal>
  Plan evaluate(Plan p,const Input& i,Native native,Legal legal)
  {
    p.valid=false;p.localStepOnly=false;p.lowerFutureRisk=false;p.firstLegEta=0.f;p.futureClearance=1e9f;
    p.uncertain=!i.estimatedSpeedKnown||i.unknownOpponents>0||i.positionSigma>300.f||i.headingSigma>.1745329252f;
    p.length=0.f;p.clearance=p.softClearance=p.opponentEta=1e9f;
    p.samplesTested=0;p.routesTested=1;p.nativeRejected=p.occupiedRejected=p.illegalRejected=p.budgetRejected=p.lowerRejected=0;
    const auto reject=[&](const char* why){p.reason=why;
      if(std::strcmp(why,"native_no_route")==0)++p.nativeRejected;
      if(std::strcmp(why,"occupied_arrival_corridor")==0||std::strcmp(why,"immediate_dynamic_block")==0)++p.occupiedRejected;
      if(std::strcmp(why,"illegal_segment")==0||std::strcmp(why,"outside_field")==0)++p.illegalRejected;
      if(std::strcmp(why,"event_budget")==0)++p.budgetRejected;
      if(std::strcmp(why,"lower_side_required")==0)++p.lowerRejected;
      return p;};
    if(!p.size||p.size>3||!inside(i.robot,i)||i.now<i.event||!WhiteFleetKickoffThroughV8::paramsValid(i.params)
       ||!WhiteFleetKickoffThroughV8::fieldValid(i.field))return reject("invalid_parameters");
    const float cap=std::min(i.maxForward*i.params.receiverSpeedFactor,i.params.empiricalReceiverSpeed);
    const float sideCap=std::min(i.maxSide*i.params.receiverSpeedFactor,900.f);
    if(!std::isfinite(cap)||cap<=0.f||!std::isfinite(sideCap)||sideCap<=0.f
       ||!std::isfinite(i.maxYaw)||i.maxYaw<=0.f||!std::isfinite(i.heading)||!std::isfinite(i.estimatedSpeed))return reject("invalid_motion_limits");
    Point from=i.robot;float elapsed=i.params.startDelay*(1.f-std::clamp((i.estimatedSpeedKnown?i.estimatedSpeed:0.f)/cap,0.f,1.f));
    float previousDirection=i.heading;
    for(unsigned n=0;n<p.size;++n)
    {
      const Point to=p.points[n];if(!inside(to,i))return reject("outside_field");
      const float straight=distance(from,to);if(!std::isfinite(straight)||straight<1.f)continue;
      for(unsigned s=0;s<=static_cast<unsigned>(straight/250.f)+1u;++s)
      {
        const float f=std::min(1.f,s*250.f/straight);
        if(!legal(add(from,mul(sub(to,from),f))))return reject("illegal_segment");
      }
      float length=straight,turn=std::abs(angle(std::atan2(to.y-from.y,to.x-from.x)-previousDirection));
      std::vector<Point> firstPath;
      if(n==0)
      {
        const auto route=native(to);
        if(!route.valid)return reject("native_no_route");
        length=route.length;turn=route.initialTurn+route.arcTurn;
        firstPath=route.samples;
      }
      const float direction=std::atan2(to.y-from.y,to.x-from.x);
      const float openHeading=std::atan2(-p.ball.y,i.field.opponentGoal-p.ball.x);
      const float a=angle(direction-openHeading);
      const float effectiveSpeed=1.f/std::hypot(std::cos(a)/cap,std::sin(a)/sideCap);
      const float duration=length/effectiveSpeed+turn/std::min(i.maxYaw,1.5f);
      for(unsigned s=1;s<=12;++s)
      {
        const float f=s/12.f,t=elapsed+f*duration;
        const Point body=n==0&&!firstPath.empty()?along(firstPath,f):add(from,mul(sub(to,from),f));
        // FIELD coordinates always attack +X. A native curve to a lower
        // endpoint is not sufficient if the curve actually goes above the
        // assigned defender. Keep the whole shoulder-crossing band below it.
        if(p.lower&&body.x>=p.defender.x-p.defenderRadius&&body.x<=p.defender.x+p.defenderRadius
           &&body.y>p.defender.y-p.defenderRadius)
        {
          if(n==0)return reject("lower_side_required");
          p.lowerFutureRisk=true; // Recheck when this becomes the actual first leg, not a new distant stop gate.
        }
        for(const auto& o:i.obstacles)
        {
          if(!finite(o.center)||!std::isfinite(o.radius)||o.radius<=0.f||!std::isfinite(o.speedBound)||o.speedBound<=0.f
             ||!std::isfinite(o.uncertainty)||o.uncertainty<0.f||(o.velocityKnown&&!finite(o.velocity)))
            return reject("invalid_obstacle");
          const Point predicted=o.velocityKnown?add(o.center,mul(o.velocity,std::min(t,1.f))):o.center;
          const float clear=distance(body,predicted)-o.radius;
          // Local nominal footprint is a hard routing obstruction. Global
          // centre-only estimates and uncertainty are TACTICAL scoring only.
          if(o.local)
          {
            if(!i.localStep||n==0)p.clearance=std::min(p.clearance,clear);
            if(n>0)p.futureClearance=std::min(p.futureClearance,clear);
          }
          const float uncertainty=o.uncertainty+(!o.velocityKnown?o.speedBound*std::min(t,.2f):0.f);
          p.softClearance=std::min(p.softClearance,clear-uncertainty);
          p.uncertain|=!o.velocityKnown;
          if(o.local&&t<=.25f&&clear< -1.f)return reject("immediate_dynamic_block");
        }
      }
      p.length+=length;elapsed+=duration;previousDirection=std::atan2(to.y-from.y,to.x-from.x);from=to;
      if(n==0)p.firstLegEta=elapsed;
    }
    if(p.clearance< -1.f)return reject("occupied_arrival_corridor");
    const float remaining=std::max(0.f,static_cast<float>(std::min(i.params.flight,i.params.total))-
        static_cast<float>(i.now-i.event))*.001f;
    p.eta=elapsed;
    if(!std::isfinite(p.eta))return reject("event_budget");
    const bool wholeRouteLate=p.eta+i.params.arrivalMargin>remaining;
    // Keep the entire actually selected FIRST native leg hard-checked. A late
    // full route may only advance that leg if it fits the ORIGINAL remaining
    // budget. No deadline extension, shortened ball lease or discarded route
    // may be used to manufacture reachability of the receiving point.
    if(wholeRouteLate&&(!i.localStep||p.firstLegEta<=0.f||!std::isfinite(p.firstLegEta)
        ||p.firstLegEta+i.params.arrivalMargin>remaining))return reject("event_budget");
    p.localStepOnly=i.localStep&&(wholeRouteLate||p.futureClearance< -1.f||p.lowerFutureRisk);
    for(const auto& o:i.obstacles)if(o.opponent)
      p.opponentEta=std::min(p.opponentEta,std::max(0.f,distance(o.center,p.ball)-o.radius)/o.speedBound);
    // The opponent value is an optimistic reach bound, not an invented
    // accurate ETA. Prefer room and lead without declaring all uncertain
    // future locations impossible to traverse.
    p.score=p.eta+std::max(0.f,-p.softClearance)/cap+
        std::max(0.f,p.eta-p.opponentEta)*.2f+std::max(0.f,p.eta-p.ballTime-i.params.receiveWait);
    if(i.localStep)p.score+=2.f*std::max(0.f,p.eta+i.params.arrivalMargin-remaining);
    if(p.lowerFutureRisk)p.score+=i.params.arrivalMargin;
    p.valid=true;p.reason="channel_ready";return p;
  }

  inline bool lowerBlocker(const Input& i,Point body,Point ball,Obstacle& selected)
  {
    if(!i.lowerChannel||i.lowerPassed)return false;
    if(i.lowerLocked&&i.lowerDefenderKnown){selected=i.lowerDefender;return true;}
    const Obstacle* found=nullptr;float first=1e9f;
    for(const auto& o:i.obstacles)
    {
      if(!o.local||!o.opponent||!o.identifiedOpponent||!o.seen||o.seen>i.now||i.now-o.seen>250u
         ||o.center.x<=i.robot.x||o.center.x>=ball.x||i.ball.y<=o.center.y||ball.y<=o.center.y)continue;
      if(WhiteFleetKickoffThroughV8::pointSegmentDistance(o.center,i.robot,body)>o.radius+o.uncertainty+300.f)continue;
      if(o.center.x<first){first=o.center.x;found=&o;}
    }
    if(!found)return false;selected=*found;return true;
  }

  template<class Predict,class Native,class Legal>
  Plan choose(const Input& i,Predict predict,Native native,Legal legal)
  {
    Plan best;unsigned calls=0,samples=0,attempts=0,nativeBad=0,occupiedBad=0,illegalBad=0,budgetBad=0,lowerBad=0;
    const char* lastReason="no_receiving_sample";
    // Pick at most two distinct receiving-region samples, then only three
    // corridor families for each. No unbounded planner search on Cognition.
    std::vector<std::pair<Point,float>> goals;
    for(unsigned k=1;k<=32&&goals.size()<2;++k)
    {
      const float t=k*.25f;
      if(i.now<i.event||t+i.params.arrivalMargin>
          (static_cast<float>(std::min(i.params.flight,i.params.total))-static_cast<float>(i.now-i.event))*.001f)break;
      Point ball=predict(t);
      ++samples;
      if(!inside(ball,i)||distance(ball,i.contract)>1500.f||distance(ball,i.robot)<=1000.f)continue;
      if(!goals.empty()&&distance(ball,goals.back().first)<300.f)continue;
      goals.push_back({ball,t});
    }
    for(const auto& [ball,t]:goals)
    {
      const Point ray=sub(ball,i.robot);const float length=distance(ray,{});
      const Point u=mul(ray,1.f/length),normal{-u.y,u.x};
      const Point body=sub(ball,mul(u,650.f));
      Obstacle assigned;const bool lower=lowerBlocker(i,body,ball,assigned);
      if(lower)
      {
        const float base=std::min(1500.f,assigned.radius+assigned.uncertainty+assigned.speedBound*.2f+i.params.obstaclePadding);
        // Three adjacent corridors on the SAME canonical -Y side. The
        // direct/upper families are not candidates until the shoulder passed.
        for(unsigned lane=0;lane<3;++lane)
        {
          const float offset=base+lane*2.f*i.params.obstaclePadding;
          Plan p;p.lower=true;p.side=-1;p.lane=lane;p.defender=assigned.center;
          p.defenderRadius=assigned.radius;p.defenderUncertainty=assigned.uncertainty;p.defenderSeen=assigned.seen;p.ball=ball;p.ballTime=t;
          p.entry={std::max(assigned.center.x-offset,i.robot.x+300.f),assigned.center.y-offset};
          p.exit={std::max(assigned.center.x+offset,p.entry.x+300.f),p.entry.y};
          p.points[p.size++]=p.entry;p.points[p.size++]=p.exit;p.points[p.size++]=body;
          const auto limited=[&](Point target){if(calls++>=6)return NativePath{};return native(target);};
          p=evaluate(p,i,limited,legal);
          ++attempts;nativeBad+=p.nativeRejected;occupiedBad+=p.occupiedRejected;
          illegalBad+=p.illegalRejected;budgetBad+=p.budgetRejected;lowerBad+=p.lowerRejected;
          if(!p.valid)lastReason=p.reason;
          const bool moreRoom=p.softClearance>=0.f&&best.softClearance<0.f;
          const bool sameRoom=(p.softClearance>=0.f)==(best.softClearance>=0.f);
          if(p.valid&&(!best.valid||moreRoom||(sameRoom&&p.score<best.score)))best=p;
        }
        continue;
      }
      const Obstacle* blocker=nullptr;float earliest=1e9f;bool hardBlock=false;
      for(const auto& o:i.obstacles)
      {
        const float initial=sub(o.center,i.robot).x*u.x+sub(o.center,i.robot).y*u.y;
        const float at=travel(std::max(0.f,initial),i.estimatedSpeed,i);
        const Point center=o.velocityKnown?add(o.center,mul(o.velocity,std::min(at,1.f))):o.center;
        const float a=sub(center,i.robot).x*u.x+sub(center,i.robot).y*u.y;
        const float clearance=WhiteFleetKickoffThroughV8::pointSegmentDistance(center,i.robot,body)-o.radius;
        const bool hard=o.local&&clearance<0.f;
        if(a<=0.f||a>=length||(hardBlock&&!hard)||((hard==hardBlock)&&a>=earliest))continue;
        if(clearance<o.uncertainty+300.f){blocker=&o;earliest=a;hardBlock=hard;}
      }
      for(int side:{0,-1,1})
      {
        if(side&&!blocker)continue;
        Plan p;p.side=side;p.ball=ball;p.ballTime=t;
        if(side)
        {
          const float offset=std::min(1500.f,blocker->radius+blocker->uncertainty+blocker->speedBound*.2f+i.params.obstaclePadding);
          const float at=travel(earliest,i.estimatedSpeed,i);
          const Point center=blocker->velocityKnown?add(blocker->center,mul(blocker->velocity,std::min(at,1.f))):blocker->center;
          const Point lateral=mul(normal,side*offset);
          const Point entry=add(sub(center,mul(u,offset)),lateral);
          const Point exit=add(add(center,mul(u,offset)),lateral);
          if(sub(entry,i.robot).x*u.x+sub(entry,i.robot).y*u.y>300.f)p.points[p.size++]=entry;
          p.points[p.size++]=exit;
        }
        p.points[p.size++]=body;
        const auto limited=[&](Point target){if(calls++>=6)return NativePath{};return native(target);};
        p=evaluate(p,i,limited,legal);
        ++attempts;nativeBad+=p.nativeRejected;occupiedBad+=p.occupiedRejected;illegalBad+=p.illegalRejected;budgetBad+=p.budgetRejected;
        if(!p.valid)lastReason=p.reason;
        const bool moreRoom=p.softClearance>=0.f&&best.softClearance<0.f;
        const bool sameRoomClass=(p.softClearance>=0.f)==(best.softClearance>=0.f);
        if(p.valid&&(!best.valid||moreRoom||(sameRoomClass&&p.score<best.score)))best=p;
      }
    }
    best.samplesTested=samples;best.routesTested=attempts;best.nativeRejected=nativeBad;
    best.occupiedRejected=occupiedBad;best.illegalRejected=illegalBad;best.budgetRejected=budgetBad;
    best.lowerRejected=lowerBad;
    if(!best.valid)best.reason=lastReason;
    return best;
  }

  inline bool handoff(const Input& i)
  {
    if(!i.seen||i.seen>i.now||i.now-i.seen>250u||!finite(i.relativeBall))return false;
    const float range=distance(i.relativeBall,{}),front=std::abs(std::atan2(i.relativeBall.y,i.relativeBall.x));
    const float shot=std::abs(angle(std::atan2(-i.ball.y,i.field.opponentGoal-i.ball.x)-i.heading));
    if(range<=650.f)return front<=.785398164f&&shot<=.785398164f; // Original front45/native foot gates.
    if(range>1000.f||front>.523598776f||shot>.523598776f)return false;
    for(const auto& o:i.obstacles)if(o.local&&distance(o.center,i.ball)<1300.f)return false;
    return true;
  }

  enum class FixedArrivalPhase:unsigned{unarmed,run,arrived};
  struct State
  {
    unsigned event=0,lastFrame=0,lastSeen=0,lastObstacleSeen=0,plannedAt=0,committedAt=0,generation=0,logs=0,lastLog=0;
    bool near=false,clockFault=false,odometryFault=false,epochPending=false,awaitingEpochCommit=false;Plan plan{};Memory memory;
    bool lowerCommitted=false,lowerPassed=false;unsigned lowerSeen=0,lowerCutAt=0,lowerLane=0,lowerChanges=0;
    Point lowerDefenderOdo{},lowerEntryOdo{},lowerExitOdo{};float lowerRadius=0.f,lowerUncertainty=300.f;
    const char* lowerReason="not_assigned";
    bool shortestPrefixValid=false,shortestAreaAvailable=false;
    unsigned shortestPrefixSeen=0;Point shortestPrefixOdo{};
    const char* shortestReason="not_planned";
    bool fixedReferenceKnown=false;Point fixedReferenceBall{},fixedReferenceBody{},fixedReferenceContract{},fixedOrigin{};
    float fixedInitialDirection=0.f;
    FixedArrivalPhase fixedArrivalPhase=FixedArrivalPhase::unarmed;bool fixedArrivalRevoked=false;
    unsigned fixedAuthorizedAt=0,fixedAuthSeen=0,fixedSessionRelease=0,fixedArrivedAt=0,fixedFreshAfterArrival=0;
    unsigned fixedFirstRunOutput=0,fixedFirstNearOutput=0,fixedArmContactAt=0;
    int fixedAuthKicker=0,fixedAuthReceiver=0;
    float anchorError=0.f,headingError=0.f;unsigned nativeQueries=0;
    WhiteFleetC15ReceiverRoute::State anchor;
    Point previousOdo{},previousField{};float previousOdoHeading=0.f,previousHeading=0.f,headingField=0.f,headingOdoYaw=0.f;
    std::array<Point,3> pointsOdo{};Point ballOdo{},ownBallOdo{};unsigned segment=0;
    const char* reason="initial";const char* lastLogReason="initial";
    void begin(unsigned e){if(event!=e){*this=State();event=e;}}
  };

  enum class Kind{wait,walk,shoot};
  struct Command{Kind kind=Kind::wait;Point target{};float heading=0.f;Plan plan{};const char* reason="wait";};

  inline bool fixedFreshPosition(const Input& i)
  {
    return i.fixedOwnFresh&&i.seen>0&&i.seen<=i.now&&i.now-i.seen<=250u&&finite(i.ball)&&finite(i.relativeBall)
        &&distance(i.ball,add(i.robot,rotate(i.relativeBall,i.heading)))<=1.f;
  }
  inline bool fixedFreshEvidence(const Input& i){return fixedFreshPosition(i)&&finite(i.velocity);}
  inline bool fixedCurrentWitness(unsigned now,unsigned event,unsigned frame,unsigned release,int kicker,int receiver)
  {return event>0&&event<=now&&frame==now&&release>event&&release<=now&&kicker>0&&receiver>0&&kicker!=receiver;}
  inline bool fixedArrivalContext(const State& s,unsigned now,unsigned event,unsigned frame,unsigned release,
                                 int kicker,int receiver,Point contract,Point reference)
  {
    return s.fixedArrivalPhase!=FixedArrivalPhase::unarmed&&!s.fixedArrivalRevoked&&s.event==event
        &&fixedCurrentWitness(now,event,frame,release,kicker,receiver)&&s.fixedSessionRelease==release
        &&s.fixedAuthKicker==kicker&&s.fixedAuthReceiver==receiver&&finite(contract)&&finite(reference)
        &&distance(contract,s.fixedReferenceContract)<=300.f&&distance(reference,s.fixedReferenceBall)<=1.f;
  }

  /** Fixed FIELD navigation reference only. No forecast, route selection or
   * kick certificate. Native detours are rejected, never stripped/disabled. */
  template<class Native,class Legal>
  Command fixedDirectUpdate(State& s,const Input& i,Native native,Legal legal)
  {
    Command out;out.plan.fixedDirect=true;
    const bool releasedRun=i.fixedRunToArrival&&s.fixedArrivalPhase==FixedArrivalPhase::run;
    const bool fresh=i.fixedRunToArrival?fixedFreshPosition(i):i.now-i.seen<=250u;
    const auto stop=[&](const char* why){out.kind=Kind::wait;out.target={};out.reason=s.reason=why;return out;};
    if(!i.localStep||i.shortest||i.lowerChannel)return stop("fixed_profile_conflict");
    const Point body=releasedRun?s.fixedReferenceBody:Point{i.fixedBall.x-650.f,i.fixedBall.y};
    if(!WhiteFleetKickoffThroughV8::paramsValid(i.params)||!WhiteFleetKickoffThroughV8::fieldValid(i.field)
       ||!inside(i.fixedBall,i)||!inside(body,i)||!legal(body)||!finite(i.contract)
       ||distance(i.fixedBall,i.contract)>1500.f)return stop("fixed_reference_invalid");
    if(s.fixedReferenceKnown&&(distance(i.fixedBall,s.fixedReferenceBall)>1.f
       ||distance(i.contract,s.fixedReferenceContract)>300.f))return stop("fixed_reference_or_contract_changed");
    if((!s.fixedReferenceKnown||!s.anchor.valid)&&!fresh)return stop("fixed_fresh_anchor_required");
    if(!releasedRun&&!fresh&&i.seen!=s.lastSeen)return stop("fixed_gap_observation_mismatch");
    const Point ray=sub(body,i.robot);const float total=distance(ray,{});
    // Existing WalkToPoint100mm navigation tolerance/frontier, not a ball
    // capture rule. Never turn180 back to a passed reference or kick merely
    // because this fixed body point was reached.
    if(!std::isfinite(total)||total<=100.f||(i.robot.x>=body.x&&std::abs(i.robot.y-body.y)<=100.f))
      return stop("fixed_body_target_reached");
    const Point direction=mul(ray,1.f/total);const float heading=std::atan2(ray.y,ray.x);
    const float remaining=std::max(0.f,float(std::min(i.params.flight,i.params.total))-float(i.now-i.event))*.001f;
    const float cap=std::min(i.maxForward*i.params.receiverSpeedFactor,i.params.empiricalReceiverSpeed),yaw=std::min(i.maxYaw,1.5f);
    if(!std::isfinite(cap)||cap<=0.f||!std::isfinite(yaw)||yaw<=0.f)return stop("fixed_invalid_limits");
    const float turn=std::abs(angle(heading-i.heading))/yaw;
    const float startup=i.params.startDelay*(1.f-std::clamp((i.estimatedSpeedKnown?i.estimatedSpeed:0.f)/cap,0.f,1.f));
    float length=std::min({total,1200.f,cap*std::max(0.f,remaining-i.params.arrivalMargin-turn-startup)});
    if(length<=1.f)return stop("fixed_segment_budget");
    Point local=rotate(mul(direction,length),-i.heading);
    if(fresh)
    {
      // Fresh near-ball terminal permission is still handled by the original
      // native handoff above this function. Pure walking never sweeps its feet
      // into a different/unqualified nearby ball merely because the reference
      // is farther away.
      const float margin=distance(i.relativeBall,{})-650.f;
      if(margin<=1.f)return stop("fixed_fresh_ball_margin");
      if(margin<length){local=mul(local,margin/length);length=margin;}
    }
    else if(!releasedRun)
    {
      const Point oldBall=rotate(sub(s.ownBallOdo,i.odometry),-i.odoHeading);
      local=boundedGapStep(local,oldBall,i.relativeBall);length=distance(local,{});
      if(length<=1.f)return stop("fixed_gap_ball_margin");
    }
    const Point target=add(i.robot,rotate(local,i.heading));
    for(unsigned k=0;k<=12;++k)
    {
      const Point p=add(i.robot,mul(sub(target,i.robot),k/12.f));
      if(!inside(p,i)||!legal(p))return stop("fixed_illegal_segment");
    }
    // Actual local footprints and original near-time250ms crossing protection.
    // No global centre/soft-uncertainty/far-time obstacle vote can stop the run.
    out.plan.clearance=1e9f;
    for(const auto& o:i.obstacles)if(o.local)
    {
      const float clear=WhiteFleetKickoffThroughV8::pointSegmentDistance(o.center,i.robot,target)-o.radius;
      out.plan.clearance=std::min(out.plan.clearance,clear);
      if(clear< -1.f)return stop("fixed_current_line_occupied");
      if(o.velocityKnown)for(unsigned k=0;k<=5;++k)
      {
        const float t=k*.05f;
        const Point robot=add(i.robot,mul(direction,std::min(length,std::hypot(i.maxForward,i.maxSide)*t)));
        const Point obstacle=add(o.center,mul(o.velocity,t));
        if(distance(robot,obstacle)<o.radius-1.f)return stop("fixed_near_dynamic_crossing");
      }
    }
    const auto route=native(target);out.plan.routesTested=1;
    if(route.redirected)return stop("fixed_native_requires_detour");
    if(!route.valid||!std::isfinite(route.length)||route.samples.size()<2
       ||distance(route.samples.front(),i.robot)>1.f||distance(route.samples.back(),target)>1.f)
      return stop("fixed_native_no_direct_route");
    float measured=0.f;
    for(size_t k=0;k<route.samples.size();++k)
    {
      const Point p=route.samples[k];if(!finite(p)||!inside(p,i)||!legal(p))return stop("fixed_native_illegal_path");
      const Point d=sub(p,i.robot);const float alongLine=d.x*direction.x+d.y*direction.y;
      const float lateral=std::abs(d.x*direction.y-d.y*direction.x);
      if(lateral>1.f||alongLine< -1.f||alongLine>length+1.f)return stop("fixed_native_requires_detour");
      if(k)measured+=distance(p,route.samples[k-1]);
    }
    if(std::abs(route.length-length)>1.f||std::abs(measured-length)>1.f)return stop("fixed_native_requires_detour");
    if(!s.fixedReferenceKnown)
    {
      s.fixedReferenceKnown=true;s.fixedReferenceBall=i.fixedBall;s.fixedReferenceBody=body;
      s.fixedReferenceContract=i.contract;s.fixedOrigin=i.robot;s.fixedInitialDirection=heading;
    }
    if(!s.anchor.valid)
    {
      if(s.awaitingEpochCommit){++s.generation;s.awaitingEpochCommit=false;}
      s.anchor=WhiteFleetC15ReceiverRoute::State();s.anchor.valid=true;s.anchor.originField=i.robot;s.anchor.originOdometry=i.odometry;
      s.anchor.originHeading=i.heading;s.anchor.originOdoHeading=i.odoHeading;s.headingField=i.heading;s.headingOdoYaw=i.odoHeading;
    }
    s.anchor.lastOdometry=i.odometry;s.anchor.lastOdoHeading=i.odoHeading;s.anchor.lastFrame=i.now;
    out.kind=Kind::walk;out.target=target;out.heading=heading;out.reason=s.reason="fixed_direct_walk";
    out.plan.valid=true;out.plan.size=1;out.plan.points[0]=target;out.plan.ball=i.fixedBall;
    out.plan.length=length;out.plan.firstLegEta=length/cap+turn+startup;out.plan.localStepOnly=true;
    s.plan=out.plan;return out;
  }

  template<class Native,class Legal>
  Command fixedArrivalUpdate(State& s,const Input& i,Native native,Legal legal)
  {
    Command out;const auto stop=[&](const char* why){out.kind=Kind::wait;out.reason=s.reason=why;return out;};
    if(s.fixedArrivalRevoked||!i.fixedDirect||!i.localStep||i.shortest||i.lowerChannel
       ||!fixedCurrentWitness(i.now,i.event,i.fixedContextFrame,i.fixedContextRelease,i.fixedContextKicker,i.fixedContextReceiver))
      return stop("fixed_arrival_context_unqualified");
    if(s.fixedArrivalPhase==FixedArrivalPhase::unarmed)
    {
      const Point body{i.fixedBall.x-650.f,i.fixedBall.y};
      if(!fixedFreshEvidence(i)||!i.fixedReleaseWitness||!inside(i.ball,i)||!legal(i.ball))return stop("fixed_arrival_wait_real_release");
      if(!inside(i.fixedBall,i)||!inside(body,i)||!legal(body)||!finite(i.contract)||distance(i.fixedBall,i.contract)>1500.f)
        return stop("fixed_reference_invalid");
      s.fixedReferenceKnown=true;s.fixedReferenceBall=i.fixedBall;s.fixedReferenceBody=body;s.fixedReferenceContract=i.contract;
      s.fixedOrigin=i.robot;s.fixedInitialDirection=std::atan2(body.y-i.robot.y,body.x-i.robot.x);
      s.fixedArrivalPhase=FixedArrivalPhase::run;s.fixedAuthorizedAt=i.now;s.fixedAuthSeen=i.seen;
      s.fixedSessionRelease=i.fixedContextRelease;s.fixedAuthKicker=i.fixedContextKicker;s.fixedAuthReceiver=i.fixedContextReceiver;
      // Authenticate the current native pose ONCE even if the very first
      // movement is blocked. A cleared obstacle never needs a new ball lease.
      if(!s.anchor.valid)
      {
        s.anchor=WhiteFleetC15ReceiverRoute::State();s.anchor.valid=true;s.anchor.originField=i.robot;s.anchor.originOdometry=i.odometry;
        s.anchor.originHeading=i.heading;s.anchor.originOdoHeading=i.odoHeading;
      }
      if(s.awaitingEpochCommit){++s.generation;s.awaitingEpochCommit=false;}
      s.headingField=i.heading;s.headingOdoYaw=i.odoHeading;
      s.anchor.lastOdometry=i.odometry;s.anchor.lastOdoHeading=i.odoHeading;s.anchor.lastFrame=i.now;
    }
    if(!fixedArrivalContext(s,i.now,i.event,i.fixedContextFrame,i.fixedContextRelease,i.fixedContextKicker,i.fixedContextReceiver,i.contract,i.fixedBall))
    {s.fixedArrivalRevoked=true;return stop("fixed_arrival_context_changed");}
    // A new native FIELD/ODO epoch has ALREADY passed all original evidence
    // gates in update(). Commit that pose independently of movement admission:
    // an occupied first segment must not make a cleared RUN wait for ball again.
    if(i.boundedFastRun&&s.fixedArrivalPhase==FixedArrivalPhase::run&&!s.anchor.valid
       &&s.awaitingEpochCommit&&!s.epochPending&&fixedFreshEvidence(i))
    {
      s.anchor=WhiteFleetC15ReceiverRoute::State();s.anchor.valid=true;s.anchor.originField=i.robot;s.anchor.originOdometry=i.odometry;
      s.anchor.originHeading=i.heading;s.anchor.originOdoHeading=i.odoHeading;
      s.headingField=i.heading;s.headingOdoYaw=i.odoHeading;
      s.anchor.lastOdometry=i.odometry;s.anchor.lastOdoHeading=i.odoHeading;s.anchor.lastFrame=i.now;
      ++s.generation;s.awaitingEpochCommit=false;
    }
    const Point body=s.fixedReferenceBody;
    if(i.now<i.event||i.now-i.event>=std::min(i.params.flight,i.params.total))return stop("fixed_arrival_deadline");
    if(s.fixedArrivalPhase==FixedArrivalPhase::run)
    {
      if(!inside(body,i)||!legal(body))return stop("fixed_reference_illegal");
      if(distance(i.robot,body)<=100.f||(i.robot.x>=body.x&&std::abs(i.robot.y-body.y)<=100.f))
      {s.fixedArrivalPhase=FixedArrivalPhase::arrived;s.fixedArrivedAt=i.now;s.near=false;}
      else return fixedDirectUpdate(s,i,native,legal);
    }
    // ARRIVED is one-way: no old fixed run or remote chase can resume.
    if(!fixedFreshEvidence(i)||i.seen<=s.fixedArrivedAt)return stop("fixed_arrived_search_new_own_ball");
    if(!inside(i.ball,i)||!legal(i.ball))return stop("fixed_arrived_ball_illegal");
    if(!s.fixedFreshAfterArrival)s.fixedFreshAfterArrival=i.seen;
    if(distance(i.ball,i.contract)>2150.f)return stop("fixed_arrived_ball_outside_near_context");
    const Point ray={i.field.opponentGoal-i.ball.x,-i.ball.y};const float length=distance(ray,{});
    if(handoff(i))
    {
      const Point foot=length>1.f?sub(i.ball,mul(ray,650.f/length)):i.robot;
      if(distance(i.relativeBall,{})<=650.f||(legal(foot)&&native(foot).valid))
      {out.kind=Kind::shoot;out.reason=s.reason="fixed_arrived_fresh_native";s.near=true;return out;}
    }
    if(distance(i.relativeBall,{})<=1000.f&&std::abs(std::atan2(i.relativeBall.y,i.relativeBall.x))<=.785398164f&&length>1.f)
    {
      Plan approach;approach.size=1;approach.points[0]=sub(i.ball,mul(ray,650.f/length));approach.ball=i.ball;
      if(inside(approach.points[0],i)&&legal(approach.points[0])&&distance(approach.points[0],i.robot)>1.f)
      {
        approach=evaluate(approach,i,native,legal);
        if(approach.valid){out.kind=Kind::walk;out.target=approach.points[0];out.heading=std::atan2(ray.y,ray.x);out.plan=approach;
          out.reason=s.reason="fixed_arrived_fresh_guarded_approach";return out;}
      }
    }
    return stop("fixed_arrived_near_geometry_wait");
  }

  template<class Predict,class Native,class Legal>
  Command shortestUpdate(State& s,Input i,Predict predict,Native native,Legal legal);
}
#include "WhiteFleetC15ShortestIntercept.h"
namespace WhiteFleetC15EarlyChannel
{

  template<class Predict,class Native,class Legal>
  Command update(State& s,Input i,Predict predict,Native native,Legal legal)
  {
    s.begin(i.event);s.nativeQueries=0;Command out;
    out.plan.uncertain=!i.estimatedSpeedKnown||i.unknownOpponents>0;
    for(const auto& o:i.obstacles)out.plan.uncertain|=!o.velocityKnown;
    const auto wait=[&](const char* why){s.reason=why;out.reason=why;return out;};
    if(i.fixedDirect&&(!i.localStep||i.shortest||i.lowerChannel))return wait("fixed_profile_conflict");
    if(s.lastFrame&&i.now<s.lastFrame)s.clockFault=true;
    if(s.clockFault)return wait("clock_regression");
    const bool arrivalWitness=i.fixedRunToArrival&&i.fixedDirect&&fixedCurrentWitness(i.now,i.event,i.fixedContextFrame,i.fixedContextRelease,i.fixedContextKicker,i.fixedContextReceiver);
    const bool releasedPoint=arrivalWitness&&fixedArrivalContext(s,i.now,i.event,i.fixedContextFrame,i.fixedContextRelease,i.fixedContextKicker,i.fixedContextReceiver,i.contract,i.fixedBall);
    if(i.fixedRunToArrival&&(!arrivalWitness||s.fixedArrivalRevoked))return wait("fixed_arrival_context_unqualified");
    if(i.fixedRunToArrival&&s.fixedArrivalPhase!=FixedArrivalPhase::unarmed&&!releasedPoint)
    {s.fixedArrivalRevoked=true;return wait("fixed_arrival_context_changed");}
    const bool badBall=!i.seen||i.seen>i.now||!finite(i.ball)||!finite(i.relativeBall)||!finite(i.velocity)
        ||distance(i.ball,add(i.robot,rotate(i.relativeBall,i.heading)))>1.f;
    if(!i.event||i.now<i.event||i.seen>i.now||(!releasedPoint&&badBall)
       ||!finite(i.robot)||!finite(i.odometry)||!std::isfinite(i.heading)||!std::isfinite(i.odoHeading)
       ||!std::isfinite(i.maxForward)||i.maxForward<=0.f||!std::isfinite(i.maxSide)||i.maxSide<=0.f
       ||!std::isfinite(i.maxYaw)||i.maxYaw<=0.f||!inside(i.robot,i)||!legal(i.robot))
    {s.plan.valid=false;s.anchor.invalidate();return wait("invalid_input");}
    for(const auto& o:i.obstacles)
      if(!finite(o.center)||!std::isfinite(o.radius)||o.radius<=0.f||!std::isfinite(o.uncertainty)||o.uncertainty<0.f
         ||!std::isfinite(o.speedBound)||o.speedBound<=0.f||(o.velocityKnown&&!finite(o.velocity)))return wait("invalid_obstacle");
    const bool fresh=i.fixedRunToArrival?fixedFreshEvidence(i):i.now-i.seen<=250u;
    const bool first=!s.lastFrame;
    const float ownStep=first?0.f:distance(i.odometry,s.previousOdo);
    const float dt=i.now>=s.lastFrame?(i.now-s.lastFrame)*.001f:-1.f;
    const Point anchoredStep=add(s.previousField,rotate(sub(i.odometry,s.previousOdo),s.previousHeading-s.previousOdoHeading));
    const bool odometryStep=first||(dt>=0.f&&distance(i.odometry,s.previousOdo)<=
        std::hypot(i.maxForward,i.maxSide)*dt+100.f&&std::abs(angle(i.odoHeading-s.previousOdoHeading))<=i.maxYaw*dt+.1f);
    const bool fieldStep=first||(distance(anchoredStep,i.robot)<=300.f&&
        std::abs(angle(s.previousHeading+i.odoHeading-s.previousOdoHeading-i.heading))<=.1745329252f);
    s.previousOdo=i.odometry;s.previousField=i.robot;s.previousHeading=i.heading;
    s.previousOdoHeading=i.odoHeading;s.lastFrame=i.now;
    if(first){s.headingField=i.heading;s.headingOdoYaw=i.odoHeading;}
    // Never follow an old route across the original increment bounds. New
    // native pose evidence is evaluated separately, not by waiting a cooldown.
    if(!odometryStep)s.odometryFault=true;
    if(s.odometryFault)
    {s.plan.valid=false;s.anchor.invalidate();s.memory=Memory();return wait("odometry_epoch_fault");}
    s.headingError=std::abs(angle(s.headingField+i.odoHeading-s.headingOdoYaw-i.heading));
    if(!fieldStep)
    {s.plan.valid=false;s.anchor.invalidate();s.memory=Memory();s.epochPending=true;return wait("field_or_heading_step");}
    if(s.headingError>.785398164f)
    {s.plan.valid=false;s.anchor.invalidate();s.memory=Memory();s.epochPending=true;return wait("heading_epoch_untrusted");}
    if(s.headingError>.1745329252f)s.epochPending=true;
    if(!releasedPoint&&i.now-i.seen>450u)return wait("ball_lease450");

    if(s.epochPending||(s.anchor.valid&&!s.anchor.coherent(i.now,i.odometry,i.odoHeading,i.robot,i.heading,
                                         std::hypot(i.maxForward,i.maxSide),i.maxYaw)))
    {
      const Point oldPosition=add(s.anchor.originField,rotate(sub(i.odometry,s.anchor.originOdometry),
          s.anchor.originHeading-s.anchor.originOdoHeading));
      const float correction=distance(oldPosition,i.robot);
      s.anchorError=correction;
      s.plan.valid=false;s.anchor.invalidate();s.memory=Memory();
      s.epochPending=true;
      const bool forwardMotion=distance(i.velocity,{})>100.f&&i.velocity.x>0.f&&i.ball.x>0.f;
      const bool frontReceive=distance(i.velocity,{})<=100.f&&distance(i.ball,i.contract)<=2150.f&&i.ball.x>0.f
          &&std::abs(std::atan2(i.relativeBall.y,i.relativeBall.x))<=.785398164f
          &&std::abs(angle(std::atan2(-i.ball.y,i.field.opponentGoal-i.ball.x)-i.heading))<=.785398164f;
      // Discard the OLD route at300/10. A different route may be built NOW
      // only from qualified fresh native pose and forward-ball evidence.
      // The sender's tighter2-degree aim criterion is not a receiver gate.
      // Native pose quality was already admitted by the unchanged caller.
      // Large finite covariance is reported as uncertainty, not a newly added
      // stop line. The old route is gone; only a NEW current-input route may run.
      if(!fresh||!std::isfinite(i.positionSigma)||!std::isfinite(i.headingSigma)||correction>1500.f
         ||(!forwardMotion&&!frontReceive))
        return wait("new_pose_evidence_required");
      s.epochPending=false;s.awaitingEpochCommit=true;s.reason="new_native_pose_epoch";
    }
    unsigned obstacleSeen=s.lastObstacleSeen;
    for(auto& o:i.obstacles)
    {
      s.memory.update(o,i.now);
      if(o.local&&o.seen<=i.now&&i.now-o.seen<=250u)obstacleSeen=std::max(obstacleSeen,o.seen);
    }
    const bool newSight=i.seen>s.lastSeen;
    const bool newObstacle=obstacleSeen>s.lastObstacleSeen;
    s.lastObstacleSeen=obstacleSeen;
    if(fresh&&newSight){s.lastSeen=i.seen;s.ownBallOdo=toOdo(i.ball,i);}
    if(i.fixedRunToArrival)
    {
      unsigned calls=0;const auto currentNative=[&](Point p){if(calls++>=7)return NativePath{};++s.nativeQueries;return native(p);};
      return fixedArrivalUpdate(s,i,currentNative,legal);
    }
    const bool receivingRegion=distance(i.ball,i.contract)<=2150.f;
    if(i.lowerChannel&&s.lowerCommitted&&!s.lowerPassed)
    {
      Point center=toField(s.lowerDefenderOdo,i);
      // Refresh ONLY the uniquely associated fresh local defender. Retained
      // model ticks do not renew lastSeen; absent/ambiguous evidence keeps the
      // original ODO-anchored intention, never invents a new defender position.
      const Obstacle* match=nullptr;float d1=1e9f,d2=1e9f;
      for(const auto& o:i.obstacles)if(o.local&&o.opponent&&o.identifiedOpponent&&o.seen&&o.seen<=i.now&&i.now-o.seen<=250u)
      {const float d=distance(o.center,center);if(d<d1){d2=d1;d1=d;match=&o;}else d2=std::min(d2,d);}
      if(match&&d1<=2200.f*(i.now-s.lowerSeen)*.001f+300.f&&d2-d1>300.f&&match->seen>s.lowerSeen)
      {center=match->center;s.lowerDefenderOdo=toOdo(center,i);s.lowerSeen=match->seen;s.lowerRadius=match->radius;s.lowerUncertainty=match->uncertainty;}
      const Point exit=toField(s.lowerExitOdo,i);
      if(i.robot.x>=std::max(exit.x,center.x+s.lowerRadius+i.params.obstaclePadding)
         &&i.robot.y<=center.y-s.lowerRadius&&distance(i.robot,center)>=s.lowerRadius+i.params.obstaclePadding)
      {s.lowerPassed=true;s.lowerCutAt=i.now;s.lowerReason="estimated_shoulder_passed";}
      i.lowerDefender={center,{},s.lowerRadius,s.lowerUncertainty,2200.f,s.lowerSeen,false,true,true,true};
      i.lowerDefenderKnown=true;
    }
    i.lowerLocked=i.lowerChannel&&s.lowerCommitted&&!s.lowerPassed;i.lowerPassed=s.lowerPassed;
    unsigned nativeCalls=0;
    s.nativeQueries=0;
    const auto budgeted=[&](Point point){if(nativeCalls++>=7)return NativePath{};++s.nativeQueries;return native(point);};
    if(!i.lowerLocked&&fresh&&receivingRegion&&handoff(i))
    {
      const Point ray={i.field.opponentGoal-i.ball.x,-i.ball.y};const float length=distance(ray,{});
      const Point foot=length>1.f?sub(i.ball,mul(ray,650.f/length)):i.robot;
      if(distance(i.relativeBall,{})<=650.f||(legal(foot)&&budgeted(foot).valid))s.near=true;
    }
    if(s.near)
    {
      if(!fresh)return wait("near_ball_stale");
      if(s.awaitingEpochCommit){++s.generation;s.awaitingEpochCommit=false;s.headingField=i.heading;s.headingOdoYaw=i.odoHeading;}
      out.kind=Kind::shoot;out.reason=s.reason="fresh_front_receive";return out;
    }
    if(!i.lowerLocked&&fresh&&receivingRegion&&distance(i.relativeBall,{})<=1000.f
       &&std::abs(std::atan2(i.relativeBall.y,i.relativeBall.x))<=.785398164f)
    {
      // Keep RUNNING under this controller when pressure makes the earlier
      // native handoff inappropriate. Approach the fresh foot-side pose with
      // native routing; this grants no kick and is not an old chase fallback.
      const Point ray={i.field.opponentGoal-i.ball.x,-i.ball.y};const float n=distance(ray,{});
      if(n>1.f)
      {
        const Point foot=sub(i.ball,mul(ray,650.f/n));
        Plan approach;approach.size=1;approach.points[0]=foot;approach.ball=i.ball;
        if(inside(foot,i)&&legal(foot)&&distance(foot,i.robot)>1.f)
        {
          approach=evaluate(approach,i,budgeted,legal);
          if(approach.valid)
          {
            out.kind=Kind::walk;out.target=foot;out.heading=std::atan2(ray.y,ray.x);out.plan=approach;
            out.reason=s.reason="fresh_guarded_approach";return out;
          }
        }
      }
    }

    if(i.fixedDirect)return fixedDirectUpdate(s,i,budgeted,legal);
    if(i.shortest)return shortestUpdate(s,i,predict,budgeted,legal);

    Plan current;current.valid=false;
    if(s.plan.valid&&s.anchor.valid)
    {
      while(s.segment+1<s.plan.size&&distance(i.robot,toField(s.pointsOdo[s.segment],i))<=650.f
            &&!(i.lowerLocked&&s.segment==1))++s.segment;
      current=s.plan;current.size=0;current.ball=toField(s.ballOdo,i);
      if(current.lower&&i.lowerLocked){current.defender=i.lowerDefender.center;current.defenderRadius=i.lowerDefender.radius;}
      // Only after the body estimate passed the committed shoulder may the
      // original receiving point become the next leg. Proximity650 alone is
      // not permission to cut across the defender's front.
      if(current.lower&&s.lowerPassed){s.segment=2;current.lower=false;}
      for(unsigned n=s.segment;n<s.plan.size;++n)current.points[current.size++]=toField(s.pointsOdo[n],i);
      current.ballTime=std::max(0.f,s.plan.ballTime-(i.now-s.committedAt)*.001f);
      if(distance(current.ball,i.contract)<=1800.f)current=evaluate(current,i,budgeted,legal);
      else {current.valid=false;current.reason="old_region_changed";}
    }
    // A newly observed blocker may change the corridor while our ball sight
    // remains within its original fresh lease. This is NOT a new ball sight:
    // lastSeen is unchanged and no motion/foot deadline is renewed.
    const bool replan=fresh&&(newSight||newObstacle||ownStep>1.f||s.awaitingEpochCommit)
        &&(!current.valid||!s.plannedAt||i.now<s.plannedAt||i.now-s.plannedAt>=250u);
    if(replan)
    {
      s.plannedAt=i.now;
      Plan next=choose(i,predict,budgeted,legal);
      // Never wait out a cooldown on a blocked route. A valid current corridor
      // keeps its side unless the new route has a meaningful time/risk gain.
      const bool roomGain=next.softClearance>=0.f&&current.softClearance<0.f
          &&next.softClearance-current.softClearance>=i.params.obstaclePadding;
      const bool replace=next.valid&&(!current.valid||(next.lower&&!s.lowerCommitted)
          ||(i.lowerLocked?(next.lower&&next.lane!=current.lane&&(roomGain||next.score+i.params.arrivalMargin<current.score)):
             (next.side==current.side||roomGain||next.score+i.params.arrivalMargin<current.score)));
      if(replace)
      {
        s.plan=next;s.segment=0;s.committedAt=i.now;s.ballOdo=toOdo(next.ball,i);
        if(next.lower)
        {
          s.lowerReason=!s.lowerCommitted?"commit_red_field_lower":!current.valid?"same_side_route_blocked":"same_side_more_room";
          if(s.lowerCommitted)++s.lowerChanges;
          s.lowerCommitted=true;s.lowerSeen=next.defenderSeen;s.lowerRadius=next.defenderRadius;s.lowerUncertainty=next.defenderUncertainty;s.lowerLane=next.lane;
          s.lowerDefenderOdo=toOdo(next.defender,i);s.lowerEntryOdo=toOdo(next.entry,i);s.lowerExitOdo=toOdo(next.exit,i);
        }
        if(s.awaitingEpochCommit){++s.generation;s.awaitingEpochCommit=false;}
        s.headingField=i.heading;s.headingOdoYaw=i.odoHeading;
        for(unsigned n=0;n<next.size;++n)s.pointsOdo[n]=toOdo(next.points[n],i);
        s.anchor=WhiteFleetC15ReceiverRoute::State();s.anchor.valid=true;
        s.anchor.originField=i.robot;s.anchor.originOdometry=i.odometry;
        s.anchor.originHeading=i.heading;s.anchor.originOdoHeading=i.odoHeading;
        s.anchor.lastOdometry=i.odometry;s.anchor.lastOdoHeading=i.odoHeading;s.anchor.lastFrame=i.now;
        current=next;
      }
      else if(!current.valid){s.plan.valid=false;out.plan=next;return wait(next.reason);}
    }
    out.plan=current;
    if(!current.valid||!s.plan.valid||!s.anchor.valid)return wait(current.reason);
    Point local=rotate(sub(s.pointsOdo[s.segment],i.odometry),-i.odoHeading);
    if(!fresh)
    {
      if(i.seen!=s.lastSeen)return wait("gap_observation_mismatch");
      const Point oldBall=rotate(sub(s.ownBallOdo,i.odometry),-i.odoHeading);
      local=boundedGapStep(local,oldBall,i.relativeBall);
    }
    out.target=add(i.robot,rotate(local,i.heading));
    if(distance(local,{})<=1.f||!inside(out.target,i)||!legal(out.target)||!legal(i.robot))return wait("unsafe_local_step");
    // Face the next shot while the legs run the selected channel. Do not chase
    // the current ball bearing with body yaw on every vision frame.
    out.heading=std::atan2(-i.ball.y,i.field.opponentGoal-i.ball.x);
    s.anchor.lastOdometry=i.odometry;s.anchor.lastOdoHeading=i.odoHeading;s.anchor.lastFrame=i.now;
    out.kind=Kind::walk;out.plan=current;
    out.reason=s.reason=i.lowerChannel&&s.lowerCommitted&&!s.lowerPassed?"lower_channel_walk":s.lowerPassed&&i.lowerChannel?"lower_cut_to_receive":
        current.localStepOnly?"channel_local_step":"channel_walk";return out;
  }
}
