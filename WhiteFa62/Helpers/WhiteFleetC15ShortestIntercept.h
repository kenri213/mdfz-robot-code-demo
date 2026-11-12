#pragma once
#include "WhiteFleetC15EarlyChannel.h"

/** Explicit receiver-only experiment. Forecast/ETA are NOT capture permits.
 * Actual native tangent/arc routes, original limits and original clock only. */
namespace WhiteFleetC15EarlyChannel
{
  inline float shortestRemaining(const Input& i)
  {return i.now>=i.event?std::max(0.f,float(std::min(i.params.flight,i.params.total))-float(i.now-i.event))*.001f:0.f;}
  inline bool preferredShape(const Input& i)
  {return std::isfinite(i.preferredMinX)&&std::isfinite(i.preferredMaxX)&&std::isfinite(i.preferredMinY)
      &&std::isfinite(i.preferredMaxY)&&i.preferredMinX<i.preferredMaxX&&i.preferredMinY<i.preferredMaxY;}
  inline bool preferredUsable(const Input& i)
  {
    if(!preferredShape(i))return false;
    for(float x:{i.preferredMinX,i.preferredMaxX})for(float y:{i.preferredMinY,i.preferredMaxY})
      if(!inside({x,y},i)||distance({x,y},i.contract)>1500.f)return false;
    return true;
  }
  inline bool preferredBall(Point p,const Input& i)
  {return preferredUsable(i)&&p.x>=i.preferredMinX&&p.x<=i.preferredMaxX&&p.y>=i.preferredMinY&&p.y<=i.preferredMaxY;}

  template<class Legal>
  Plan shortestInspect(Plan p,const Input& i,const NativePath& route,Legal legal)
  {
    p.valid=false;p.shortest=true;p.nativeRejected=p.occupiedRejected=p.illegalRejected=p.budgetRejected=0;
    const auto reject=[&](const char* why){p.reason=why;
      if(std::strcmp(why,"native_no_route")==0)++p.nativeRejected;
      if(std::strcmp(why,"occupied_first_step")==0)++p.occupiedRejected;
      if(std::strcmp(why,"illegal_native_path")==0)++p.illegalRejected;
      if(std::strcmp(why,"first_step_budget")==0)++p.budgetRejected;return p;};
    if(!WhiteFleetKickoffThroughV8::paramsValid(i.params)||!WhiteFleetKickoffThroughV8::fieldValid(i.field)
       ||!p.size||p.size>1||!inside(p.points[0],i)||!legal(p.points[0]))return reject("illegal_native_path");
    if(!route.valid||!std::isfinite(route.length)||route.length<=1.f||route.samples.size()<2
       ||!std::isfinite(route.arcTurn)||!std::isfinite(route.initialTurn)||distance(route.samples.front(),i.robot)>1.f
       ||distance(route.samples.back(),p.points[0])>1.f)return reject("native_no_route");
    float polyline=0.f;
    for(size_t k=1;k<route.samples.size();++k)
    {if(!finite(route.samples[k]))return reject("native_no_route");polyline+=distance(route.samples[k],route.samples[k-1]);}
    if(!std::isfinite(polyline)||std::abs(polyline-route.length)>std::max(1.f,route.length*.001f))return reject("native_no_route");
    // Bounded full native curve legality, never a Euclidean chord certificate.
    const unsigned checks=static_cast<unsigned>(std::min(257.f,std::ceil(route.length/250.f)));
    if(checks>256u)return reject("native_no_route");
    for(unsigned k=0;k<=checks;++k)
    {const Point point=along(route.samples,float(k)/checks);if(!inside(point,i)||!legal(point))return reject("illegal_native_path");}
    const float cap=std::min(i.maxForward*i.params.receiverSpeedFactor,i.params.empiricalReceiverSpeed);
    const float yaw=std::min(i.maxYaw,1.5f),remaining=shortestRemaining(i);
    if(!std::isfinite(cap)||cap<=0||!std::isfinite(yaw)||yaw<=0)return reject("invalid_motion_limits");
    p.tangent={};for(size_t k=1;k<route.samples.size();++k)
      if(distance(route.samples[k],i.robot)>1.f){p.tangent=sub(route.samples[k],i.robot);break;}
    if(distance(p.tangent,{})<=1.f)return reject("native_no_route");
    const float initial=std::abs(angle(std::atan2(p.tangent.y,p.tangent.x)-i.heading));
    const float startup=i.params.startDelay*(1.f-std::clamp((i.estimatedSpeedKnown?i.estimatedSpeed:0.f)/cap,0.f,1.f));
    const float firstLength=std::min(1200.f,route.length);
    float prefixTurn=0.f,covered=0.f,previous=std::atan2(p.tangent.y,p.tangent.x);
    for(size_t k=1;k<route.samples.size()&&covered<firstLength;++k)
    {
      const Point step=sub(route.samples[k],route.samples[k-1]);const float d=distance(step,{});if(d<=.001f)continue;
      const float direction=std::atan2(step.y,step.x);prefixTurn+=std::abs(angle(direction-previous));previous=direction;covered+=d;
    }
    p.shortestLength=route.length;p.shortestTurn=initial;
    p.shortestEta=route.length/cap+(initial+route.arcTurn)/yaw+startup;
    p.firstLegEta=firstLength/cap+(initial+prefixTurn)/yaw+startup;
    if(!std::isfinite(p.shortestEta)||!std::isfinite(p.firstLegEta)||p.firstLegEta+i.params.arrivalMargin>remaining)
      return reject("first_step_budget");
    p.clearance=p.softClearance=1e9f;p.uncertain=!i.estimatedSpeedKnown||i.unknownOpponents>0;
    for(unsigned k=1;k<=12;++k)
    {
      const float f=k/12.f,t=startup+f*(p.firstLegEta-startup);
      const Point body=along(route.samples,f*firstLength/route.length);
      for(const auto& o:i.obstacles)
      {
        const Point predicted=o.velocityKnown?add(o.center,mul(o.velocity,std::min(t,1.f))):o.center;
        const float clear=distance(body,predicted)-o.radius;
        p.softClearance=std::min(p.softClearance,clear-o.uncertainty-(!o.velocityKnown?o.speedBound*std::min(t,.2f):0.f));
        p.uncertain|=!o.velocityKnown;
        if(o.local){p.clearance=std::min(p.clearance,clear);if(clear< -1.f)return reject("occupied_first_step");}
      }
    }
    p.shortestPreferred=preferredBall(p.ball,i);
    p.shortestTimely=p.shortestEta+i.params.arrivalMargin<=remaining&&p.shortestEta<=p.ballTime+i.params.receiveWait;
    p.localStepOnly=!p.shortestTimely;p.length=p.shortestLength;p.eta=p.shortestEta;
    p.score=p.shortestLength;p.valid=true;p.reason="shortest_native_ready";return p;
  }

  inline bool shortestBetter(const Plan& a,const Plan& b,const Input& i)
  {
    if(!a.valid)return false;if(!b.valid)return true;
    const int ca=a.shortestTimely?(a.shortestPreferred?0:1):2,cb=b.shortestTimely?(b.shortestPreferred?0:1):2;
    if(ca!=cb)return ca<cb;
    const float sameLength=std::min(i.maxForward*i.params.receiverSpeedFactor,i.params.empiricalReceiverSpeed)*i.params.arrivalMargin;
    if(a.shortestLength+sameLength<b.shortestLength)return true;
    if(b.shortestLength+sameLength<a.shortestLength)return false;
    if(a.shortestEta+i.params.arrivalMargin<b.shortestEta)return true;
    if(b.shortestEta+i.params.arrivalMargin<a.shortestEta)return false;
    if(a.shortestTurn+.1745329252f<b.shortestTurn)return true;
    if(b.shortestTurn+.1745329252f<a.shortestTurn)return false;
    return a.shortestLength<b.shortestLength;
  }
  inline bool shortestReplace(const Plan& a,const Plan& b,const Input& i)
  {
    if(!a.valid)return false;if(!b.valid)return true;
    const int ca=a.shortestTimely?(a.shortestPreferred?0:1):2,cb=b.shortestTimely?(b.shortestPreferred?0:1):2;
    const float margin=std::min(i.maxForward*i.params.receiverSpeedFactor,i.params.empiricalReceiverSpeed)*i.params.arrivalMargin;
    return ca<cb||a.shortestLength+margin<b.shortestLength
        ||(a.shortestLength<=b.shortestLength+margin&&(a.shortestEta+i.params.arrivalMargin<b.shortestEta
            ||a.shortestTurn+.1745329252f<b.shortestTurn));
  }
  struct ShortestChoice{Plan plan{};NativePath path;};
  inline bool shortestGapRouteSafe(const NativePath& path,const Input& i,Point snapshot)
  {
    if(!path.valid||path.samples.size()<2||!finite(snapshot)||!std::isfinite(path.length)||path.length<=1.f)return false;
    // Preserve BOTH original1500 front margins on the actual curved current
    //1200mm prefix, not merely on the straight endpoint/chord.
    for(unsigned k=0;k<=12;++k)
    {
      const Point field=along(path.samples,(k/12.f)*std::min(1.f,1200.f/path.length));
      const Point local=rotate(sub(field,i.robot),-i.heading);
      if(distance(local,snapshot)<1500.f-.001f||distance(local,i.relativeBall)<1500.f-.001f)return false;
    }
    return true;
  }
  template<class Predict,class Native,class Legal>
  ShortestChoice shortestChoose(const Input& i,Predict predict,Native native,Legal legal,unsigned maximum=4)
  {
    std::vector<std::pair<Point,float>> preferred,other;
    unsigned samples=0,attempts=0,nativeBad=0,occupiedBad=0,illegalBad=0,budgetBad=0;
    for(unsigned k=0;k<=64;++k)
    {
      const float t=k*.125f;if(t+i.params.arrivalMargin>shortestRemaining(i))break;
      const Point ball=predict(t);++samples;
      if(!inside(ball,i)||distance(ball,i.contract)>1500.f||distance(ball,i.robot)<=1000.f)continue;
      auto& list=preferredBall(ball,i)?preferred:other;
      if(!list.empty()&&distance(ball,list.back().first)<300.f)
      {if(distance(ball,list.back().first)<=.001f)list.back().second=t;continue;}
      list.push_back({ball,t});
    }
    std::vector<std::pair<Point,float>> selected;
    const auto take=[&](const auto& list,unsigned count){if(list.empty()||!count)return;
      for(unsigned n=0;n<count;++n){const size_t at=count==1?0:(list.size()-1)*n/(count-1);
        if(std::none_of(selected.begin(),selected.end(),[&](auto p){return distance(p.first,list[at].first)<=.001f;}))selected.push_back(list[at]);}};
    if(preferred.empty())take(other,maximum);
    else if(other.empty())take(preferred,maximum);
    else {take(preferred,(maximum+1)/2);take(other,maximum/2);}
    ShortestChoice best;const char* last="no_receiving_sample";
    for(const auto& [ball,t]:selected)
    {
      const Point ray=sub(ball,i.robot);const float d=distance(ray,{});
      Plan p;p.shortest=p.shortestHasGoal=true;p.size=1;p.ball=ball;p.ballTime=t;p.points[0]=sub(ball,mul(ray,650.f/d));
      const auto path=native(p.points[0]);p=shortestInspect(p,i,path,legal);
      ++attempts;nativeBad+=p.nativeRejected;occupiedBad+=p.occupiedRejected;illegalBad+=p.illegalRejected;budgetBad+=p.budgetRejected;
      if(!p.valid)last=p.reason;if(shortestBetter(p,best.plan,i))best={p,path};
    }
    best.plan.samplesTested=samples;best.plan.routesTested=attempts;best.plan.nativeRejected=nativeBad;
    best.plan.occupiedRejected=occupiedBad;best.plan.illegalRejected=illegalBad;best.plan.budgetRejected=budgetBad;
    if(!best.plan.valid)best.plan.reason=last;return best;
  }

  template<class Predict,class Native,class Legal>
  Command shortestUpdate(State& s,Input i,Predict predict,Native native,Legal legal)
  {
    Command out;const bool fresh=i.now-i.seen<=250u;
    const auto stop=[&](const char* why){s.reason=s.shortestReason=why;out.reason=why;out.kind=Kind::wait;out.target={};return out;};
    if(!i.localStep||i.lowerChannel)return stop("invalid_shortest_profile");
    if(!fresh&&i.seen!=s.lastSeen)return stop("gap_observation_mismatch");
    s.shortestAreaAvailable=preferredUsable(i);
    Plan current;NativePath currentPath;
    if(s.plan.valid&&s.plan.shortest&&s.anchor.valid)
    {
      current=s.plan;current.points[0]=toField(s.pointsOdo[0],i);current.ball=toField(s.ballOdo,i);
      current.ballTime=std::max(0.f,s.plan.ballTime-(i.now-s.committedAt)*.001f);
      const Point expected=predict(current.ballTime);
      // A rejected remote prediction is NOT a valid old full target. Only its
      // separately cached, currently revalidated short prefix may survive.
      if(finite(expected)&&distance(expected,current.ball)<=300.f&&distance(expected,i.contract)<=1500.f)
      {currentPath=native(current.points[0]);current=shortestInspect(current,i,currentPath,legal);}
      else current.valid=false;
    }
    const bool replan=fresh&&(!current.valid||!s.plannedAt||i.now-s.plannedAt>=250u);
    if(replan)
    {
      s.plannedAt=i.now;const unsigned available=s.nativeQueries<7u?7u-s.nativeQueries:0u;
      auto next=shortestChoose(i,predict,native,legal,std::min(4u,available>1u?available-1u:0u));
      if(shortestReplace(next.plan,current,i))
      {
        current=next.plan;currentPath=std::move(next.path);s.plan=current;s.segment=0;s.committedAt=i.now;
        s.pointsOdo[0]=toOdo(current.points[0],i);s.ballOdo=toOdo(current.ball,i);
        if(s.awaitingEpochCommit){++s.generation;s.awaitingEpochCommit=false;}
        s.headingField=i.heading;s.headingOdoYaw=i.odoHeading;
        s.anchor=WhiteFleetC15ReceiverRoute::State();s.anchor.valid=true;s.anchor.originField=i.robot;s.anchor.originOdometry=i.odometry;
        s.anchor.originHeading=i.heading;s.anchor.originOdoHeading=i.odoHeading;
      }
      else if(!next.plan.valid)s.shortestReason=next.plan.reason;
    }
    if(current.valid&&s.anchor.valid)
    {
      if(fresh&&i.seen>s.shortestPrefixSeen)
      {
        const Point prefix=along(currentPath.samples,std::min(1.f,1200.f/currentPath.length));
        s.shortestPrefixOdo=toOdo(prefix,i);s.shortestPrefixSeen=i.seen;s.shortestPrefixValid=true;
      }
      out.plan=current;out.target=current.points[0];out.kind=Kind::walk;
      out.reason=s.reason=s.shortestReason="shortest_native_walk";
    }
    else
    {
      if(!s.plan.valid||!s.anchor.valid||!s.shortestPrefixValid)return stop(s.shortestReason);
      Point local=rotate(sub(s.shortestPrefixOdo,i.odometry),-i.odoHeading);
      const Point snapshot=rotate(sub(s.ownBallOdo,i.odometry),-i.odoHeading);
      local=boundedGapStep(local,snapshot,i.relativeBall);
      const Point future=predict(std::min(1.f,shortestRemaining(i))),target=add(i.robot,rotate(local,i.heading));
      const Point forward=sub(future,i.robot),step=sub(target,i.robot);
      if(!finite(future)||distance(local,{})<=1.f||step.x<=0.f||distance(forward,{})<=1.f
         ||std::abs(angle(std::atan2(step.y,step.x)-std::atan2(forward.y,forward.x)))>.785398164f)
      {s.shortestPrefixValid=false;s.plan.valid=false;s.anchor.invalidate();return stop("untrusted_safe_prefix");}
      Plan prefix;prefix.shortest=true;prefix.size=1;prefix.points[0]=target;prefix.ball=i.ball;
      const auto path=native(target);prefix=shortestInspect(prefix,i,path,legal);
      if(!prefix.valid||!shortestGapRouteSafe(path,i,snapshot))
      {s.shortestPrefixValid=false;s.plan.valid=false;s.anchor.invalidate();out.plan=prefix;return stop(prefix.valid?"unsafe_prefix_ball_margin":prefix.reason);}
      prefix.localStepOnly=true;out.plan=prefix;out.target=target;out.kind=Kind::walk;
      out.reason=s.reason="shortest_safe_prefix";out.plan.tangent=prefix.tangent;
    }
    if(!fresh)
    {
      const Point local=rotate(sub(out.target,i.robot),-i.heading),snapshot=rotate(sub(s.ownBallOdo,i.odometry),-i.odoHeading);
      const Point clipped=boundedGapStep(local,snapshot,i.relativeBall);
      if(distance(clipped,{})<=1.f)return stop("unsafe_shortest_gap");
      out.target=add(i.robot,rotate(clipped,i.heading));
      Plan check=out.plan;check.points[0]=out.target;
      const auto path=native(out.target);check=shortestInspect(check,i,path,legal);
      if(!check.valid||!shortestGapRouteSafe(path,i,snapshot))
      {s.shortestPrefixValid=false;s.plan.valid=false;s.anchor.invalidate();out.plan=check;return stop(check.valid?"unsafe_gap_ball_margin":check.reason);}
      out.plan=check;
    }
    const float pathHeading=std::atan2(out.plan.tangent.y,out.plan.tangent.x);
    const float footHeading=std::atan2(-i.ball.y,i.field.opponentGoal-i.ball.x);
    const float blend=std::clamp((1500.f-distance(i.relativeBall,{}))/850.f,0.f,1.f);
    out.heading=angle(pathHeading+blend*angle(footHeading-pathHeading));
    s.anchor.lastOdometry=i.odometry;s.anchor.lastOdoHeading=i.odoHeading;s.anchor.lastFrame=i.now;
    return out;
  }
}
