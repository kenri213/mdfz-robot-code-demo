#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

/** Through-kickoff planning values only: no robot/module/environment dependency.
 * Units: millimetres, seconds for ETA/speeds, milliseconds for session budgets.
 * Response times and speed factors are engineering margins, not calibrated
 * bounds on a learned kick, receiver locomotion or opponent behaviour. */
namespace WhiteFleetKickoffThroughV8
{
  struct Point {float x=0.f,y=0.f;};
  struct Field {float ownGoal=-10940.f,opponentGoal=10940.f,opponentPenalty=7560.f,halfWidth=6940.f,goalHalfWidth=1300.f,penaltyHalfWidth=0.f;};
  struct Params
  {
    int kicker=3,side=0;
    float targetXFraction=.50f,targetYFraction=.52f,prepareX=-650.f;
    float prepareSafetyBackoff=0.f; // Navigation-only own-half clearance; canonical profile stays unchanged.
    unsigned minPrepare=250,maxPrepare=6500,flight=11000,finish=6000,total=22000;
    float fieldMargin=600.f,penaltyMargin=300.f,goalCone=.174532925f;
    float receiverSpeedFactor=.75f,releaseDelay=.25f,startDelay=.30f,headBodyYaw=.35f,headBodyDelay=.25f;
    float arrivalMargin=.25f,obstaclePadding=150.f,opponentReleaseAdvantage=.20f;
    float sampleSpacing=250.f;
    bool calibrated=false,adaptiveMeet=false;
    bool playbook=false; // Independent opt-in; no speed/TTL change.
    bool isolated=false; // Independent control phase, same C9 switch/right side.
    bool nativeReceive=false; // C11: legal real-ball pursuit, NOT a capture certificate.
    bool c14EarlyStart=false; // Explicit local simulation successor; zero preserves C13.
    bool c15DeepPass=false; // Explicit local deep-pass trial; preserves all C14 execution.
    bool restoreProvenPool=false; // Same C15 chain may use the pre-existing C14 table; no new targets.
    bool defenceAdaptiveAngle=false; // Before offer only: guarded contour-derived rays near existing points.
    bool insideCornerLane=false; // Optional inner side of a low defender, only near the real penalty corner.
    bool raceRendezvous=false; // Bounded rolling first-hop; separate from legacy stop-at-deep-P plans.
    bool directCenter=false; // Separate opt-in finite centre pass; never a rolling-race fallback.
    bool centerFast=false; // Independent C2 right/deep domain and measured local prediction.
    bool receiverPrepareHold=false; // C3: keep a truly standing receiver within the existing safe150 prepare domain.
    bool receiverBallApproach=false; // C4: movement-only real-ball approach; NOT an expanded capture or shooting permission.
    bool receiverFollowFromRelease=false; // C4 follow v2: fresh-ball navigation from the first legal run, never a fixed-P fallback.
    bool stableKickAim=false; // One qualified, bounded sender-foot ODOM aim; never writes localization.
    bool receiverEarlyNative=false; // Receiver-only native pursuit after the original legal fresh-motion release.
    bool receiverGapWalk=false; // Short, far-ball ODOM walk only; never stale-ball kicking.
    bool firstPassClearance=false; // Rank only already-admitted finite centre passes by existing corridor reserve.
    float firstPassPrepareAdvance=0.f; // Explicit legal READY target shift; never substitutes for the actual peer pose.
    float prepareYFraction=0.f; // Zero retains the original targetYFraction staging.
    bool legalPreload=false; // Fleet successor: own-half preparation, NOT a ball-motion certificate.
    float c12RightMinDegrees=0.f,c12ReserveMin=0.f; // Local simulation experiment; zero preserves C11.
    float exitRatio=1.06f,rollRatio=.80f,transitionSeconds=.25f;
    float empiricalReceiverSpeed=1200.f,empiricalReleaseDelay=.60f,receiveWait=1.f;
    float predictionDeceleration=0.f; // C2 forecast ONLY [mm/s^2]; zero keeps actual-spec legacy prediction.
  };
  struct Obstacle
  {
    Point center{},left{},right{};
    bool mobile=true,teammate=false;
    float speedBound=2200.f,radius=0.f;
    bool angleSource=false; // New rays require fresh local/identified-global association; safety still uses every obstacle.
    float calibratedMotionSpeed=-1.f; // Tactical estimate, NOT an actuator bound; unknown retains speedBound.
  };
  struct Plan
  {
    bool valid=false;
    Point target{},prepare{};
    float nominalLength=0.f,mappedLength=0.f,initialSpeed=0.f,ballEta=0.f,receiverEta=0.f,safetyMargin=0.f;
    int side=0;
    const char* reason="invalid_input";
    float exitSpeed=0.f,rollSpeed=0.f,receiveUntil=0.f;
    bool captureTiming=false;
    Point timingOrigin{},timingTarget{},captureBall{};
    float captureTime=0.f;
    bool nativeRiskChecked=false;
    float nativePass=0.f,nativeCapturePass=0.f,nativeGoal=0.f;
    bool earlyRiskChecked=false;
    float earlyClearance=0.f;
    bool rollingRendezvous=false;
    bool hardRaceConflict=false; // Fresh evaluated collision/time conflict; not a missing-data label.
    float temporalGap=0.f,controlBoundaryTime=0.f;
  };
  struct FlightSample {Point position{};float time=0.f;};
  struct FlightProfile
  {
    bool valid=false;
    float nominalLength=0.f,mappedLength=0.f,initialSpeed=0.f,stopDistance=0.f,terminalSpeed=0.f,ballEta=0.f;
    std::vector<FlightSample> samples;
    const char* reason="invalid_mapping";
    bool calibrated=false;
    float exitSpeed=0.f,rollSpeed=0.f,stopTime=0.f,receiveUntil=0.f;
    Point stopPoint{};
  };

  inline bool finite(Point p){return std::isfinite(p.x)&&std::isfinite(p.y);}
  inline float distance(Point a,Point b){return std::hypot(a.x-b.x,a.y-b.y);}
  inline bool fieldValid(const Field& f)
  {
    return std::isfinite(f.ownGoal)&&std::isfinite(f.opponentGoal)&&std::isfinite(f.opponentPenalty)
        &&std::isfinite(f.halfWidth)&&std::isfinite(f.goalHalfWidth)&&f.ownGoal<0.f
        &&f.opponentGoal>f.opponentPenalty&&f.opponentPenalty>0.f&&f.halfWidth>1500.f
        &&f.goalHalfWidth>100.f&&f.goalHalfWidth<f.halfWidth;
  }
  inline bool paramsValid(const Params& p)
  {
    const float values[]{p.targetXFraction,p.targetYFraction,p.prepareX,p.fieldMargin,p.penaltyMargin,p.goalCone,
      p.receiverSpeedFactor,p.releaseDelay,p.startDelay,p.headBodyYaw,p.headBodyDelay,p.arrivalMargin,
      p.obstaclePadding,p.opponentReleaseAdvantage,p.sampleSpacing,p.exitRatio,p.rollRatio,p.transitionSeconds,
      p.empiricalReceiverSpeed,p.empiricalReleaseDelay,p.receiveWait,p.c12RightMinDegrees,p.c12ReserveMin,p.prepareYFraction,p.predictionDeceleration,
      p.firstPassPrepareAdvance,p.prepareSafetyBackoff};
    for(float value:values)if(!std::isfinite(value))return false;
    return p.kicker>0&&(p.side==0||p.side==1||p.side==-1)&&p.targetXFraction>.15f&&p.targetXFraction<.85f
        &&p.targetYFraction>.15f&&p.targetYFraction<.85f&&p.prepareX<=-100.f&&p.fieldMargin>=300.f
        &&p.prepareSafetyBackoff>=0.f&&p.prepareSafetyBackoff<=2000.f
        &&p.penaltyMargin>=0.f&&p.goalCone>=.05f&&p.goalCone<1.f
        &&p.receiverSpeedFactor>0.f&&p.receiverSpeedFactor<=1.f&&p.releaseDelay>=0.f&&p.startDelay>=0.f
        &&p.headBodyYaw>=0.f&&p.headBodyYaw<1.2f&&p.headBodyDelay>=0.f&&p.arrivalMargin>=0.f
        &&p.obstaclePadding>=0.f&&p.opponentReleaseAdvantage>=0.f&&p.sampleSpacing>=50.f&&p.sampleSpacing<=500.f
        &&p.minPrepare<=p.maxPrepare&&p.maxPrepare>0&&p.flight>0&&p.finish>0&&p.total>p.maxPrepare
        &&p.exitRatio>=.8f&&p.exitRatio<=1.3f&&p.rollRatio>=.65f&&p.rollRatio<=1.f&&p.rollRatio<=p.exitRatio
        &&p.transitionSeconds>=.1f&&p.transitionSeconds<=.5f&&p.empiricalReceiverSpeed>=600.f&&p.empiricalReceiverSpeed<=1600.f
        &&p.empiricalReleaseDelay>=.2f&&p.empiricalReleaseDelay<=1.5f&&p.receiveWait>=0.f&&p.receiveWait<=1.f
        &&p.c12RightMinDegrees>=0.f&&p.c12RightMinDegrees<=60.f&&(p.c12ReserveMin==0.f||p.c12ReserveMin==800.f)
        &&(!p.restoreProvenPool||(p.c15DeepPass&&p.c14EarlyStart&&p.nativeReceive&&p.isolated&&p.playbook&&p.side==-1))
        &&(!p.defenceAdaptiveAngle||(p.restoreProvenPool&&p.calibrated&&p.adaptiveMeet&&p.c12RightMinDegrees>=40.f))
        &&(!p.insideCornerLane||p.defenceAdaptiveAngle)
        &&(p.prepareYFraction==0.f||(p.prepareYFraction>=.15f&&p.prepareYFraction<=.85f))
        &&(!p.raceRendezvous||(p.defenceAdaptiveAngle&&p.prepareYFraction>0.f&&p.side==-1))
        &&(!p.directCenter||(p.kicker==3&&p.side==-1&&p.calibrated&&p.adaptiveMeet&&p.playbook&&p.isolated
          &&p.c14EarlyStart&&!p.nativeReceive&&!p.raceRendezvous&&!p.c15DeepPass&&!p.restoreProvenPool
          &&!p.defenceAdaptiveAngle&&!p.insideCornerLane&&!p.legalPreload&&p.prepareX==(p.centerFast?-750.f+p.firstPassPrepareAdvance:-850.f)
          &&p.prepareYFraction==.15f&&p.headBodyYaw==0.f&&p.c12RightMinDegrees==0.f))
        &&(!p.centerFast||(p.directCenter&&p.predictionDeceleration>=380.f&&p.predictionDeceleration<=450.f))
        &&(p.centerFast||p.predictionDeceleration==0.f)
        &&(!p.receiverPrepareHold||p.centerFast)
        &&(!p.receiverBallApproach||(p.centerFast&&p.receiverPrepareHold))
        &&(!p.receiverFollowFromRelease||p.receiverBallApproach)
        &&(!p.stableKickAim||p.receiverFollowFromRelease)
        &&(!p.receiverEarlyNative||p.stableKickAim)
        &&(!p.receiverGapWalk||p.receiverEarlyNative)
        &&(!p.firstPassClearance||(p.centerFast&&p.stableKickAim))
        &&(p.firstPassPrepareAdvance==0.f||(p.firstPassPrepareAdvance==250.f
          &&p.centerFast&&p.stableKickAim&&p.firstPassClearance));
  }
  inline bool validPlan(const Plan& p)
  {
    return p.valid&&finite(p.target)&&finite(p.prepare)&&(p.side==1||p.side==-1)
        &&std::isfinite(p.nominalLength)&&p.nominalLength>0.f&&std::isfinite(p.mappedLength)&&p.mappedLength>0.f
        &&std::isfinite(p.initialSpeed)&&p.initialSpeed>0.f&&std::isfinite(p.ballEta)&&p.ballEta>0.f
        &&std::isfinite(p.receiverEta)&&p.receiverEta>=0.f&&std::isfinite(p.safetyMargin)&&p.safetyMargin>=0.f;
  }
  inline int effectiveSide(const Params& p,Point receiver)
  {return p.side?p.side:finite(receiver)?(receiver.y<0.f?-1:1):0;}
  inline Point preparePoint(const Field& f,const Params& p,int side)
  {return {p.prepareX-p.prepareSafetyBackoff,static_cast<float>(side)*f.halfWidth*(p.prepareYFraction>0.f?p.prepareYFraction:p.targetYFraction)};}
  inline float receiverPrepareHoldRadius(const Params& p,bool actualMotionStand)
  {return p.receiverPrepareHold&&actualMotionStand?150.f:100.f;}

  inline Point directCenterTarget(const Field& f)
  {return {f.opponentPenalty-1000.f,0.f};}
  inline Point directCenterTarget(const Field& f,const Params& p)
  {return p.centerFast?Point{f.opponentPenalty+250.f,-1000.f}:directCenterTarget(f);}
  inline bool directCenterDomain(Point target,const Field& f,const Params& p,float tolerance=0.f)
  {
    if(!p.directCenter||!paramsValid(p)||!fieldValid(f)||!finite(target)||!std::isfinite(tolerance)
       ||tolerance<0.f||tolerance>300.f)return false;
    if(p.centerFast)return target.x>=f.opponentPenalty-250.f-tolerance&&target.x<=f.opponentPenalty+500.f+tolerance
        &&target.y>=-2000.f-tolerance&&target.y<=-500.f+tolerance;
    return std::abs(target.x-directCenterTarget(f).x)<=tolerance&&std::abs(target.y)<=tolerance&&target.x>250.f;
  }

  inline bool targetSafe(Point ball,Point target,const Field& f,const Params& p,int side,float wireTolerance=0.f)
  {
    if(!fieldValid(f)||!paramsValid(p)||!finite(ball)||!finite(target)||(side!=1&&side!=-1))return false;
    if(!std::isfinite(wireTolerance)||wireTolerance<0.f||wireTolerance>300.f||(!p.centerFast&&wireTolerance!=0.f))return false;
    if(ball.x<f.ownGoal||ball.x>f.opponentGoal||std::abs(ball.y)>f.halfWidth)return false;
    const float dx=target.x-ball.x,dy=target.y-ball.y;
    if(dx<=250.f||target.x<=0.f||target.x>(p.centerFast?f.opponentPenalty+500.f+wireTolerance:f.opponentPenalty-p.penaltyMargin)
       ||target.x>f.opponentGoal-p.fieldMargin||std::abs(target.y)>f.halfWidth-p.fieldMargin
       ||target.x<f.ownGoal+p.fieldMargin)return false;
    // This is a different, explicitly selected finite-stop PASS domain. The
    // planner must validate the predicted stop before the box, not extrapolate
    // this ray to the goal as if it were an intentional direct kickoff shot.
    if(p.directCenter)return directCenterDomain(target,f,p,p.centerFast?wireTolerance:300.f);
    if(target.y*side<=0.f)return false;
    const float angle=std::atan2(dy,dx);
    if(std::abs(angle)<p.goalCone)return false;
    const float goalY=ball.y+(f.opponentGoal-ball.x)*dy/dx;
    return std::isfinite(goalY)&&std::abs(goalY)>f.goalHalfWidth+300.f;
  }
  inline Point centerMeetingBody(Point launch,Point target,Point ball,const Params& p)
  {
    if(!p.centerFast||!finite(launch)||!finite(target)||!finite(ball))return target;
    const float dx=target.x-launch.x,dy=target.y-launch.y,d2=dx*dx+dy*dy,d=distance(ball,target);
    if(d2<=1.f||d>1500.f||((ball.x-launch.x)*dx+(ball.y-launch.y)*dy)/d2<.5f)return target;
    return d>1190.f?Point{target.x+(ball.x-target.x)*1190.f/d,target.y+(ball.y-target.y)*1190.f/d}:ball;
  }
  struct ReceiverApproach
  {
    bool valid=false;
    Point body{};
    float heading=0.f;
  };
  inline ReceiverApproach receiverBallApproach(Point launch,Point target,Point ball,Point position,
                                               const Field& f,const Params& p)
  {
    ReceiverApproach result;
    if(!p.receiverBallApproach||!paramsValid(p)||!fieldValid(f)||!finite(launch)||!finite(target)
       ||!finite(ball)||!finite(position)||!directCenterDomain(target,f,p,300.f))return result;
    const float dx=target.x-launch.x,dy=target.y-launch.y,l2=dx*dx+dy*dy;
    if(l2<=1.f)return result;
    const float progress=((ball.x-launch.x)*dx+(ball.y-launch.y)*dy)/l2;
    const float crossTrack=std::abs((ball.x-launch.x)*dy-(ball.y-launch.y)*dx)/std::sqrt(l2);
    // This corridor is ONLY a navigation envelope along the admitted pass.
    // The locked endpoint and original 650/1500 capture remain unchanged.
    // This flag changes PURE NAVIGATION only. Session still requires native
    // PLAY + real fresh observed motion; the foot/capture gates stay separate.
    // Do not wait for half the pass or a 2.5m range while blindly running to P.
    if(progress>(1.1f)||crossTrack>1500.f)return result;
    if(p.receiverFollowFromRelease)
    {if(progress<=0.f)return result;}
    else if(progress<.5f||distance(position,ball)>2500.f)return result;
    const float gx=f.opponentGoal-ball.x,gy=-ball.y,length=std::hypot(gx,gy);
    if(length<=350.f)return result;
    result.heading=std::atan2(gy,gx);
    result.body={ball.x-350.f*gx/length,ball.y-350.f*gy/length};
    if(result.body.x<f.ownGoal+300.f||result.body.x>f.opponentGoal-300.f
       ||std::abs(result.body.y)>f.halfWidth-300.f)return {};
    result.valid=true;
    return result;
  }
  inline bool receiverBallInFront(Point relative)
  {return finite(relative)&&relative.x>0.f&&std::abs(std::atan2(relative.y,relative.x))<=.7853981634f;}
  inline std::vector<Point> targetCandidates(const Field& f,const Params& p,int side)
  {
    std::vector<Point> result;
    if(!fieldValid(f)||!paramsValid(p)||(side!=1&&side!=-1))return result;
    for(float x:{p.targetXFraction,p.targetXFraction-.05f,p.targetXFraction+.05f})
      for(float y:{p.targetYFraction,p.targetYFraction-.06f,p.targetYFraction+.06f})
        result.push_back({f.opponentGoal*x,static_cast<float>(side)*f.halfWidth*y});
    if(p.playbook)
      for(Point fraction:std::vector<Point>{{.25f,.41f},{.18f,.36f},{.30f,.52f},{.20f,.52f},{.15f,.52f}})
        result.push_back({f.opponentGoal*fraction.x,side*f.halfWidth*fraction.y});
    return result;
  }

  /** Receiver starts from the LEGAL prepare point, only after native release.
   * A modest inward body yaw leaves head range for looking at the ball while
   * still running forwards. The ellipse and margins do not alter native speed. */
  inline float receiverTravelTime(Point from,Point target,float maxForward,float maxSide,float maxRotation,const Params& p)
  {
    if(!finite(from)||!finite(target)||!paramsValid(p)||!std::isfinite(maxForward)||maxForward<=0.f
       ||!std::isfinite(maxSide)||maxSide<=0.f||!std::isfinite(maxRotation)||maxRotation<=0.f)
      return std::numeric_limits<float>::infinity();
    const float length=distance(from,target);
    const float forward=length*std::cos(p.headBodyYaw)/(maxForward*p.receiverSpeedFactor);
    const float lateral=length*std::sin(p.headBodyYaw)/(maxSide*p.receiverSpeedFactor);
    return std::hypot(forward,lateral)+p.headBodyYaw/maxRotation+p.releaseDelay+p.startDelay+p.headBodyDelay;
  }
  inline float empiricalReceiverTime(Point from,Point target,float heading,float maxForward,float maxSide,float maxRotation,
                                     const Params& p,bool initialRelease=true)
  {
    if(!finite(from)||!finite(target)||!std::isfinite(heading)||!paramsValid(p)
       ||!std::isfinite(maxForward)||!std::isfinite(maxSide)||!std::isfinite(maxRotation)
       ||maxForward<=0.f||maxSide<=0.f||maxRotation<=0.f)return std::numeric_limits<float>::infinity();
    const float dx=target.x-from.x,dy=target.y-from.y,length=std::hypot(dx,dy);
    const float forward=std::min(maxForward*p.receiverSpeedFactor,p.empiricalReceiverSpeed);
    const float lateral=std::min(maxSide*p.receiverSpeedFactor,900.f);
    const float side=from.y<0.f?1.f:-1.f;
    const float desired=(length>1.f?std::atan2(dy,dx):heading)+side*p.headBodyYaw;
    const float turn=std::abs(std::atan2(std::sin(desired-heading),std::cos(desired-heading)))/std::min(maxRotation,1.5f);
    return std::hypot(length*std::cos(p.headBodyYaw)/forward,length*std::sin(p.headBodyYaw)/lateral)
        +turn+p.startDelay+p.headBodyDelay+(initialRelease?p.empiricalReleaseDelay:0.f);
  }
  inline float pointSegmentDistance(Point p,Point a,Point b)
  {
    const float dx=b.x-a.x,dy=b.y-a.y,l2=dx*dx+dy*dy;
    if(l2<=1e-6f)return distance(p,a);
    const float t=std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/l2,0.f,1.f);
    return distance(p,{a.x+t*dx,a.y+t*dy});
  }
  inline float cross(Point a,Point b,Point c)
  {return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);}
  inline float segmentDistance(Point a,Point b,Point c,Point d)
  {
    const float abc=cross(a,b,c),abd=cross(a,b,d),cda=cross(c,d,a),cdb=cross(c,d,b);
    if(((abc>0.f&&abd<0.f)||(abc<0.f&&abd>0.f))&&((cda>0.f&&cdb<0.f)||(cda<0.f&&cdb>0.f)))return 0.f;
    return std::min({pointSegmentDistance(a,c,d),pointSegmentDistance(b,c,d),pointSegmentDistance(c,a,b),pointSegmentDistance(d,a,b)});
  }
  inline bool insideTriangle(Point p,const Obstacle& o)
  {
    if(std::abs(cross(o.center,o.left,o.right))<1e-5f)return false;
    const float a=cross(o.center,o.left,p),b=cross(o.left,o.right,p),c=cross(o.right,o.center,p);
    return !((a<0.f||b<0.f||c<0.f)&&(a>0.f||b>0.f||c>0.f));
  }
  inline float obstacleDistance(Point a,Point b,const Obstacle& o)
  {
    if(insideTriangle(a,o)||insideTriangle(b,o))return 0.f;
    return std::min({segmentDistance(a,b,o.left,o.right),segmentDistance(a,b,o.center,o.left),segmentDistance(a,b,o.center,o.right)});
  }

  inline Plan evaluate(Point ball,Point receiver,Point target,const Field& field,const std::vector<Obstacle>& obstacles,
                       const Params& p,int side,const FlightProfile& flight,float maxForward,float maxSide,float maxRotation,float ballRadius,
                       float receiverHeading=std::numeric_limits<float>::quiet_NaN())
  {
    Plan plan;plan.target=target;plan.prepare=preparePoint(field,p,side);plan.side=side;
    if(!finite(receiver)||!targetSafe(ball,target,field,p,side)||!std::isfinite(ballRadius)||ballRadius<=0.f
       ||!finite(plan.prepare)||plan.prepare.x<field.ownGoal+p.fieldMargin
       ||std::abs(plan.prepare.y)>field.halfWidth-p.fieldMargin)
    {plan.reason="invalid_geometry";return plan;}
    plan.nominalLength=flight.nominalLength;plan.mappedLength=flight.mappedLength;plan.initialSpeed=flight.initialSpeed;plan.ballEta=flight.ballEta;
    plan.exitSpeed=flight.exitSpeed;plan.rollSpeed=flight.rollSpeed;plan.receiveUntil=flight.receiveUntil;
    if(!flight.valid||flight.samples.size()<2||!std::isfinite(flight.ballEta)||flight.ballEta<=0.f
       ||flight.ballEta>p.flight*.001f||distance(flight.samples.front().position,ball)>1.f
       ||distance(flight.samples.back().position,target)>1.f||std::abs(flight.samples.front().time)>.001f
       ||std::abs(flight.samples.back().time-flight.ballEta)>.001f)
    {plan.reason=flight.reason;return plan;}
    // A distant receiver cannot be assumed to have reached the legal start.
    const float preparation=receiverTravelTime(receiver,plan.prepare,maxForward,maxSide,maxRotation,p);
    if(!std::isfinite(preparation)||preparation>p.maxPrepare*.001f)
    {plan.reason="prepare_unreachable";return plan;}
    if(p.calibrated)
    {
      if(!flight.calibrated||!finite(flight.stopPoint)||distance(flight.stopPoint,target)>1200.f
         ||!std::isfinite(flight.receiveUntil)||flight.receiveUntil<flight.ballEta||flight.receiveUntil>p.flight*.001f)
      {plan.reason="invalid_calibrated_reception";return plan;}
      if(!std::isfinite(receiverHeading))receiverHeading=side>0?-1.3f:1.3f;
      plan.receiverEta=empiricalReceiverTime(plan.prepare,flight.stopPoint,receiverHeading,maxForward,maxSide,maxRotation,p);
    }
    else plan.receiverEta=receiverTravelTime(plan.prepare,target,maxForward,maxSide,maxRotation,p);
    plan.safetyMargin=(p.calibrated?flight.receiveUntil:flight.ballEta)-plan.receiverEta;
    if(!std::isfinite(plan.receiverEta)||plan.safetyMargin<p.arrivalMargin)
    {plan.reason="receiver_late";return plan;}
    float previousTime=-1.f;
    for(const FlightSample& sample:flight.samples)
    {
      if(!finite(sample.position)||!std::isfinite(sample.time)||sample.time<0.f||sample.time<previousTime
         ||sample.time>flight.ballEta+.001f)
      {plan.reason="invalid_flight";return plan;}
      previousTime=sample.time;
    }
    for(const Obstacle& obstacle:obstacles)
    {
      if(!finite(obstacle.center)||!finite(obstacle.left)||!finite(obstacle.right)||!std::isfinite(obstacle.radius)||obstacle.radius<0.f
         ||!std::isfinite(obstacle.speedBound)||obstacle.speedBound<0.f)
      {plan.reason="invalid_obstacle";return plan;}
      const float padding=obstacle.radius+ballRadius+p.obstaclePadding;
      for(std::size_t i=1;i<flight.samples.size();++i)
      {
        const auto& a=flight.samples[i-1];const auto& b=flight.samples[i];
        const float clearance=obstacleDistance(a.position,b.position,obstacle);
        if(clearance<=padding){plan.reason="blocked_corridor";return plan;}
        const float available=b.time+p.opponentReleaseAdvantage;
        if(obstacle.mobile&&clearance<=padding+obstacle.speedBound*available)
        {plan.reason="opponent_first";return plan;}
      }
      // The ball may arrive before a measured-speed receiver. Do not pretend
      // this safe waiting interval exists when an opponent reaches it first.
      if(p.calibrated)
      {
        const float clearance=obstacleDistance(target,flight.stopPoint,obstacle);
        if(clearance<=padding+(obstacle.mobile?obstacle.speedBound*(plan.receiverEta+p.arrivalMargin+p.opponentReleaseAdvantage):0.f))
        {plan.reason="opponent_first_at_receive";return plan;}
      }
    }
    plan.valid=true;plan.reason="ready";return plan;
  }
}
