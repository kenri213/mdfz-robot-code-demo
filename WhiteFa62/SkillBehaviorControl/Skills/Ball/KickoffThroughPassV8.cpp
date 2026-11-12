/** Scoped kickoff through-pass outputs. Session/ownership decisions live upstream. */
#include "WhiteFleetSkillBehaviorControl.h"
#include "WhiteFleetSimPhysical.h"
#include "WhiteFleetC15KickoffSwitch.h"
#include "WhiteFleetC9ForwardFlank.h"
#include "WhiteFleetC15RollingRendezvous.h"
#include "Platform/SystemCall.h"
#include "WhiteFleetK1V21MatchProfile.h"
#include "Framework/Settings.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

option((WhiteFleetSkillBehaviorControl) KickoffThroughPassV8)
{
  // Do not call a child option before this real state/action has named the
  // ActivationGraph entry. In particular, first-frame rejection is an action.
  initial_state(execute)
  {
    action
    {
      const int stageAction=theSkillRequest.kickoffThroughAction;
      const bool kicker=theGameState.playerNumber==theSkillRequest.kickoffThroughKicker;
      const bool receiver=theGameState.playerNumber==theSkillRequest.kickoffThroughReceiver;
      const bool ownKickoff=theGameState.state==GameState::ownKickOff && theGameState.isForOwnTeam();
      const bool playing=theGameState.state==GameState::playing;
      const bool ready=theGameState.state==GameState::setupOwnKickOff;
      const bool set=theGameState.state==GameState::waitForOwnKickOff;
      const auto profile=WhiteFleetK1V21MatchProfile::guardedPrepare(WhiteFleetC15KickoffSwitch::configuredForwardPrepareParams(theBehaviorParameters.c15EarlyStart,theBehaviorParameters.c15RestoreProvenPool,theBehaviorParameters.c15DefenceAdaptiveAngle,theBehaviorParameters.c15InsideCornerLane,theBehaviorParameters.c15RaceRendezvous,theBehaviorParameters.c15DirectCenterKickoff,theBehaviorParameters.c15CenterFastKickoff,theBehaviorParameters.c15ReceiverPrepareHold,theBehaviorParameters.c15ReceiverBallApproach,theBehaviorParameters.c15ReceiverFollowFromRelease,theBehaviorParameters.c15StableKickAim,theBehaviorParameters.c15ReceiverEarlyNative,theBehaviorParameters.c15ReceiverGapWalk,theBehaviorParameters.c15FirstPassClearance,theBehaviorParameters.c15ForwardPrepare),theFieldDimensions.xPosHalfwayLine,theFieldDimensions.fieldLinesWidth,theBehaviorParameters.illegalAreasLargeDistance);
      const WhiteFleetKickoffThroughV8::Field field{theFieldDimensions.xPosOwnGoalLine,theFieldDimensions.xPosOpponentGoalLine,
          theFieldDimensions.xPosOpponentPenaltyArea,theFieldDimensions.yPosLeftTouchline,theFieldDimensions.yPosLeftGoal};
      const bool fixedArrivalMode=theBehaviorParameters.c15ReceiverFixedRunToArrival;
      const bool simFlow=theBehaviorParameters.c15ReceiverSimFlow&&theBehaviorParameters.c15ReceiverSimMaxRun
          &&theBehaviorParameters.c15ReceiverBoundedFastRun&&theBehaviorParameters.c15ReceiverCoordinateRun
          &&fixedArrivalMode&&theBehaviorParameters.c15ReceiverFixedDirect
          &&theBehaviorParameters.c15EarlyChannelReceive&&theBehaviorParameters.c15ReceiverLocalStep
          &&!theBehaviorParameters.c15ReceiverLowerChannel&&!theBehaviorParameters.c15ReceiverShortestIntercept
          &&profile.firstPassPrepareAdvance==250.f&&receiver&&theGameState.playerNumber==2
          &&WhiteFleetSimPhysical::simulated()
          &&Global::getSettings().teamNumber==5&&Global::getSettings().playerNumber==2;
      const bool physicalFlow=theBehaviorParameters.c15ReceiverPhysicalFlow
          &&WhiteFleetSimPhysical::physical()&&WhiteFleetK1V21MatchProfile::guardedPhysicalReceiver()
          &&theBehaviorParameters.c15EarlyChannelReceive&&theBehaviorParameters.c15ReceiverLocalStep
          &&!theBehaviorParameters.c15ReceiverSimMaxRun&&!theBehaviorParameters.c15ReceiverSimFlow
          &&!theBehaviorParameters.c15ReceiverFixedDirect&&!fixedArrivalMode
          &&!theBehaviorParameters.c15ReceiverBoundedFastRun&&!theBehaviorParameters.c15ReceiverCoordinateRun
          &&!theBehaviorParameters.c15ReceiverLowerChannel&&!theBehaviorParameters.c15ReceiverShortestIntercept
          &&profile.firstPassPrepareAdvance==250.f&&receiver;
      if(theBehaviorParameters.c15ReceiverPhysicalFlow&&!physicalFlow&&receiver)
      {LookActive({.withBall=true});Stand();return;}
      if(fixedArrivalMode&&receiver)c15EarlyChannel.begin(theSkillRequest.kickoffThroughEvent);
      const bool fixedSession=fixedArrivalMode&&theBehaviorParameters.c15ReceiverFixedDirect
          &&theBehaviorParameters.c15EarlyChannelReceive&&theBehaviorParameters.c15ReceiverLocalStep
          &&!theBehaviorParameters.c15ReceiverLowerChannel&&!theBehaviorParameters.c15ReceiverShortestIntercept
          &&profile.firstPassPrepareAdvance==250.f&&receiver&&playing&&stageAction==7
          &&theSkillRequest.kickoffThrough&&theSkillRequest.kickoffThroughPublish
          &&!theSkillRequest.kickoffThroughRetired&&!theSkillRequest.isolatedKickoffFinished
          &&theSkillRequest.kickoffThroughHasLaunch&&theSkillRequest.kickoffThroughLaunch.allFinite()
          &&WhiteFleetC15EarlyChannel::fixedCurrentWitness(theFrameInfo.time,theSkillRequest.kickoffThroughEvent,
              theSkillRequest.kickoffFixedRunFrame,theSkillRequest.kickoffFixedRunRelease,
              theSkillRequest.kickoffThroughKicker,theSkillRequest.kickoffThroughReceiver);
      const bool fixedCarry=fixedSession&&WhiteFleetC15EarlyChannel::fixedArrivalContext(c15EarlyChannel,theFrameInfo.time,
          theSkillRequest.kickoffThroughEvent,theSkillRequest.kickoffFixedRunFrame,theSkillRequest.kickoffFixedRunRelease,
          theSkillRequest.kickoffThroughKicker,theSkillRequest.kickoffThroughReceiver,
          {theSkillRequest.kickoffThroughTarget.x(),theSkillRequest.kickoffThroughTarget.y()},
          {theBehaviorParameters.c15FixedReceiveBallX,theBehaviorParameters.c15FixedReceiveBallY});
      const bool nativeReceiveIntercept=(profile.raceRendezvous||profile.directCenter)&&receiver&&
          ((playing&&(stageAction==4||stageAction==6||stageAction==7))||
           (ownKickoff&&(stageAction==2||(profile.directCenter&&stageAction==8))))&&
          theSkillRequest.kickoffThrough&&theSkillRequest.kickoffThroughPublish&&
          theBallModel.timeWhenLastSeen>0&&theBallModel.timeWhenLastSeen<=theFrameInfo.time&&
          theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen)<=250&&theFieldBall.ballWasSeen(250)&&
          theFieldBall.positionRelative.allFinite()&&
          theFieldBall.positionOnField.allFinite()&&
          theSkillRequest.kickoffThroughHasLaunch&&theSkillRequest.kickoffThroughLaunch.allFinite()&&
          WhiteFleetC15RollingRendezvous::approachingOwnContract({theFieldBall.positionOnField.x(),theFieldBall.positionOnField.y()},
              {theSkillRequest.kickoffThroughLaunch.x(),theSkillRequest.kickoffThroughLaunch.y()},
              {theSkillRequest.kickoffThroughTarget.x(),theSkillRequest.kickoffThroughTarget.y()})&&
          (profile.directCenter?WhiteFleetKickoffThroughV8::directCenterDomain(
              {theSkillRequest.kickoffThroughTarget.x(),theSkillRequest.kickoffThroughTarget.y()},field,profile,300.f):
           WhiteFleetC15RollingRendezvous::endpoint({theSkillRequest.kickoffThroughTarget.x(),theSkillRequest.kickoffThroughTarget.y()},field,profile,300.f))&&
          (!playing||!theIllegalAreas.isPositionIllegal(theFieldBall.positionOnField,150.f));
      const bool intercept=theInterceptBallInformation.walkOutOfBallPath||
          ((theInterceptBallInformation.intercept||theFieldInterceptBall.interceptBall||theFieldInterceptBall.predictedInterceptBall)&&!nativeReceiveIntercept&&!fixedSession);
      const bool receiverContact=profile.firstPassPrepareAdvance==250.f&&receiver&&playing&&stageAction==7
          &&theFallDownState.state==FallDownState::staggering&&theFallDownState.direction==FallDownState::none
          &&theFallDownState.timestampSinceStateSwitch>=theSkillRequest.kickoffThroughPostureSince
          &&theFallDownState.timestampSinceStateSwitch>0&&theFallDownState.timestampSinceStateSwitch<=theFrameInfo.time
          &&theFrameInfo.time-theFallDownState.timestampSinceStateSwitch<=100u
          &&theMotionInfo.executedPhase==MotionPhase::walk&&theMotionInfo.isMotionStable
          &&theSkillRequest.kickoffThroughPostureFrame==theFrameInfo.time
          &&theSkillRequest.kickoffThroughPostureSince>=theSkillRequest.kickoffThroughEvent
          &&theSkillRequest.kickoffThroughPostureSince>0&&theSkillRequest.kickoffThroughPostureSince<=theFrameInfo.time
          &&theFrameInfo.time-theSkillRequest.kickoffThroughPostureSince<=100u
          &&theBallModel.timeWhenLastSeen>0&&theBallModel.timeWhenLastSeen<=theFrameInfo.time
          &&theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen)<=250&&theFieldBall.ballWasSeen(250)
          &&theFieldBall.positionRelative.allFinite()&&theFieldBall.positionOnField.allFinite()&&theBallModel.estimate.position.allFinite()
          &&theFieldDimensions.isInsideField(theRobotPose.translation)&&theFieldDimensions.isInsideField(theFieldBall.positionOnField)
          &&theFieldBall.positionRelative.norm()<=650.f
          &&WhiteFleetKickoffThroughV8::receiverBallInFront({theFieldBall.positionRelative.x(),theFieldBall.positionRelative.y()});
      const bool allowed=WhiteFleetC15KickoffSwitch::enabled(theBehaviorParameters.matchTacticalRepair&&theBehaviorParameters.c15Kickoff) && theSkillRequest.kickoffThrough
          && (kicker||receiver) && !theGameState.isGoalkeeper()
          && theSkillRequest.kickoffThroughKicker!=theSkillRequest.kickoffThroughReceiver
          && theSkillRequest.kickoffThroughEvent>0 && theSkillRequest.kickoffThroughEvent<=theFrameInfo.time
          && (ownKickoff||playing||ready||set) && !theGameState.stopped && theGameControllerData.stopped==0
          && (!observationHeld||fixedCarry) // Quarantined ball cannot authenticate or grant feet; fixed navigation uses no ball.
          && theGameState.playerState==GameState::active && !WhiteFleetBoosterPickupControl::held()
          && !theBehaviorStateInformation.remoteControlActive && !theSystemSensorData.emergencyMode
          && !theEmergencyButtonState.buttonPressed && !intercept
          && (theFallDownState.state==FallDownState::upright||receiverContact)
          && (theMotionInfo.executedPhase==MotionPhase::stand || theMotionInfo.executedPhase==MotionPhase::walk)
          && theRobotPose.quality!=RobotPose::poor && theRobotPose.isFinite()
          && theSkillRequest.kickoffThroughTarget.allFinite()
          && theSkillRequest.kickoffThroughBodyTarget.allFinite()
          && std::isfinite(theSkillRequest.kickoffThroughLength)
          && (!theSkillRequest.kickoffThroughBallKnown || theSkillRequest.kickoffThroughBall.allFinite());
      const bool ownBall=theBallModel.timeWhenLastSeen>0 && theBallModel.timeWhenLastSeen<=theFrameInfo.time
          && theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen)<=650 && theFieldBall.ballWasSeen(650)
          && theFieldBall.positionRelative.allFinite() && theFieldBall.positionOnField.allFinite()
          && theBallModel.estimate.position.allFinite();
      const bool freshFootBall=ownBall && theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen)<=250;
      const bool adaptiveMeeting=profile.adaptiveMeet;
      const auto fixedGuardLog=[&](const char* reason,unsigned mask)
      {
        if(!theBehaviorParameters.c15ReceiverFixedDirect||!receiver||stageAction!=7)return;
        c15EarlyChannel.begin(theSkillRequest.kickoffThroughEvent);
        if(c15EarlyChannel.logs>=64u||(std::strcmp(reason,c15EarlyChannel.lastLogReason)==0&&theFrameInfo.time-c15EarlyChannel.lastLog<250u))return;
        ++c15EarlyChannel.logs;c15EarlyChannel.lastLog=theFrameInfo.time;c15EarlyChannel.lastLogReason=reason;
        std::printf("C15_FIXED_DIRECT event=%u frame=%u reason=%s hard_mask=%u actual_motion=%u seen=%u\n",
            theSkillRequest.kickoffThroughEvent,theFrameInfo.time,reason,mask,static_cast<unsigned>(theMotionRequest.motion),theBallModel.timeWhenLastSeen);
      };
      if(theBehaviorParameters.c15ReceiverSimMaxRun&&receiver&&stageAction==7)
      {
        const bool simOnly=WhiteFleetSimPhysical::simulated()&&Global::getSettings().teamNumber==5
            &&Global::getSettings().playerNumber==2&&theGameState.playerNumber==2;
        if(!simOnly){LookActive({.withBall=false});Stand();fixedGuardLog("SIM_max_run_forbidden",32768u);return;}
        const unsigned event=theSkillRequest.kickoffThroughEvent;
        if(c15SimMaxRun.event!=event){c15SimMaxRun=SimMaxRun();c15SimMaxRun.event=event;}
        if(!c15SimMaxRun.done)
        {
          // Explicit virtual collision ablation. Do not call the tactical/native
          // route, arm, field-area, fresh-near-margin or anchor-recovery paths.
          // Legal release/identity and actual stop/fault/bad-data still win.
          const bool hardAllowed=WhiteFleetC15KickoffSwitch::enabled(theBehaviorParameters.matchTacticalRepair&&theBehaviorParameters.c15Kickoff)
              &&fixedSession&&theBehaviorParameters.c15ReceiverBoundedFastRun&&theBehaviorParameters.c15ReceiverCoordinateRun
              &&!theGameState.stopped&&theGameControllerData.stopped==0&&theGameState.playerState==GameState::active
              &&!WhiteFleetBoosterPickupControl::held()&&!theBehaviorStateInformation.remoteControlActive
              &&!theSystemSensorData.emergencyMode&&!theEmergencyButtonState.buttonPressed
              &&theFallDownState.state==FallDownState::upright
              &&(theMotionInfo.executedPhase==MotionPhase::stand||theMotionInfo.executedPhase==MotionPhase::walk)
              &&theRobotPose.isFinite()&&theOdometryData.isFinite()
              &&theSkillRequest.kickoffThroughTarget.allFinite();
          if(!hardAllowed){LookActive({.withBall=false});Stand();fixedGuardLog("SIM_max_hard_stop",65536u);return;}
          const Vector2f ballRef(theBehaviorParameters.c15FixedReceiveBallX,theBehaviorParameters.c15FixedReceiveBallY);
          const Vector2f bodyRef(ballRef.x()-650.f,ballRef.y());
          if(!ballRef.allFinite()||!bodyRef.allFinite())
          {LookActive({.withBall=false});Stand();return;}
          if(!c15SimMaxRun.active)
          {
            const Vector2f observed=theRobotPose*theBallModel.estimate.position;
            if(theRobotPose.quality==RobotPose::poor||!freshFootBall||observationHeld
               ||theBallModel.timeWhenLastSeen<=event||!observed.allFinite()
               ||(observed-theSkillRequest.kickoffThroughLaunch).norm()<=300.f)
            {LookActive({.withBall=false});Stand();return;}
            c15SimMaxRun.active=true;c15SimMaxRun.release=theSkillRequest.kickoffFixedRunRelease;
            c15SimMaxRun.body=bodyRef;c15SimMaxRun.ball=ballRef;c15SimMaxRun.contract=theSkillRequest.kickoffThroughTarget;
          }
          if(c15SimMaxRun.release!=theSkillRequest.kickoffFixedRunRelease
             ||(c15SimMaxRun.body-bodyRef).norm()>1.f||(c15SimMaxRun.contract-theSkillRequest.kickoffThroughTarget).norm()>300.f)
          {LookActive({.withBall=false});Stand();return;}
          unsigned flowMask=0;bool flowCandidate=false,nativeFresh=false;
          const auto traceFlow=[&]()
          {
            const unsigned motion=static_cast<unsigned>(theMotionRequest.motion);
            if(!simFlow||(!flowCandidate&&!c15SimMaxRun.near)||c15SimMaxRun.nearLogs>=24u)return;
            if(c15SimMaxRun.lastNearLog&&flowMask==c15SimMaxRun.lastNearMask&&motion==c15SimMaxRun.lastNearMotion
               &&theFrameInfo.time-c15SimMaxRun.lastNearLog<250u)return;
            ++c15SimMaxRun.nearLogs;c15SimMaxRun.lastNearLog=theFrameInfo.time;
            c15SimMaxRun.lastNearMask=flowMask;c15SimMaxRun.lastNearMotion=motion;
            std::printf("SIM_FLOW_NEAR event=%u frame=%u candidate=%u entered=%u fresh_native=%d gate_mask=%u seen=%u range_mm=%.1f field_seen250=%d held=%d intercept=%d pose_quality=%u field_ball=(%.1f,%.1f) final_motion=%u drive_version=%u intent_not_capture=1\n",
                event,theFrameInfo.time,c15SimMaxRun.candidateAt,c15SimMaxRun.nearAt,nativeFresh,flowMask,
                theBallModel.timeWhenLastSeen,theBallModel.estimate.position.norm(),theFieldBall.ballWasSeen(250),
                observationHeld,intercept,static_cast<unsigned>(theRobotPose.quality),
                theFieldBall.positionOnField.x(),theFieldBall.positionOnField.y(),motion,theMotionRequest.fixedRunDriveCoordinateVersion);
          };
          if(simFlow)
          {
            const bool bodyInside=theFieldDimensions.isInsideField(theRobotPose.translation);
            const bool ballInside=theFieldBall.positionOnField.allFinite()&&theFieldDimensions.isInsideField(theFieldBall.positionOnField);
            const bool illegal=ballInside&&theIllegalAreas.isPositionIllegal(theFieldBall.positionOnField,150.f);
            const bool fieldFresh=theFieldBall.ballWasSeen(250);
            flowMask=(!freshFootBall?1u:0u)|(observationHeld?2u:0u)|(intercept?4u:0u)
                |(theRobotPose.quality==RobotPose::poor?8u:0u)|(!bodyInside?16u:0u)|(!ballInside?32u:0u)
                |(illegal?64u:0u)|(!fieldFresh?128u:0u);
            nativeFresh=flowMask==0;
            flowCandidate=theBallModel.estimate.position.allFinite()&&theBallModel.estimate.position.norm()<=1100.f;
            if(!c15SimMaxRun.candidateAt&&flowCandidate&&theBallModel.estimate.position.norm()<=1000.f
               &&theBallModel.timeWhenLastSeen>0&&theBallModel.timeWhenLastSeen<=theFrameInfo.time
               &&theFrameInfo.time-theBallModel.timeWhenLastSeen<=250u)c15SimMaxRun.candidateAt=theFrameInfo.time;
            if(!c15SimMaxRun.near&&nativeFresh&&theBallModel.estimate.position.norm()<=1000.f)
            {c15SimMaxRun.near=true;c15SimMaxRun.nearAt=theFrameInfo.time;}
            if(c15SimMaxRun.near)
            {
              // One-way original native approach, not a1m foot/capture claim.
              // Never carry the far-point velocity certificate into a ball skill.
              theMotionRequest.fixedRunDriveEvent=theMotionRequest.fixedRunDriveFrame=0;
              theMotionRequest.fixedRunDriveSpeed=0.f;theMotionRequest.fixedRunDriveCoordinateVersion=0;
              if(nativeFresh)KickAtGoal();
              else {LookActive({.withBall=true});Stand();} // Real fresh-foot protection, no far-point fallback.
              // Native goal output may publish its aim; keep the still-live
              // receiver intent separate, without inventing a capture/ACK.
              PassTarget({.passTarget=-1,.ballTarget=theRobotPose.inverse()*theSkillRequest.kickoffThroughTarget});
              traceFlow();
              return;
            }
          }
          // Preserve the original receiver's same-contract intent echo before
          // this fast path returns. Exactly one publication per path/frame.
          PassTarget({.passTarget=-1,.ballTarget=theRobotPose.inverse()*theSkillRequest.kickoffThroughTarget});
          const Vector2f local=theRobotPose.inverse()*c15SimMaxRun.body;
          if(local.norm()<=100.f||(theRobotPose.translation.x()>=bodyRef.x()&&std::abs(theRobotPose.translation.y()-bodyRef.y())<=100.f))
          {c15SimMaxRun.done=true;LookActive({.withBall=false});Stand();return;}
          const Vector2f head=freshFootBall&&!observationHeld?theBallModel.estimate.position:theRobotPose.inverse()*ballRef;
          LookAtPoint({.target={head.x(),head.y(),theBallSpecification.radius}});
          PublishMotion({.target=local,.speed={1.f,1.f,1.f}});
          WalkToPose({.target={local.angle(),local},.speed={1.f,1.f,1.f},.obstacleAvoidance={},
              .keepTargetRotation=true,.targetOfInterest={},.sideWalkingRequest=SideWalkingRequest::allowed});
          theMotionRequest.shouldInterceptBall=false;theMotionRequest.shouldWalkOutOfBallLine=false;
          theMotionRequest.fixedRunDriveEvent=event;theMotionRequest.fixedRunDriveFrame=theFrameInfo.time;
          theMotionRequest.fixedRunDriveSpeed=std::min(2000.f,theWalkingEngineOutput.maxSpeed.translation.x());
          theMotionRequest.fixedRunDriveCoordinateVersion=2;
          traceFlow();
          if(c15SimMaxRun.logs<48u&&(!c15SimMaxRun.lastLog||theFrameInfo.time-c15SimMaxRun.lastLog>=250u))
          {++c15SimMaxRun.logs;c15SimMaxRun.lastLog=theFrameInfo.time;
            std::printf("SIM_ONLY_RED2_MAX_RUN event=%u frame=%u release=%u full_destination_mm=%.1f requested_mm_s=%.1f collision=OFF arm=OFF area=OFF pose_correction=OFF ball_margin=OFF taper=OFF motion=%u\n",
                event,theFrameInfo.time,c15SimMaxRun.release,local.norm(),theMotionRequest.fixedRunDriveSpeed,static_cast<unsigned>(theMotionRequest.motion));}
          return;
        }
      }
      if(!allowed)
      {
        c15ReceiverRoute.invalidate();
        LookActive({.withBall=true});Stand();
        if(theBehaviorParameters.c15ReceiverFixedDirect)
        {
          const unsigned mask=(!(ownKickoff||playing||ready||set)||theGameState.stopped||theGameControllerData.stopped?1u:0u)
              |(observationHeld?2u:0u)|((theGameState.playerState!=GameState::active||WhiteFleetBoosterPickupControl::held()||theBehaviorStateInformation.remoteControlActive)?4u:0u)
              |((theSystemSensorData.emergencyMode||theEmergencyButtonState.buttonPressed)?8u:0u)|(intercept?16u:0u)
              |(((theFallDownState.state!=FallDownState::upright&&!receiverContact)||(theMotionInfo.executedPhase!=MotionPhase::stand&&theMotionInfo.executedPhase!=MotionPhase::walk))?32u:0u)
              |((theRobotPose.quality==RobotPose::poor||!theRobotPose.isFinite())?64u:0u)
              |((!theSkillRequest.kickoffThroughTarget.allFinite()||!theSkillRequest.kickoffThroughBodyTarget.allFinite()||!std::isfinite(theSkillRequest.kickoffThroughLength)
                  ||(theSkillRequest.kickoffThroughBallKnown&&!theSkillRequest.kickoffThroughBall.allFinite()))?128u:0u);
          fixedGuardLog("parent_guard_rejected",mask?mask:256u);
        }
        return;
      }
      if(set)
      {LookActive({.withBall=true});Stand();fixedGuardLog("gc_set",1u);return;}

      // Physical adaptation of4a2's handoff only. The far segment below still
      // uses the real native planner, arm/obstacle/area and observation guards.
      // No fixed-point/full-speed certificate can enter physical feet.
      if(physicalFlow&&playing&&stageAction==7)
      {
        c15ReceiverRoute.begin(theSkillRequest.kickoffThroughEvent);
        const bool nativeFresh=freshFootBall&&theFieldBall.ballWasSeen(250)
            &&theBallModel.timeWhenLastSeen>theSkillRequest.kickoffThroughEvent
            &&!observationHeld&&theFieldDimensions.isInsideField(theRobotPose.translation)
            &&theFieldDimensions.isInsideField(theFieldBall.positionOnField)
            &&!theIllegalAreas.isPositionIllegal(theFieldBall.positionOnField,150.f)
            &&theSkillRequest.kickoffThroughPublish&&!theSkillRequest.kickoffThroughRetired
            &&!theSkillRequest.isolatedKickoffFinished&&theSkillRequest.kickoffThroughHasLaunch
            &&theSkillRequest.kickoffThroughLaunch.allFinite();
        if(nativeFresh&&theBallModel.estimate.position.norm()<=1000.f)c15ReceiverRoute.near=true;
        if(c15ReceiverRoute.near)
        {
          theMotionRequest.fixedRunDriveEvent=theMotionRequest.fixedRunDriveFrame=0;
          theMotionRequest.fixedRunDriveSpeed=0.f;theMotionRequest.fixedRunDriveCoordinateVersion=0;
          if(nativeFresh)KickAtGoal();
          else {LookActive({.withBall=true});Stand();}
          if(theSkillRequest.kickoffThroughPublish&&!theSkillRequest.kickoffThroughRetired)
            PassTarget({.passTarget=-1,.ballTarget=theRobotPose.inverse()*theSkillRequest.kickoffThroughTarget});
          return; // One-way near handling, never a return to a blind far point.
        }
      }

      // Existing wire fields only. Receiver echoes the same global endpoint
      // with passTarget=-1; it does not announce a second pass back to sender.
      if(theSkillRequest.kickoffThroughPublish)
        PassTarget({.passTarget=kicker?theSkillRequest.kickoffThroughReceiver:-1,
            .ballTarget=theRobotPose.inverse()*theSkillRequest.kickoffThroughTarget});

      const bool gazeKnown=ownBall||theSkillRequest.kickoffThroughBallKnown;
      const Vector2f gaze=ownBall?theFieldBall.positionRelative:
          (theRobotPose.inverse()*theSkillRequest.kickoffThroughBall);
      if(profile.firstPassPrepareAdvance==250.f&&receiver&&stageAction==10)
      {LookActive({.withBall=true});Stand();return;} // Legacy gap executor is unreachable in exclusive mode.
      if(stageAction==10)
      {
        // Walking-only visual-gap witness. Never retain a prior MotionRequest
        // or call a ball/kick skill with this older observation.
        const unsigned age=theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
        if(!profile.receiverGapWalk||!receiver||!playing||!ownBall||age<=250u||age>450u
           ||!theSkillRequest.kickoffThroughGapValid
           ||theSkillRequest.kickoffThroughGapSeen!=theBallModel.timeWhenLastSeen
           ||theSkillRequest.kickoffThroughGapFrame!=theFrameInfo.time
           ||!theSkillRequest.kickoffThroughGapBallOdometry.allFinite()
           ||!theSkillRequest.kickoffThroughGapTargetOdometry.allFinite()||!theOdometryData.isFinite())
        {LookActive({.withBall=true});Stand();return;}
        const Vector2f snapshotBall=theOdometryData.inverse()*theSkillRequest.kickoffThroughGapBallOdometry;
        const Vector2f relativeWay=theOdometryData.inverse()*theSkillRequest.kickoffThroughGapTargetOdometry;
        const Vector2f point=theRobotPose*relativeWay;
        if(!snapshotBall.allFinite()||!relativeWay.allFinite()||!point.allFinite()
           ||snapshotBall.norm()<2000.f||theBallModel.estimate.position.norm()<2000.f
           ||relativeWay.norm()<=1.f||relativeWay.norm()>1200.001f||relativeWay.dot(snapshotBall)<=0.f
           ||(point-theSkillRequest.kickoffThroughBodyTarget).norm()>1.f
           ||point.x()<field.ownGoal+300.f||point.x()>field.opponentGoal-300.f
           ||std::abs(point.y())>field.halfWidth-300.f
           ||!theFieldDimensions.isInsideField(theFieldBall.positionOnField)
           ||theIllegalAreas.isPositionIllegal(point,150.f)
           ||theIllegalAreas.isPositionIllegal(theRobotPose.translation,150.f))
        {LookActive({.withBall=true});Stand();return;}
        // BallModel is already predicted to the current frame. Use it only
        // for reacquisition gaze, never extrapolate it a second time or turn
        // this older observation into walking/kicking permission.
        Vector2f headBall=snapshotBall;
        const Vector2f& predictedBall=theBallModel.estimate.position;
        if(predictedBall.allFinite()&&predictedBall.x()>0.f
           &&predictedBall.dot(snapshotBall)>0.f&&(predictedBall-snapshotBall).norm()<=1000.f)
          headBall=predictedBall;
        LookAtPoint({.target={headBall.x(),headBall.y(),theBallSpecification.radius}});
        // Replan around current native obstacles; a nonempty path is expected,
        // not permission to fall back to blind straight motion or kicking.
        WalkToPointObstacle({.target={snapshotBall.angle(),relativeWay},.speed={1.f,1.f,1.f},
            .reduceWalkSpeedType=ReduceWalkSpeedType::noChange,.disableAligning=true,.targetOfInterest={}});
        return;
      }
      if(profile.centerFast&&receiver&&!ownBall&&gazeKnown&&!fixedCarry)
      {
        // A fresh, already validated peer ball can direct the HEAD to reacquire.
        // It never makes our own observation fresh or grants blind locomotion.
        LookAtPoint({.target={gaze.x(),gaze.y(),theBallSpecification.radius}});Stand();return;
      }
      if(stageAction==4&&true&&ownBall&&!freshFootBall
         &&theFieldBall.positionRelative.norm()<=650.f)
      {
        // A stale close ball is not permission to sweep the feet while
        // turning toward the old P. Keep the echo; Strategy owns the fixed wait.
        LookAtPoint({.target={gaze.x(),gaze.y(),theBallSpecification.radius}});Stand();return;
      }
      if(stageAction==3)
      {
        const Vector2f ray=theSkillRequest.kickoffThroughTarget-theFieldBall.positionOnField;
        const float maximum=theKickInfo[KickInfo::rlKickDynamic].range.max;
        if(!kicker || !(ownKickoff||playing) || !freshFootBall || !ray.allFinite() || ray.norm()<500.f
           || theSkillRequest.kickoffThroughLength<500.f || !std::isfinite(maximum)
           || theSkillRequest.kickoffThroughLength>maximum)
        {LookActive({.withBall=true});Stand();return;}
        // Recheck in FIELD coordinates at the real foot entry. A localization
        // correction or changed ball must never turn the locked pass backwards.
        if(true&&!WhiteFleetC9ForwardFlank::directed(
            {theFieldBall.positionOnField.x(),theFieldBall.positionOnField.y()},
            {theSkillRequest.kickoffThroughTarget.x(),theSkillRequest.kickoffThroughTarget.y()},
            profile.directCenter?profile.side:theSkillRequest.kickoffThroughTarget.y()<0.f?-1:1,field,profile))
        {LookActive({.withBall=true});Stand();return;}
        // Native GoToBallAndKick supplies head-before-motion and obstacle
        // handling. Use the existing normal network, never soft-cap a 7 m pass.
        Angle direction=Angle::normalize(ray.angle()-theRobotPose.rotation);
        if(profile.stableKickAim)
        {
          if(!theSkillRequest.kickoffThroughAimValid||theSkillRequest.kickoffThroughAimFrame!=theFrameInfo.time
             ||!theSkillRequest.kickoffThroughAimTargetOdometry.allFinite()||!theOdometryData.isFinite())
          {LookActive({.withBall=true});Stand();return;}
          const Vector2f localTarget=theOdometryData.inverse()*theSkillRequest.kickoffThroughAimTargetOdometry;
          const Vector2f localRay=localTarget-theFieldBall.positionRelative;
          if(!localRay.allFinite()||localRay.norm()<500.f)
          {LookActive({.withBall=true});Stand();return;}
          direction=localRay.angle(); // Motion still performs its original request-to-preview ODOM correction.
        }
        GoToBallAndKick({.targetDirection=direction,
            .kickType=KickInfo::rlKickDynamic,.alignPrecisely=profile.centerFast&&!profile.stableKickAim?KickPrecision::justHitTheBall:KickPrecision::normal,
            .length=theSkillRequest.kickoffThroughLength});
        return;
      }
      if(stageAction==6)
      {
        if(!receiver || !playing || !freshFootBall || theFieldBall.positionRelative.norm()>1200.f
           ||(adaptiveMeeting&&(theFieldBall.positionRelative.norm()>650.f
              ||(theFieldBall.positionOnField-theSkillRequest.kickoffThroughTarget).norm()>1500.f)))
        {LookActive({.withBall=true});Stand();return;}
        // Native goal evaluation / indirect-kick legality / dribble fallback
        // remains responsible for the next real action, not a forced goal ray.
        KickAtGoal();
        return;
      }
      if(stageAction==7)
      {
        if(theBehaviorParameters.c15ReceiverCoordinateRun&&(!theBehaviorParameters.c15ReceiverBoundedFastRun||!fixedSession))
        {LookActive({.withBall=false});Stand();fixedGuardLog("coordinate_run_profile_missing",16384u);return;}
        if(theBehaviorParameters.c15ReceiverBoundedFastRun&&!fixedSession)
        {LookActive({.withBall=false});Stand();fixedGuardLog("bounded_run_profile_missing",8192u);return;}
        if(fixedArrivalMode&&!fixedSession)
        {LookActive({.withBall=false});Stand();fixedGuardLog("fixed_arrival_session_missing",4096u);return;}
        if(theBehaviorParameters.c15ReceiverFixedDirect&&(!theBehaviorParameters.c15EarlyChannelReceive
            ||!theBehaviorParameters.c15ReceiverLocalStep||theBehaviorParameters.c15ReceiverLowerChannel
            ||theBehaviorParameters.c15ReceiverShortestIntercept||profile.firstPassPrepareAdvance!=250.f))
        {LookActive({.withBall=true});Stand();fixedGuardLog("invalid_exclusive_profile",512u);return;}
        if(theBehaviorParameters.c15ReceiverShortestIntercept&&(!theBehaviorParameters.c15EarlyChannelReceive
            ||!theBehaviorParameters.c15ReceiverLocalStep||theBehaviorParameters.c15ReceiverLowerChannel||theBehaviorParameters.c15ReceiverFixedDirect||profile.firstPassPrepareAdvance!=250.f))
        {
          c15EarlyChannel.begin(theSkillRequest.kickoffThroughEvent);
          if(c15EarlyChannel.logs++<64u)std::printf("C15_SHORTEST event=%u frame=%u reason=invalid_exclusive_profile\n",
              theSkillRequest.kickoffThroughEvent,theFrameInfo.time);
          LookActive({.withBall=true});Stand();return;
        }
        // Keep the dedicated receiver, reuse the original continuous executor.
        // No fixed-P arrival, separate body-yaw throttle or fabricated capture.
        if(!(profile.nativeReceive||profile.receiverEarlyNative)||!receiver||!playing||(!ownBall&&!fixedCarry)
           ||(!freshFootBall&&profile.firstPassPrepareAdvance!=250.f)
           ||(!fixedCarry&&(!theFieldDimensions.isInsideField(theFieldBall.positionOnField)
           ||theIllegalAreas.isPositionIllegal(theFieldBall.positionOnField,150.f))))
        {LookActive({.withBall=true});Stand();fixedGuardLog(!playing?"not_playing":!ownBall?"ball_observation_invalid_or_over650":"receiver_ball_context",1024u);return;}
        if(theBehaviorParameters.c15EarlyChannelReceive&&profile.firstPassPrepareAdvance==250.f)
        {
          namespace E=WhiteFleetC15EarlyChannel;
          E::Input input;input.now=theFrameInfo.time;input.event=theSkillRequest.kickoffThroughEvent;
          input.seen=theBallModel.timeWhenLastSeen;input.params=profile;input.field=field;
          input.localStep=theBehaviorParameters.c15ReceiverLocalStep;
          input.fixedDirect=theBehaviorParameters.c15ReceiverFixedDirect;
          input.fixedRunToArrival=fixedArrivalMode;
          input.boundedFastRun=theBehaviorParameters.c15ReceiverBoundedFastRun;
          input.fixedOwnFresh=freshFootBall&&!observationHeld&&theFieldDimensions.isInsideField(theFieldBall.positionOnField)
              &&!theIllegalAreas.isPositionIllegal(theFieldBall.positionOnField,150.f);
          input.fixedContextFrame=theSkillRequest.kickoffFixedRunFrame;input.fixedContextRelease=theSkillRequest.kickoffFixedRunRelease;
          input.fixedContextKicker=theSkillRequest.kickoffThroughKicker;input.fixedContextReceiver=theSkillRequest.kickoffThroughReceiver;
          if(input.fixedDirect)input.fixedBall={theBehaviorParameters.c15FixedReceiveBallX,theBehaviorParameters.c15FixedReceiveBallY};
          input.shortest=theBehaviorParameters.c15ReceiverShortestIntercept;
          if(input.shortest)
          {
            input.preferredMinX=theBehaviorParameters.c15ShortestReceiveMinX;input.preferredMaxX=theBehaviorParameters.c15ShortestReceiveMaxX;
            input.preferredMinY=theBehaviorParameters.c15ShortestReceiveMinY;input.preferredMaxY=theBehaviorParameters.c15ShortestReceiveMaxY;
          }
          input.lowerChannel=input.localStep&&theBehaviorParameters.c15ReceiverLowerChannel;
          input.robot={theRobotPose.translation.x(),theRobotPose.translation.y()};input.heading=theRobotPose.rotation;
          input.odometry={theOdometryData.translation.x(),theOdometryData.translation.y()};input.odoHeading=theOdometryData.rotation;
          input.relativeBall={theBallModel.estimate.position.x(),theBallModel.estimate.position.y()};
          const Vector2f ball=theRobotPose*theBallModel.estimate.position;
          input.ball={ball.x(),ball.y()};input.velocity=E::rotate(
              {theBallModel.estimate.velocity.x(),theBallModel.estimate.velocity.y()},input.heading);
          c15EarlyChannel.begin(input.event);
          input.fixedReleaseWitness=input.fixedOwnFresh&&input.seen>input.event
              &&(ball-theSkillRequest.kickoffThroughLaunch).norm()>300.f;
          if(!theSkillRequest.kickoffThroughHasLaunch||!theSkillRequest.kickoffThroughLaunch.allFinite()
             ||!theSkillRequest.kickoffThroughPublish||((fixedArrivalMode?
                c15EarlyChannel.fixedArrivalPhase==E::FixedArrivalPhase::unarmed:!c15EarlyChannel.lastFrame)&&
                (!freshFootBall||input.seen<=input.event||(ball-theSkillRequest.kickoffThroughLaunch).norm()<=300.f)))
          {
            if(c15EarlyChannel.logs++<64u)std::printf("C15_EARLY_CHANNEL event=%u frame=%u reason=release_witness_missing\n",input.event,input.now);
            LookActive({.withBall=true});Stand();fixedGuardLog("release_witness_missing",2048u);return;
          }
          input.contract={theSkillRequest.kickoffThroughTarget.x(),theSkillRequest.kickoffThroughTarget.y()};
          input.maxForward=theWalkingEngineOutput.maxSpeed.translation.x();input.maxSide=theWalkingEngineOutput.maxSpeed.translation.y();
          input.maxYaw=theWalkingEngineOutput.maxSpeed.rotation;
          input.positionSigma=std::sqrt(std::max(theRobotPose.covariance(0,0),theRobotPose.covariance(1,1)));
          input.headingSigma=std::sqrt(theRobotPose.covariance(2,2));
          input.estimatedSpeed=theMotionInfo.speed.translation.allFinite()?theMotionInfo.speed.translation.norm():0.f;
          input.estimatedSpeedKnown=theMotionInfo.speed.translation.allFinite();
          input.unknownOpponents=std::max(0,theGlobalOpponentsModel.numOfUnknownOpponents);
          if(c15EarlyChannel.lastFrame&&input.now>c15EarlyChannel.lastFrame&&input.now-c15EarlyChannel.lastFrame<=250u)
          {
            input.estimatedSpeed=E::distance(input.odometry,c15EarlyChannel.previousOdo)*1000.f/(input.now-c15EarlyChannel.lastFrame);
            input.estimatedSpeedKnown=true;
          }
          for(const auto& o:theObstacleModel.obstacles)
          {
            const Vector2f center=theRobotPose*o.center;
            const float radius=std::max(600.f,(o.left-o.right).norm()*.5f+300.f);
            const float sigma=std::sqrt(std::max(o.covariance(0,0),o.covariance(1,1)));
            input.obstacles.push_back({{center.x(),center.y()},{},radius,3.f*sigma,2200.f,o.lastSeen,false,true,!o.isTeammate(),input.lowerChannel&&o.isOpponent()});
          }
          // The native global model is centre-only and explicitly unsuitable
          // for collision avoidance. Use unmatched centres for tactical scoring
          // only; local native obstacles remain the physical routing authority.
          for(const auto& o:theGlobalOpponentsModel.opponents)
          {
            bool local=false;for(const auto& p:input.obstacles)
              if(E::distance(p.center,{o.position.x(),o.position.y()})<=600.f)local=true;
            if(!local)input.obstacles.push_back({{o.position.x(),o.position.y()},{},600.f,650.f,2200.f,0,false,false,true});
          }
          const auto legal=[&](E::Point point){const Vector2f p(point.x,point.y);
            return theFieldDimensions.isInsideField(p)&&!theIllegalAreas.isPositionIllegal(p,150.f);};
          MotionRequest::ObstacleAvoidance checkedFixedAvoidance;
          Vector2f checkedFixedTarget;bool checkedFixed=false;
          const auto native=[&](E::Point target)
          {
            const Vector2f p(target.x,target.y),local=theRobotPose.inverse()*p;
            const float targetHeading=input.fixedDirect?std::atan2(p.y()-input.robot.y,p.x()-input.robot.x):std::atan2(-input.ball.y,field.opponentGoal-input.ball.x);
            const auto path=thePathPlanner.plan({targetHeading,p},{1.f,1.f,1.f},fixedArrivalMode&&!input.fixedOwnFresh);
            if(path.path.empty())for(const auto& o:theObstacleModel.obstacles)
              if(WhiteFleetKickoffThroughV8::pointSegmentDistance({o.center.x(),o.center.y()},{},{local.x(),local.y()})<600.f)return E::NativePath{};
            std::vector<E::Circle> circles;for(const auto& s:path.path)
              circles.push_back({{s.obstacle.center.x(),s.obstacle.center.y()},s.obstacle.radius,s.clockwise});
            auto result=E::nativePath({local.x(),local.y()},circles);
            if(input.fixedDirect)result.redirected=!path.path.empty()||!path.avoidance.allFinite()||path.avoidance.squaredNorm()>1e-6f;
            if(fixedArrivalMode&&input.fixedDirect&&!result.redirected&&result.valid)
            {checkedFixedAvoidance=path;checkedFixedTarget=local;checkedFixed=true;}
            for(auto& sample:result.samples){const Vector2f p=theRobotPose*Vector2f(sample.x,sample.y);sample={p.x(),p.y()};}
            return result;
          };
          const auto predict=[&](float t){const Vector2f p=theRobotPose*BallPhysics::propagateBallPosition(
              theBallModel.estimate.position,theBallModel.estimate.velocity,t,theBallSpecification);return E::Point{p.x(),p.y()};};
          const auto command=E::update(c15EarlyChannel,input,predict,native,legal);
          const auto logDecision=[&](const char* reason)
          {
            if(c15EarlyChannel.logs>=64u||(std::strcmp(reason,c15EarlyChannel.lastLogReason)==0
                &&input.now-c15EarlyChannel.lastLog<250u))return;
            ++c15EarlyChannel.logs;c15EarlyChannel.lastLog=input.now;c15EarlyChannel.lastLogReason=reason;
            std::printf("C15_EARLY_CHANNEL event=%u frame=%u seen=%u planned_mode=%u actual_motion=%u reason=%s side=%d segment=%u generation=%u own_est_mm_s=%.1f speed_known=%d eta=%.3f opp_lower_eta=%.3f unknown=%d clearance=%.1f soft_clearance=%.1f target=(%.1f,%.1f) native_queries=%u samples=%u candidates=%u native_rejects=%u occupied_rejects=%u illegal_rejects=%u budget_rejects=%u rejected_anchor_mm=%.1f heading_error_rad=%.4f\n",
                input.event,input.now,input.seen,static_cast<unsigned>(command.kind),static_cast<unsigned>(theMotionRequest.motion),reason,command.plan.side,
                c15EarlyChannel.segment,c15EarlyChannel.generation,input.estimatedSpeed,input.estimatedSpeedKnown,command.plan.eta,command.plan.opponentEta,
                command.plan.uncertain,command.plan.clearance,command.plan.softClearance,command.target.x,command.target.y,
                c15EarlyChannel.nativeQueries,command.plan.samplesTested,command.plan.routesTested,command.plan.nativeRejected,
                command.plan.occupiedRejected,command.plan.illegalRejected,command.plan.budgetRejected,c15EarlyChannel.anchorError,c15EarlyChannel.headingError);
            if(input.localStep)std::printf("C15_LOCAL_STEP event=%u frame=%u first_leg_eta=%.3f full_route_eta=%.3f future_clearance=%.1f local_only=%d\n",
                input.event,input.now,command.plan.firstLegEta,command.plan.eta,command.plan.futureClearance,command.plan.localStepOnly);
            if(input.lowerChannel&&c15EarlyChannel.lowerCommitted)
            {
              const auto defender=E::toField(c15EarlyChannel.lowerDefenderOdo,input);
              const auto entry=E::toField(c15EarlyChannel.lowerEntryOdo,input),exit=E::toField(c15EarlyChannel.lowerExitOdo,input);
              std::printf("C15_LOWER_CHANNEL event=%u frame=%u assigned=%d defender=(%.1f,%.1f) defender_seen=%u ball=(%.1f,%.1f) robot=(%.1f,%.1f) entry=(%.1f,%.1f) exit=(%.1f,%.1f) lane=%u changes=%u cut_at=%u route_reason=%s final_motion=%u requested_fraction=(1,1,1) reduce=noChange native_max=(%.1f,%.1f,%.3f) lower_rejects=%u\n",
                  input.event,input.now,c15EarlyChannel.lowerCommitted,defender.x,defender.y,c15EarlyChannel.lowerSeen,
                  input.ball.x,input.ball.y,input.robot.x,input.robot.y,entry.x,entry.y,exit.x,exit.y,c15EarlyChannel.lowerLane,
                  c15EarlyChannel.lowerChanges,c15EarlyChannel.lowerCutAt,c15EarlyChannel.lowerReason,
                  static_cast<unsigned>(theMotionRequest.motion),input.maxForward,input.maxSide,input.maxYaw,command.plan.lowerRejected);
            }
            else if(input.lowerChannel)std::printf("C15_LOWER_CHANNEL event=%u frame=%u assigned=0 reason=no_associated_local_defender final_motion=%u\n",
                input.event,input.now,static_cast<unsigned>(theMotionRequest.motion));
            if(input.shortest)std::printf("C15_SHORTEST event=%u frame=%u selected=%d forecast_candidate=%d reason=%s remote_reason=%s goal_ball=(%.1f,%.1f) body_target=(%.1f,%.1f) path_mm=%.1f full_eta=%.3f first_eta=%.3f ball_t=%.3f preferred=%d area_available=%d prefix_only=%d prefix_seen=%u heading_rad=%.4f actual_motion=%u requested_fraction=(1,1,1) reduce=noChange native_max=(%.1f,%.1f,%.3f)\n",
                input.event,input.now,command.plan.shortest,command.plan.shortestHasGoal,command.reason,c15EarlyChannel.shortestReason,command.plan.ball.x,command.plan.ball.y,
                command.target.x,command.target.y,command.plan.shortestLength,command.plan.shortestEta,command.plan.firstLegEta,
                command.plan.ballTime,command.plan.shortestPreferred,c15EarlyChannel.shortestAreaAvailable,command.plan.localStepOnly,
                c15EarlyChannel.shortestPrefixSeen,command.heading,static_cast<unsigned>(theMotionRequest.motion),input.maxForward,input.maxSide,input.maxYaw);
            if(input.fixedDirect)std::printf("C15_FIXED_DIRECT event=%u frame=%u far=%d started=%d planned_walk=%d reason=%s fixed_ball=(%.1f,%.1f) fixed_body=(%.1f,%.1f) robot=(%.1f,%.1f) segment=(%.1f,%.1f) initial_dir=%.4f request_dir=%.4f ball_age=%u first_eta=%.3f actual_motion=%u configured_fraction=(1,1,1) reduce=noChange native_max=(%.1f,%.1f,%.3f)\n",
                input.event,input.now,command.plan.fixedDirect,c15EarlyChannel.fixedReferenceKnown,command.kind==E::Kind::walk,reason,input.fixedBall.x,input.fixedBall.y,input.fixedBall.x-650.f,input.fixedBall.y,
                input.robot.x,input.robot.y,command.target.x,command.target.y,c15EarlyChannel.fixedInitialDirection,command.heading,input.now-input.seen,
                command.plan.firstLegEta,static_cast<unsigned>(theMotionRequest.motion),input.maxForward,input.maxSide,input.maxYaw);
            if(fixedArrivalMode)std::printf("C15_FIXED_ARRIVAL event=%u frame=%u phase=%u release=%u authorized=%u auth_seen=%u arrived=%u new_own_seen=%u first_run=%u first_near=%u reason=%s final_motion=%u\n",
                input.event,input.now,static_cast<unsigned>(c15EarlyChannel.fixedArrivalPhase),c15EarlyChannel.fixedSessionRelease,
                c15EarlyChannel.fixedAuthorizedAt,c15EarlyChannel.fixedAuthSeen,c15EarlyChannel.fixedArrivedAt,
                c15EarlyChannel.fixedFreshAfterArrival,c15EarlyChannel.fixedFirstRunOutput,c15EarlyChannel.fixedFirstNearOutput,
                reason,static_cast<unsigned>(theMotionRequest.motion));
            if(input.boundedFastRun)std::printf("C15_FIXED_DRIVE event=%u frame=%u certified=%u requested_mm_s=%.1f endpoint_mm=%.1f lifetime_ms=100 max_phase_ms=20 reason=%s final_motion=%u\n",
                input.event,input.now,theMotionRequest.fixedRunDriveFrame,theMotionRequest.fixedRunDriveSpeed,
                theMotionRequest.walkTarget.translation.norm(),reason,static_cast<unsigned>(theMotionRequest.motion));
          };
          if(command.kind==E::Kind::shoot)
          {
            KickAtGoal();
            if(fixedArrivalMode&&!c15EarlyChannel.fixedFirstNearOutput
               &&(theMotionRequest.motion==MotionRequest::walkToBallAndKick||theMotionRequest.motion==MotionRequest::dribble))
              c15EarlyChannel.fixedFirstNearOutput=input.now;
            logDecision(command.reason);return;
          }
          if(command.kind!=E::Kind::walk){LookActive({.withBall=!fixedArrivalMode});Stand();logDecision(command.reason);return;}
          const Vector2f local=theRobotPose.inverse()*Vector2f(command.target.x,command.target.y);
          const bool fixedRun=fixedArrivalMode&&command.plan.fixedDirect&&c15EarlyChannel.fixedArrivalPhase==E::FixedArrivalPhase::run;
          if(fixedRun)
          {
            // Preserve original arm-contact protection, but stop rather than
            // its sideways detour. Original800ms trigger and3000ms hold.
            const unsigned right=theArmContactModel.status[Arms::right].timeOfLastContact,left=theArmContactModel.status[Arms::left].timeOfLastContact;
            const bool contactFuture=right>input.now||left>input.now;
            const bool contact=(right&&input.now-right<800u)||(left&&input.now-left<800u);
            if(contact&&!c15EarlyChannel.fixedArmContactAt)c15EarlyChannel.fixedArmContactAt=input.now;
            const bool hold=c15EarlyChannel.fixedArmContactAt&&input.now-c15EarlyChannel.fixedArmContactAt<=3000u;
            if(!hold&&!contact)c15EarlyChannel.fixedArmContactAt=0;
            if(contactFuture||contact||hold||!checkedFixed||(checkedFixedTarget-local).norm()>1.f)
            {LookActive({.withBall=false});Stand();logDecision(contactFuture?"fixed_arm_clock_fault":contact||hold?"fixed_arm_contact_hold":"fixed_native_proof_missing");return;}
            const Vector2f head=input.fixedOwnFresh?theBallModel.estimate.position:
                theRobotPose.inverse()*Vector2f(c15EarlyChannel.fixedReferenceBall.x,c15EarlyChannel.fixedReferenceBall.y);
            LookAtPoint({.target={head.x(),head.y(),theBallSpecification.radius}});
            // The exact checked native avoidance is passed unchanged. No
            // stale-ball LibWalk obstacle or unverified second planner runs.
            theMotionRequest.shouldInterceptBall=false; // Owned fixed navigation is not remembered-ball interception.
            PublishMotion({.target=local,.speed={1.f,1.f,1.f}});
            WalkToPose({.target={Angle::normalize(command.heading-theRobotPose.rotation),local},.speed={1.f,1.f,1.f},
                .obstacleAvoidance=checkedFixedAvoidance,.keepTargetRotation=true,.targetOfInterest={},.sideWalkingRequest=SideWalkingRequest::allowed});
          }
          else
          {
            LookAtPoint({.target={theBallModel.estimate.position.x(),theBallModel.estimate.position.y(),theBallSpecification.radius}});
            WalkToPointObstacle({.target={Angle::normalize(command.heading-theRobotPose.rotation),local},.speed={1.f,1.f,1.f},
                .reduceWalkSpeedType=ReduceWalkSpeedType::noChange,.disableAligning=true,.disableStanding=true,.targetOfInterest={}});
          }
          bool rejected=theMotionRequest.motion!=MotionRequest::walkToPose||!theMotionRequest.walkTarget.isFinite()
             ||!legal({(theRobotPose*theMotionRequest.walkTarget.translation).x(),(theRobotPose*theMotionRequest.walkTarget.translation).y()})
             ||(!freshFootBall&&theMotionRequest.walkTarget.translation.norm()>local.norm()+.001f);
          const char* outputReason="native_output_rejected";
          if(input.fixedDirect&&command.plan.fixedDirect&&!rejected)
          {
            const auto& actual=theMotionRequest.walkTarget.translation;const float length=local.norm();
            const float lateral=std::abs(actual.x()*local.y()-actual.y()*local.x())/std::max(1.f,length);
            const float projection=(actual.x()*local.x()+actual.y()*local.y())/std::max(1.f,length);
            if(!theMotionRequest.obstacleAvoidance.avoidance.allFinite()||theMotionRequest.obstacleAvoidance.avoidance.squaredNorm()>1e-6f
               ||!theMotionRequest.obstacleAvoidance.path.empty()||theMotionRequest.shouldInterceptBall||theMotionRequest.shouldWalkOutOfBallLine)
            {rejected=true;outputReason="fixed_output_avoidance_or_override";}
            else if(lateral>1.f||projection< -1.f||projection>length+1.f
                ||std::abs(Angle::normalize(theMotionRequest.walkTarget.rotation-(command.heading-theRobotPose.rotation)))>.0001f)
            {rejected=true;outputReason="fixed_output_not_direct";}
          }
          if(rejected)Stand();
          else if(fixedRun&&!c15EarlyChannel.fixedFirstRunOutput)c15EarlyChannel.fixedFirstRunOutput=input.now;
          if(!rejected&&fixedRun&&input.boundedFastRun)
          {
            theMotionRequest.fixedRunDriveEvent=input.event;theMotionRequest.fixedRunDriveFrame=input.now;
            theMotionRequest.fixedRunDriveSpeed=std::min(2000.f,input.maxForward);
            theMotionRequest.fixedRunDriveCoordinateVersion=theBehaviorParameters.c15ReceiverCoordinateRun?1u:0u;
          }
          logDecision(rejected?outputReason:command.reason);
          return;
        }
        if(profile.firstPassPrepareAdvance==250.f)
        {
          c15ReceiverRoute.begin(theSkillRequest.kickoffThroughEvent);
          const Vector2f& currentBall=theBallModel.estimate.position;
          const unsigned age=theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen);
          if(age>450u)
          {LookActive({.withBall=true});Stand();return;} // Safety stop, NEVER an old strategy fallback.
          // One-way handoff before reaching the planned point. A fresh near
          // ball never waits for the waypoint or for its predicted stop.
          if(freshFootBall&&currentBall.norm()<=1500.f)c15ReceiverRoute.near=true;
          if(c15ReceiverRoute.near)
          {
            if(freshFootBall)KickAtGoal(); // Intended final approach/shot, once genuinely near.
            else {LookActive({.withBall=true});Stand();}
            return;
          }
          if(!c15ReceiverRoute.near&&theOdometryData.isFinite()
             &&theBallModel.estimate.velocity.allFinite()
             &&theSkillRequest.kickoffThroughHasLaunch)
          {
            const unsigned seen=theBallModel.timeWhenLastSeen;
            const auto coherentRoute=[&](){return c15ReceiverRoute.coherent(theFrameInfo.time,
                {theOdometryData.translation.x(),theOdometryData.translation.y()},theOdometryData.rotation,
                {theRobotPose.translation.x(),theRobotPose.translation.y()},theRobotPose.rotation,
                theWalkingEngineOutput.maxSpeed.translation.norm(),theWalkingEngineOutput.maxSpeed.rotation);};
            // Validate the OLD anchor before a new observation may replan.
            // Otherwise an odometry jump itself could mint a new trusted epoch.
            if(c15ReceiverRoute.valid&&!coherentRoute())
            {
              c15ReceiverRoute.invalidate();c15ReceiverRoute.plannedAt=theFrameInfo.time;
              c15ReceiverRoute.seen=seen;
            }
            const auto assess=[&](WhiteFleetKickoffThroughV8::Point body)
            {
              const Vector2f point(body.x,body.y);
              if(!theFieldDimensions.isInsideField(point)||theIllegalAreas.isPositionIllegal(point,150.f))
                return WhiteFleetC15ReceiverRoute::Route{};
              const Vector2f relative=theRobotPose.inverse()*point;
              const auto route=thePathPlanner.plan({Angle::normalize(theRobotPose.rotation+relative.angle()),point},{1.f,1.f,1.f},false);
              // Native plan() has no success flag: an empty route can also
              // mean no path. Never count a visibly blocked fallback as clear.
              if(route.path.empty())
                for(const auto& o:theObstacleModel.obstacles)
                  if(WhiteFleetKickoffThroughV8::pointSegmentDistance({o.center.x(),o.center.y()},{},
                      {relative.x(),relative.y()})<600.f)return WhiteFleetC15ReceiverRoute::Route{};
              std::vector<WhiteFleetC15ReceiverRoute::Circle> circles;
              for(const auto& s:route.path)
                circles.push_back({{s.obstacle.center.x(),s.obstacle.center.y()},s.obstacle.radius,s.clockwise});
              return WhiteFleetC15ReceiverRoute::measure({relative.x(),relative.y()},circles);
            };
            // Only a genuinely new sight may create or revise a destination.
            // Re-reading a refreshed model never extends its observation age.
            if(freshFootBall&&seen>c15ReceiverRoute.seen)
            {
              const Vector2f odoBall=theOdometryData*currentBall;
              const float future=c15ReceiverRoute.arrivesAt>=theFrameInfo.time?
                  (c15ReceiverRoute.arrivesAt-theFrameInfo.time)*.001f:0.f;
              const Vector2f expected=theOdometryData*BallPhysics::propagateBallPosition(currentBall,
                  theBallModel.estimate.velocity,future,theBallSpecification);
              const bool moved=c15ReceiverRoute.valid&&
                  (odoBall-Vector2f(c15ReceiverRoute.ballOdometry.x,c15ReceiverRoute.ballOdometry.y)).norm()>650.f;
              const bool changed=c15ReceiverRoute.valid&&
                  (!expected.allFinite()
                   ||(expected-Vector2f(c15ReceiverRoute.expectedBallOdometry.x,c15ReceiverRoute.expectedBallOdometry.y)).norm()>650.f);
              if(!c15ReceiverRoute.plannedAt||changed||moved||theFrameInfo.time<c15ReceiverRoute.plannedAt
                 ||theFrameInfo.time-c15ReceiverRoute.plannedAt>=250u)
              {
                // A failed new search does not revoke a still coherent route.
                // A genuinely changed trajectory does: never hold a known-wrong plan.
                if(changed)c15ReceiverRoute.invalidate();
                c15ReceiverRoute.plannedAt=theFrameInfo.time; // Failed searches are rate-limited too.
                const auto predict=[&](float t)
                {
                  // estimate.position is already current; forecast t, not age+t.
                  const Vector2f b=theRobotPose*BallPhysics::propagateBallPosition(currentBall,
                      theBallModel.estimate.velocity,t,theBallSpecification);
                  return WhiteFleetKickoffThroughV8::Point{b.x(),b.y()};
                };
                const float remaining=std::max(0.f,static_cast<float>(std::min(profile.total,profile.flight))-
                    static_cast<float>(theFrameInfo.time-theSkillRequest.kickoffThroughEvent))*.001f;
                const auto choice=WhiteFleetC15ReceiverRoute::choose({theRobotPose.translation.x(),theRobotPose.translation.y()},
                    {theSkillRequest.kickoffThroughTarget.x(),theSkillRequest.kickoffThroughTarget.y()},field,profile,
                    theWalkingEngineOutput.maxSpeed.translation.x(),theWalkingEngineOutput.maxSpeed.rotation,
                    remaining,predict,assess);
                if(choice.valid)
                {
                  const Vector2f local=theRobotPose.inverse()*Vector2f(choice.body.x,choice.body.y);
                  const Vector2f target=theOdometryData*local;
                  c15ReceiverRoute.targetOdometry={target.x(),target.y()};
                  const Vector2f futureBall=theOdometryData*(theRobotPose.inverse()*Vector2f(choice.ball.x,choice.ball.y));
                  c15ReceiverRoute.expectedBallOdometry={futureBall.x(),futureBall.y()};
                  c15ReceiverRoute.arrivesAt=theFrameInfo.time+static_cast<unsigned>(choice.ballTime*1000.f);
                  c15ReceiverRoute.originField={theRobotPose.translation.x(),theRobotPose.translation.y()};
                  c15ReceiverRoute.originHeading=theRobotPose.rotation;
                  c15ReceiverRoute.originOdometry={theOdometryData.translation.x(),theOdometryData.translation.y()};
                  c15ReceiverRoute.originOdoHeading=theOdometryData.rotation;
                  c15ReceiverRoute.lastOdometry=c15ReceiverRoute.originOdometry;
                  c15ReceiverRoute.lastOdoHeading=theOdometryData.rotation;c15ReceiverRoute.lastFrame=theFrameInfo.time;
                  c15ReceiverRoute.valid=true;c15ReceiverRoute.plannedAt=theFrameInfo.time;
                  if(c15ReceiverRoute.diagnostics++<16u)
                    std::printf("C15_RECEIVER_ROUTE event=%u frame=%u seen=%u target=(%.1f,%.1f) ball=(%.1f,%.1f) route_mm=%.1f eta=%.3f ball_time=%.3f pure_walk=1\n",
                        c15ReceiverRoute.event,theFrameInfo.time,seen,choice.body.x,choice.body.y,
                        choice.ball.x,choice.ball.y,choice.length,choice.eta,choice.ballTime);
                }
              }
              c15ReceiverRoute.seen=seen;c15ReceiverRoute.ballOdometry={odoBall.x(),odoBall.y()};
            }
            if(c15ReceiverRoute.valid)
            {
              Vector2f local=theOdometryData.inverse()*Vector2f(c15ReceiverRoute.targetOdometry.x,c15ReceiverRoute.targetOdometry.y);
              if(!freshFootBall)
              {
                if(c15ReceiverRoute.seen!=seen)
                {LookActive({.withBall=true});Stand();return;}
                const Vector2f snapshot=theOdometryData.inverse()*Vector2f(c15ReceiverRoute.ballOdometry.x,c15ReceiverRoute.ballOdometry.y);
                const auto step=WhiteFleetC15ReceiverRoute::boundedGapStep({local.x(),local.y()},
                    {snapshot.x(),snapshot.y()},{currentBall.x(),currentBall.y()});
                local={step.x,step.y};
              }
              const Vector2f point=theRobotPose*local;
              if(local.allFinite()&&point.allFinite()&&local.norm()>1.f
                 &&coherentRoute()
                 &&theFieldDimensions.isInsideField(point)&&!theIllegalAreas.isPositionIllegal(point,150.f)
                 &&!theIllegalAreas.isPositionIllegal(theRobotPose.translation,150.f))
              {
                c15ReceiverRoute.lastOdometry={theOdometryData.translation.x(),theOdometryData.translation.y()};
                c15ReceiverRoute.lastOdoHeading=theOdometryData.rotation;c15ReceiverRoute.lastFrame=theFrameInfo.time;
                LookAtPoint({.target={currentBall.x(),currentBall.y(),theBallSpecification.radius}});
                // Replans the selected route LAST; candidate queries cannot
                // leave PathPlanner's direction preference on another target.
                WalkToPointObstacle({.target={local.angle(),local},.speed={1.f,1.f,1.f},
                    .reduceWalkSpeedType=ReduceWalkSpeedType::noChange,.disableAligning=true,
                    .disableStanding=true,.targetOfInterest={}});
                // Native obstacle avoidance may shift an occupied waypoint.
                // During a stale-ball step, independently check its ACTUAL
                // outgoing target, not merely the requested bounded waypoint.
                if(!freshFootBall&&(theMotionRequest.motion!=MotionRequest::walkToPose
                    ||!theMotionRequest.walkTarget.isFinite()
                    ||theMotionRequest.walkTarget.translation.norm()>local.norm()+.001f
                    ||theIllegalAreas.isPositionIllegal(theRobotPose*theMotionRequest.walkTarget.translation,150.f)))
                  Stand();
                return; // Pure locomotion, not a retained/forecast-ball kick.
              }
              c15ReceiverRoute.invalidate();
            }
          }
          // Exclusive mode owns all far-segment outcomes, including safe
          // waiting. No fallthrough into KickAtGoal or legacy gap chasing.
          LookActive({.withBall=true});Stand();return;
        }
        // Fleet adapter deliberately retains the current native power/aim provider.
        KickAtGoal(); // Original indirect-kick, duel, obstacle and head handling.
        return;
      }
      if(stageAction==8&&(profile.centerFast||!WhiteFleetC15KickoffSwitch::throughParams().c14EarlyStart||!receiver||!ownKickoff||!freshFootBall))
      {LookActive({.withBall=true});Stand();return;}
      if(stageAction==9&&(!profile.legalPreload||!receiver||!ownKickoff||!freshFootBall))
      {LookActive({.withBall=true});Stand();return;}
      if(stageAction!=2 && stageAction!=4 && stageAction!=5 && stageAction!=8 && stageAction!=9)
      {
        if(gazeKnown)LookAtPoint({.target={gaze.x(),gaze.y(),theBallSpecification.radius}});
        else LookActive({.withBall=true});
        Stand();return;
      }
      if((stageAction==4 && (!receiver||!playing)) || (stageAction==5 && (!kicker||!playing))
         || (!ready && (!gazeKnown || !ownBall)))
      {LookActive({.withBall=true});Stand();return;}

      const Vector2f point=theSkillRequest.kickoffThroughBodyTarget;
      const auto approach=WhiteFleetKickoffThroughV8::receiverBallApproach(
          {theSkillRequest.kickoffThroughLaunch.x(),theSkillRequest.kickoffThroughLaunch.y()},
          {theSkillRequest.kickoffThroughTarget.x(),theSkillRequest.kickoffThroughTarget.y()},
          {theFieldBall.positionOnField.x(),theFieldBall.positionOnField.y()},
          {theRobotPose.translation.x(),theRobotPose.translation.y()},field,profile);
      const bool currentBallApproach=profile.receiverBallApproach&&receiver&&playing&&stageAction==4&&freshFootBall
          &&theSkillRequest.kickoffThroughHasLaunch&&approach.valid
          &&(!profile.receiverFollowFromRelease||
             ((theFieldBall.positionOnField-theSkillRequest.kickoffThroughLaunch).norm()>300.f
              &&theBallModel.timeWhenLastSeen>theSkillRequest.kickoffThroughEvent))
          &&(point-Vector2f(approach.body.x,approach.body.y)).norm()<=1.f;
      if(profile.receiverFollowFromRelease&&receiver&&stageAction==4&&!currentBallApproach)
      {
        // A fresh current-ball target is mandatory, even inside the old P
        // leash. A stale or retained fixed-P request must never move us blindly.
        if(gazeKnown)LookAtPoint({.target={gaze.x(),gaze.y(),theBallSpecification.radius}});
        else LookActive({.withBall=true});
        Stand();return;
      }
      const auto fieldPrepare=WhiteFleetKickoffThroughV8::preparePoint(field,profile,-1);
      // A live receiver preparation directive is the Session's native
      // stand_observe_native_release, not permission for another pose tweak.
      if((simFlow||physicalFlow)&&ownKickoff&&stageAction==2&&theSkillRequest.kickoffThroughPublish
         &&!theSkillRequest.kickoffThroughRetired&&theSkillRequest.kickoffThroughHasLaunch
         &&theSkillRequest.kickoffThroughLaunch.allFinite())
      {
        PublishMotion({.target=theRobotPose.inverse()*Vector2f(fieldPrepare.x,fieldPrepare.y),.speed={0.f,0.f,0.f}});
        if(gazeKnown)LookAtPoint({.target={gaze.x(),gaze.y(),theBallSpecification.radius}});
        else LookActive({.withBall=true});
        Stand();return; // The real GSP still owns all ball-motion/PLAY evidence.
      }
      if((stageAction==8||stageAction==9)&&(point.x()>(stageAction==9?-750.f:-350.f)||
          (point-Vector2f(fieldPrepare.x,fieldPrepare.y)).norm()>
            (stageAction==9?300.f:600.f)))
      {LookActive({.withBall=true});Stand();return;}
      if(stageAction==4&&adaptiveMeeting&&(point-theSkillRequest.kickoffThroughTarget).norm()>1200.f&&!currentBallApproach)
      {LookActive({.withBall=true});Stand();return;}
      // Never clear illegal-area bits. Even a stale/forged run request cannot
      // move the receiver over halfway before its native local release.
      const float legalHalfway=theFieldDimensions.xPosHalfwayLine-theFieldDimensions.fieldLinesWidth*.5f-200.f;
      if(((ownKickoff||ready) && point.x()>legalHalfway) || theIllegalAreas.isPositionIllegal(point,150.f))
      {LookActive({.withBall=true});Stand();return;}
      if((profile.raceRendezvous||profile.directCenter)&&receiver&&ownKickoff&&stageAction==2&&
         (theRobotPose.translation-Vector2f(fieldPrepare.x,fieldPrepare.y)).norm()<=
            WhiteFleetKickoffThroughV8::receiverPrepareHoldRadius(profile,theMotionInfo.executedPhase==MotionPhase::stand))
      {
        // Publish the real desired prepare point with ZERO motion. Actual
        // Stand (not walk-to-a-nearby-pose) lets native GSP collect its own
        // genuine ball-motion samples. It alone decides legal release.
        PublishMotion({.target=theRobotPose.inverse()*Vector2f(fieldPrepare.x,fieldPrepare.y),.speed={0.f,0.f,0.f}});
        if(gazeKnown)LookAtPoint({.target={gaze.x(),gaze.y(),theBallSpecification.radius}});
        else LookActive({.withBall=true});
        Stand();return;
      }
      Vector2f relative=theRobotPose.inverse()*point;
      const float pathAngle=relative.norm()>100.f?static_cast<float>(relative.angle()):0.f;
      Angle bodyAngle=pathAngle;
      // Ball tracking and body occupancy are separate: face the attacking
      // direction while moving to the live behind-ball pose, not past the ball
      // toward P. Native obstacle/illegal-area handling and speed stay intact.
      if(currentBallApproach)bodyAngle=Angle::normalize(approach.heading-theRobotPose.rotation);
      if((profile.raceRendezvous||profile.directCenter)&&ready&&stageAction==2)
      {
        const float desired=kicker?std::atan2(theSkillRequest.kickoffThroughTarget.y(),theSkillRequest.kickoffThroughTarget.x()):
          WhiteFleetC9ForwardFlank::runHeading({point.x(),point.y()},
              {theSkillRequest.kickoffThroughTarget.x(),theSkillRequest.kickoffThroughTarget.y()},-1,profile);
        bodyAngle=Angle::normalize(desired-theRobotPose.rotation);
      }
      if(true&&receiver&&ownKickoff&&stageAction==2&&relative.norm()<=600.f)
      {
        const auto p=profile;
        const int side=theSkillRequest.kickoffThroughTarget.y()<0.f?-1:1;
        const float heading=WhiteFleetC9ForwardFlank::runHeading({point.x(),point.y()},
            {theSkillRequest.kickoffThroughTarget.x(),theSkillRequest.kickoffThroughTarget.y()},side,p);
        bodyAngle=Angle::normalize(heading-theRobotPose.rotation);
        if(relative.norm()<=100.f)relative=Vector2f::Zero();
      }
      float speed=1.f;
      if(gazeKnown)
      {
        const float pan=std::max(0.f,std::min(static_cast<float>(theHeadLimits.maxPan()),
            -static_cast<float>(theHeadLimits.minPan()))-static_cast<float>(5_deg));
        if(!std::isfinite(pan) || pan<static_cast<float>(10_deg))
        {LookActive({.withBall=true});Stand();return;}
        const float ballFromPath=Angle::normalize(gaze.angle()-bodyAngle);
        const float excess=std::max(0.f,std::abs(ballFromPath)-pan);
        bodyAngle=Angle::normalize(bodyAngle+std::copysign(excess,ballFromPath));
        if(excess>0.f)speed=.5f;
      }
      if(std::abs(static_cast<float>(bodyAngle))>static_cast<float>(60_deg))
      {relative=Vector2f::Zero();speed=.35f;}
      // Independent native head request. Body direction is explicit, and no
      // targetOfInterest is passed down to silently turn it back to the ball.
      if(gazeKnown)LookAtPoint({.target={gaze.x(),gaze.y(),theBallSpecification.radius}});
      else LookActive({.withBall=true});
      // A reached-pose navigator intentionally swallows <10/20deg rotation.
      // At the legal own-half prepare point only, use its existing no-standing
      // interface for the small yaw that keeps a FRESH/recent own ball in view.
      // Zero translation, unchanged yaw/speed limits and native avoidance.
      const bool prepareViewTurn=true&&receiver&&ownKickoff&&stageAction==2
          &&ownBall&&relative.norm()<=100.f
          &&std::abs(static_cast<float>(bodyAngle))>static_cast<float>(1_deg)
          &&std::abs(static_cast<float>(bodyAngle))<=static_cast<float>(20_deg)
          &&!theIllegalAreas.isPositionIllegal(theRobotPose.translation,150.f);
      if(prepareViewTurn)relative=Vector2f::Zero();
      const bool prestartMoving=(stageAction==8||stageAction==9)&&relative.norm()>50.f;
      WalkToPointObstacle({.target={bodyAngle,relative},.speed={1.f,speed,speed},
          .reduceWalkSpeedType=ReduceWalkSpeedType::noChange,.disableAligning=true,.disableStanding=prepareViewTurn||prestartMoving,
          .targetOfInterest={}});
    }
  }
}
