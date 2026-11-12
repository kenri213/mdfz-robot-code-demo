#include "WhiteFleetSimEntry.h"
#include "WhiteFleetSimPhysical.h"
/**
 * @file WhiteFleetSkillBehaviorControl.cpp
 *
 * This file implements the behavior control for the skill layer.
 *
 * @author Arne Hasselbring
 */

#define FULL_HEADER
#include "WhiteFleetSkillBehaviorControl.h"
#include "Debugging/DebugDrawings3D.h"
#include "WhiteFleetBoosterAppCommissioning.h"
#include "WhiteFleetBoosterOfficialLiveTest.h"
#include "WhiteFleetK1HotChase.h"
#include "WhiteFleetMatchTacticalRepair.h"
#include "WhiteFleetC15KickoffSwitch.h"
#include "WhiteFleetAutonomousKickoff.h"
#include "Tools/BehaviorControl/TeamActivityPolicy.h"
#include "Platform/SystemCall.h"
#include "Streaming/TypeRegistry.h"
#include <string>

// 2026-11-10 robust kickoff: red2's enhanced receive run (landing run, poor-pose
// run, early shot) is allowed in the simulator for team 5 / player 2 and on the
// identity-bound guarded physical profile for player 2. Ignoring obstacles is
// also enabled on the guarded physical receiver (light contact accepted).
namespace
{
  inline bool red2EnhancedRunAllowed()
  {
    return (WhiteFleetSimPhysical::simulated()&&Global::getSettings().teamNumber==5&&
            Global::getSettings().playerNumber==2)||
           (WhiteFleetK1V21MatchProfile::guardedPhysicalReceiver()&&Global::getSettings().playerNumber==2);
  }
  // The kickoff run to the landing point ignores obstacles on the guarded
  // physical receiver too (light contact accepted, 2026-11-10 decision).
  inline bool red2IgnoreObstacles() {return red2EnhancedRunAllowed();}
}

MAKE_MODULE(WhiteFleetSkillBehaviorControl);

WhiteFleetSkillBehaviorControl::WhiteFleetSkillBehaviorControl() :
  Cabsl<WhiteFleetSkillBehaviorControl>(const_cast<ActivationGraph*>(&theActivationGraph))
{}

void WhiteFleetSkillBehaviorControl::update(ActivationGraph&)
{
  WhiteFleetSimPhysical::ensureProfile(); // SIM-only physical-path emulation

  if(std::getenv("MDFZ_MATCH_CONTACT_AUDIT_ENABLE") &&
     (theGameState.ownTeam.number==5 || theGameState.ownTeam.number==70) &&
     theGameState.playerNumber>=1 && theGameState.playerNumber<=3)
  {
    static thread_local bool goal3FieldWitness=false;
    if(!goal3FieldWitness)
    {
      std::printf("MATCH_FIELD_ACTUAL team=%u player=%u goal_line_mm=%.3f post_left_mm=%.3f post_right_mm=%.3f post_radius_mm=%.3f clear_goal_mm=%.3f\n",
        unsigned(theGameState.ownTeam.number),unsigned(theGameState.playerNumber),
        theFieldDimensions.xPosOpponentGoalLine,theFieldDimensions.yPosLeftGoal,
        theFieldDimensions.yPosRightGoal,theFieldDimensions.goalPostRadius,
        theFieldDimensions.yPosLeftGoal-theFieldDimensions.yPosRightGoal-2.f*theFieldDimensions.goalPostRadius);
      std::fflush(stdout);goal3FieldWitness=true;
    }
  }


  WhiteFleetSimEntry::activate();
  static thread_local bool fleetProviderSeen=false;
  if(!fleetProviderSeen){WhiteFleetSimEntry::witness("WhiteFleetSkillBehaviorControl");fleetProviderSeen=true;}
  DECLARE_DEBUG_RESPONSE("option:ManualCameraCalibration:finishNow");
  DECLARE_DEBUG_RESPONSE("option:ManualCameraCalibration:prepareRecording");
  DECLARE_DEBUG_DRAWING("option:ClearBall:bonus", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:ClearBall:borders", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:ClearBall:clearBall", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:ClearBall:evaluation", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:ClearBall:zero", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:DirectKickOff:wheel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:DribbleTargetProvider:wheel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:DribbleTargetProvider:direction", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:DribbleTargetProvider:step", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:HandleBallAtOwnGoalPost:blueprintWheel", "drawingOnField");
  DECLARE_DEBUG_RESPONSE("debug data:option:HandleBallAtOwnGoalPost:drawKickType");
  DECLARE_DEBUG_DRAWING("option:HandleBallAtOwnGoalPost:kickWheel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:KickAtGoal:goalLineIntersection", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:KickAtGoal:cullLine", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:KickAtGoal:goalSector", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:KickAtGoal:wheel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:KickAtGoal:primaryRange", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:Mark:markPose", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:PassToTeammate:target", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:PassToTeammate:precision", "drawingOnField");
  DECLARE_DEBUG_DRAWING3D("option:PassToTeammate:trajectory", "field");
  DECLARE_DEBUG_DRAWING("option:ReceivePass:target", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:WalkPotentialField:angles", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:WalkPotentialField:potentialField", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:WalkToPointObstacle:obstacleAtMyPosition", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:Zweikampf:wheel", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:Zweikampf:kickDirections", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:Zweikampf:kicks", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:Zweikampf:wheelSteal", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:Zweikampf:sidewardRange", "drawingOnField");
  DECLARE_DEBUG_DRAWING("option:Zweikampf:sideSteal", "drawingOnField");

  theBehaviorStateInformation.transitionFailed = false;
  theBehaviorStateInformation.remoteControlActive = false;

  theBehaviorStatus.calibrationFinished = false;
  theBehaviorStatus.passTarget = -1;
  theBehaviorStatus.passOrigin = -1;
  theBehaviorStatus.walkingTo = Vector2f::Zero();
  theBehaviorStatus.speed = 0.f;
  theBehaviorStatus.shootingTo.reset();
  theBehaviorStatus.restartPassAdmission=0;
  theBehaviorStatus.restartPassFlightMs=-1.f;
  theBehaviorStatus.restartPassEtaMs=-1.f;
  theBehaviorStatus.restartPassSafety=-1.f;

  theArmMotionRequest.armMotion[Arms::left] = ArmMotionRequest::none;
  theArmMotionRequest.armMotion[Arms::right] = ArmMotionRequest::none;

  theHeadMotionRequest.mode = HeadMotionRequest::panTiltMode;
  theHeadMotionRequest.pan = JointAngles::off;
  theHeadMotionRequest.tilt = JointAngles::off;
  theHeadMotionRequest.speed = 150_deg;
  theHeadMotionRequest.stopAndGoMode = false;

  theMotionRequest.odometryData = theOdometryData;
  theMotionRequest.ballEstimate = theBallModel.estimate;
  theMotionRequest.ballEstimateTimestamp = theFrameInfo.time;
  theMotionRequest.ballTimeWhenLastSeen = theBallModel.timeWhenLastSeen;
  theMotionRequest.postKickPursuitTimestamp = 0;
  theMotionRequest.postKickPursuitActive = false;
  theMotionRequest.fixedRunDriveEvent=theMotionRequest.fixedRunDriveFrame=0;
  theMotionRequest.fixedRunDriveSpeed=0.f; // No ordinary or stale output inherits a RUN lease.
  theMotionRequest.fixedRunDriveCoordinateVersion=0;
  autonomousRunOnlyThisFrame=false;
  theMotionRequest.targetOfInterest.reset();

  if(theEnhancedKeyStates.hitStreak[KeyStates::walk] == 3 && !theEnhancedKeyStates.pressed[KeyStates::stand])
  {
    // On request, tell the user the deployed configuration of this robot.
    SystemCall::say((Global::getSettings().robotName +
                     " is wearing a " +
                     std::string(TypeRegistry::getEnumName(theGameState.color())) +
                     " jersey with the number " +
                     std::to_string(theGameState.playerNumber) +
                     ". It is deployed in " +
                     std::string(TypeRegistry::getEnumName(theRobotHealth.configuration)) +
                     " for the scenario " +
                     theRobotHealth.scenario +
                     " at the location " +
                     theRobotHealth.location +
                     ".").c_str(), true, 0.9f);
  }

  WhiteFleetBoosterOfficialLiveTest::notePreviousPlayerState(static_cast<unsigned>(theExtendedGameState.playerStateLastFrame));
  physicalPickupVisited = false;
  postKickPlayingVisited = false;
  beginFrame(theFrameInfo.time);
  // The process-wide commissioning flag is set before Robot construction and
  // cannot be overridden by scenario, robot configuration, or remote buttons.
  const bool commissioning = WhiteFleetBoosterAppCommissioning::enabled();
  if(commissioning)
  {
    execute("CommissionPlayDead");
    WhiteFleetBoosterAppCommissioning::noteBehaviorFrame();
  }
  else
    execute(root);
  endFrame();
  maintainAutonomousKickoffContext(true); // Retire private intent on hard preemption; never overwrite manual motion.
  if(WhiteFleetMatchTacticalRepair::enabled(theBehaviorParameters.matchTacticalRepair) && postKickPlayingVisited &&
     theGameState.isPlaying() && !theGameState.stopped && theGameState.playerState==GameState::active && theGameState.isGoalkeeper() &&
     !theBehaviorStateInformation.remoteControlActive && !theInterceptBallInformation.intercept &&
     (theMotionRequest.motion==MotionRequest::walkToBallAndKick || theMotionRequest.motion==MotionRequest::dribble))
  {
    const float ratio=WhiteFleetMatchTacticalRepair::keeperApproachRatio(
      {theRobotPose.translation.x(),theRobotPose.translation.y()},theFieldDimensions.xPosOwnPenaltyArea,
      theFieldDimensions.yPosLeftPenaltyArea,theBehaviorParameters.keeperPursuitOutsidePenalty,
      theBehaviorParameters.keeperPursuitBuffer);
    theMotionRequest.walkSpeed.translation*=ratio;
  }
  if(!physicalPickupVisited) physicalPickupGate.suspend();
  if(!postKickPlayingVisited)
  {
    postKickReacquire.suspend(theMotionInfo.lastKickTimestamp);
    pursuitSession.reset();observationGuard.reset();observationHeld=false;
  }
  else if(theMotionRequest.motion == MotionRequest::walkToBallAndKick ||
          theMotionRequest.motion == MotionRequest::dribble)
    postKickReacquire.noteCommand(postKickInput, theMotionRequest.targetDirection, theMotionRequest.kickLength);

  theMotionRequest.shouldInterceptBall = !commissioning && !WhiteFleetBoosterPickupControl::held() && theInterceptBallInformation.intercept && !theBehaviorStateInformation.remoteControlActive;
  theMotionRequest.shouldWalkOutOfBallLine = !commissioning && !WhiteFleetBoosterPickupControl::held() && theInterceptBallInformation.walkOutOfBallPath && !theBehaviorStateInformation.remoteControlActive;
  if(autonomousRunOnlyThisFrame)
    theMotionRequest.shouldInterceptBall=false; // Do not replace fixed navigation with an automatic ball chase.
  if(theBehaviorParameters.c15ReceiverSimMaxRun&&WhiteFleetSimPhysical::simulated()
     &&Global::getSettings().teamNumber==5&&Global::getSettings().playerNumber==2
     &&theMotionRequest.fixedRunDriveCoordinateVersion==2u&&theMotionRequest.fixedRunDriveFrame==theFrameInfo.time
     &&theMotionRequest.motion==MotionRequest::walkToPose)
  {
    if(theInterceptBallInformation.walkOutOfBallPath||theMotionRequest.shouldWalkOutOfBallLine)
    {
      // A real walk-out request wins; do not leave an invalid RUN lease that
      // would make the existing motion consumer stop the normal escape output.
      theMotionRequest.fixedRunDriveEvent=theMotionRequest.fixedRunDriveFrame=0;
      theMotionRequest.fixedRunDriveSpeed=0.f;theMotionRequest.fixedRunDriveCoordinateVersion=0;
    }
    else {theMotionRequest.shouldInterceptBall=false;theMotionRequest.shouldWalkOutOfBallLine=false;}
  }

  // Automatic remembered-ball interception must not replace an authenticated
  // fixed RUN. Walk-out, collisions, falls and faults remain independent.
  if(theBehaviorParameters.c15ReceiverFixedRunToArrival&&theBehaviorParameters.c15ReceiverFixedDirect
     &&theBehaviorParameters.c15EarlyChannelReceive&&theBehaviorParameters.c15ReceiverLocalStep
     &&!theBehaviorParameters.c15ReceiverLowerChannel&&!theBehaviorParameters.c15ReceiverShortestIntercept
     &&theSkillRequest.kickoffThrough&&theSkillRequest.kickoffThroughAction==7
     &&theGameState.playerNumber==theSkillRequest.kickoffThroughReceiver
     &&!theSkillRequest.kickoffThroughRetired&&!theSkillRequest.isolatedKickoffFinished
     &&theMotionRequest.motion==MotionRequest::walkToPose
     &&c15EarlyChannel.fixedArrivalPhase==WhiteFleetC15EarlyChannel::FixedArrivalPhase::run
     &&WhiteFleetC15EarlyChannel::fixedArrivalContext(c15EarlyChannel,theFrameInfo.time,theSkillRequest.kickoffThroughEvent,
         theSkillRequest.kickoffFixedRunFrame,theSkillRequest.kickoffFixedRunRelease,
         theSkillRequest.kickoffThroughKicker,theSkillRequest.kickoffThroughReceiver,
         {theSkillRequest.kickoffThroughTarget.x(),theSkillRequest.kickoffThroughTarget.y()},
         {theBehaviorParameters.c15FixedReceiveBallX,theBehaviorParameters.c15FixedReceiveBallY}))
    theMotionRequest.shouldInterceptBall=false;

  theLibCheck.performCheck();
}

void WhiteFleetSkillBehaviorControl::publishRestartPassIntent()
{
  // Native output skill, called INSIDE the playing option's action/frame.
  // Existing messages keep their original scheduler, rate and budget.
  if(WhiteFleetMatchTacticalRepair::enabled(theBehaviorParameters.matchTacticalRepair) &&
     (theSkillRequest.restartPassInFlight || (theSkillRequest.restartPassRequired&&theSkillRequest.restartPassPreparing&&
       theSkillRequest.restartPassPasser==theGameState.playerNumber&&theSkillRequest.restartPassEpoch&&
       theFrameInfo.time>=theSkillRequest.restartPassEpoch&&theFrameInfo.time-theSkillRequest.restartPassEpoch<
         (theSkillRequest.restartPassGoalKick?WhiteFleetMatchTacticalRepair::goalKickContractBudget:WhiteFleetMatchTacticalRepair::kickoffContractBudget))) && !theGameState.stopped &&
     theGameControllerData.stopped==0 && !observationHeld &&
     !theBehaviorStateInformation.remoteControlActive && theBehaviorStatus.passTarget<0 &&
     theSkillRequest.restartPassReceiver>0 && theSkillRequest.restartPassTarget.allFinite())
    PassTarget({.passTarget=theSkillRequest.restartPassReceiver,
                .ballTarget=theRobotPose.inverse()*theSkillRequest.restartPassTarget});
  publishAutonomousKickoffIntent(); // Inside the original action/frame and existing message scheduler.
}

void WhiteFleetSkillBehaviorControl::maintainAutonomousKickoffContext(bool retireActive)
{
  const bool enabled=WhiteFleetAutonomousKickoff::enabled(WhiteFleetSimPhysical::physical(),
      WhiteFleetK1V21MatchProfile::guardedPhysicalReceiver(),WhiteFleetSimPhysical::simulated(),
      Global::getSettings().teamNumber,std::getenv("MDFZ_AUTONOMOUS_KICKOFF_V4_ENABLE"));
  const bool authorized=enabled&&theGameState.playerState==GameState::active&&!theGameState.stopped&&!theGameControllerData.stopped&&
      theGameControllerData.setPlay==SET_PLAY_NONE&&
      !theBehaviorStateInformation.remoteControlActive&&!WhiteFleetBoosterPickupControl::held()&&!WhiteFleetBoosterRemoteStand::held()&&
      !theSystemSensorData.emergencyMode&&!theEmergencyButtonState.buttonPressed&&theFallDownState.state==FallDownState::upright&&
      (theMotionInfo.executedPhase==MotionPhase::stand||theMotionInfo.executedPhase==MotionPhase::walk)&&
      (SystemCall::getMode()!=SystemCall::physicalRobot||WhiteFleetSimPhysical::bodyAuthorized());
  const bool own=theGameState.state==GameState::ownKickOff&&theGameState.isForOwnTeam();
  if(!authorized||(!own&&theGameState.state!=GameState::playing))
  {
    if(autonomousKickoffObservedEvent)autonomousReceiveFinishedEvent=autonomousKickoffObservedEvent;
    autonomousKickoffObservedEvent=0;
    if(retireActive&&autonomousReceiveEvent)
    {autonomousReceiveFinishedEvent=autonomousReceiveEvent;autonomousReceiveEvent=0;}
  }
  else if(own)
  {
    if(autonomousReceiveEvent&&autonomousReceiveEvent!=theGameState.timeWhenStateStarted)
    {autonomousReceiveFinishedEvent=autonomousReceiveEvent;autonomousReceiveEvent=0;}
    autonomousKickoffObservedEvent=theGameState.timeWhenStateStarted;
  }
}

void WhiteFleetSkillBehaviorControl::publishAutonomousKickoffIntent()
{
  maintainAutonomousKickoffContext(false);
  const unsigned now=theFrameInfo.time;
  if(theGameState.state==GameState::playing&&!theGameState.isGoalkeeper()&&autonomousEvent&&
     autonomousEvent==autonomousKickoffObservedEvent&&now>=autonomousEvent&&now-autonomousEvent<=2000u&&
     autonomousReceiver>=Settings::lowestValidPlayerNumber&&autonomousReceiver<=Settings::highestValidPlayerNumber&&
     autonomousReceiver!=theGameState.playerNumber&&!theGameState.ownTeam.isGoalkeeper(autonomousReceiver)&&
     theGameState.ownTeam.playerStates[autonomousReceiver-Settings::lowestValidPlayerNumber]==GameState::active&&
     !theSkillRequest.restartPassRequired&&!theSkillRequest.restartPassInFlight&&
     autonomousTarget.allFinite()&&theRobotPose.isFinite()&&theBehaviorStatus.passTarget<0&&!theBehaviorStatus.shootingTo.has_value())
    PassTarget({.passTarget=autonomousReceiver,.ballTarget=theRobotPose.inverse()*autonomousTarget});
  // No new wire/ACK/rate, no motion or owner. Existing current pass intents win.
}

Vector2f WhiteFleetSkillBehaviorControl::prepareAutonomousKickoffSupport(Vector2f desired)
{
  const float margin=150.f;
  const float halfLine=theFieldDimensions.fieldLinesWidth*.5f;
  const float halfway=theFieldDimensions.xPosHalfwayLine;
  const Vector2f current=theRobotPose.translation;
  Vector2f target{halfway-halfLine-margin,current.y()};
  bool move=target.x()>current.x()&&theFieldDimensions.isInsideField(current)&&
      !theIllegalAreas.isPositionIllegal(current,margin);
  // Only an actual illegal-side estimate beyond the line warrants the
  // original retreat. Being inside the forward buffer is not bad placement.
  if(current.x()>halfway+halfLine&&theIllegalAreas.isPositionIllegal(current,0.f))
  {
    const auto stage=WhiteFleetMatchTacticalRepair::kickoffStaging({desired.x(),desired.y()},halfway,
        theFieldDimensions.fieldLinesWidth,theBehaviorParameters.illegalAreasLargeDistance,theFieldDimensions.yPosLeftTouchline);
    target={std::clamp(current.x(),theFieldDimensions.xPosOwnGoalLine+600.f,stage.x),
        std::clamp(current.y(),-theFieldDimensions.yPosLeftTouchline+700.f,theFieldDimensions.yPosLeftTouchline-700.f)};
    move=theFieldDimensions.isInsideField(current);
  }
  if(move&&theFieldDimensions.isInsideField(target)&&!theIllegalAreas.isPositionIllegal(target,margin))
    WalkToPointObstacle({.target=Pose2f(Angle::normalize(-theRobotPose.rotation),theRobotPose.inverse()*target),
        .reduceWalkSpeedType=ReduceWalkSpeedType::noChange,.disableObstacleAvoidance=false,
        .targetOfInterest=theFieldBall.recentBallPositionRelative()});
  else
  {
    // No precise staging requirement, no backward shuffle inside our half,
    // and no crossing until the native state releases the half.
    target=current;
    Stand();
  }
  if(theBallModel.timeWhenLastSeen>0&&theBallModel.timeWhenLastSeen<=theFrameInfo.time&&
     theFrameInfo.time-theBallModel.timeWhenLastSeen<=600u&&!observationHeld&&theBallModel.estimate.position.allFinite())
    LookAtPoint({.target={theBallModel.estimate.position.x(),theBallModel.estimate.position.y(),theBallSpecification.radius},.speed=180_deg});
  else LookActive({.withBall=true,.onlyOwnBall=true});
  return target;
}

bool WhiteFleetSkillBehaviorControl::handleRestartReceive()
{
  if(WhiteFleetC15KickoffSwitch::enabled(theBehaviorParameters.matchTacticalRepair&&theBehaviorParameters.c15Kickoff)&&
     theGameState.isKickOff()&&theGameState.isForOwnTeam())return false;
  const bool context=(theGameState.isKickOff()||theGameState.isGoalKick())&&theGameState.isForOwnTeam();
  if(!WhiteFleetMatchTacticalRepair::enabled(theBehaviorParameters.matchTacticalRepair)||theGameState.isGoalkeeper()||
     !theGameState.isPlaying()||theGameState.stopped||theGameControllerData.stopped!=0||theBehaviorStateInformation.remoteControlActive||
     theGameState.playerState!=GameState::active||(!context&&!theSkillRequest.restartPassRequired)||
     (theSkillRequest.restartPassRequired&&theSkillRequest.restartPassPasser==theGameState.playerNumber))return false;
  const unsigned epoch=theSkillRequest.restartPassRequired?theSkillRequest.restartPassEpoch:theGameState.timeWhenStateStarted;
  for(const Teammate& peer:theTeamData.teammates)
    if(peer.number!=theGameState.playerNumber&&peer.theRobotStatus.isUpright&&peer.theRobotPose.isFinite()&&
       peer.theRobotPose.quality!=RobotPose::poor&&TeamActivityPolicy::fresh(theFrameInfo.time,peer.theFrameInfo.time,1200)&&
       peer.theFrameInfo.time>=epoch&&peer.theBehaviorStatus.passTarget==theGameState.playerNumber&&
       peer.theBehaviorStatus.shootingTo.has_value()&&
       (!(theGameState.isGoalKick()||theSkillRequest.restartPassGoalKick)||peer.isGoalkeeper))
    {
      const Vector2f target=peer.theRobotPose*peer.theBehaviorStatus.shootingTo.value();
      if(!target.allFinite()||target.x()>=theFieldDimensions.xPosOpponentGoalLine-600.f||
         std::abs(target.y())>=theFieldDimensions.yPosLeftTouchline-500.f)continue;
      theBehaviorStatus.passOrigin=peer.number;
      ReceivePass({.playerNumber=peer.number});return true;
    }
  return false;
}

bool WhiteFleetSkillBehaviorControl::handlePostKickReacquire()
{
  const bool ballAction = theSkillRequest.skill == SkillRequest::shoot ||
    theSkillRequest.skill == SkillRequest::dribble || theSkillRequest.skill == SkillRequest::clear;
  const bool tactical=WhiteFleetMatchTacticalRepair::enabled(theBehaviorParameters.matchTacticalRepair);
  const bool normalBodyPlay=theGameState.state==GameState::playing && !theGameState.stopped &&
    theGameControllerData.stopped==0 && theGameState.playerState==GameState::active &&
    !theBehaviorStateInformation.remoteControlActive && !WhiteFleetBoosterPickupControl::held() &&
    !theSystemSensorData.emergencyMode && !theEmergencyButtonState.buttonPressed &&
    theFallDownState.state==FallDownState::upright;
  const bool held=tactical && observationGuard.update(theFrameInfo.time,theBallModel.timeWhenLastSeen,
    theMotionInfo.lastKickTimestamp,normalBodyPlay,
    {theBallModel.estimate.position.x(),theBallModel.estimate.position.y()},
    {theOdometryData.translation.x(),theOdometryData.translation.y()},theOdometryData.rotation);
  if(held!=observationHeld)
    OUTPUT_TEXT("MATCH_BALL_OBSERVATION quarantine=" << held << " seen=" << theBallModel.timeWhenLastSeen <<
      " kick=" << theMotionInfo.lastKickTimestamp << " unchanged_raw_model=1");
  observationHeld=held;
  if(held && ballAction)
  {
    pursuitSession.reset();postKickReacquire.suspend(theMotionInfo.lastKickTimestamp);
    LookActive({.withBall=true});Stand();return true;
  }
  // This runs below native Physical/STOP/Joystick/Player/return gates. A new
  // owner, a pass, a fall or any interruption must NOT keep the old shooter's
  // session alive. No goalkeeper chase or set-play permission is introduced.
  const bool allowed = WhiteFleetK1HotChase::reacquireEnabled() && ballAction &&
    theGameState.state == GameState::playing && !theGameState.stopped &&
    theGameControllerData.stopped == 0 && theGameState.playerState == GameState::active &&
    !theGameState.isGoalkeeper() && !WhiteFleetBoosterPickupControl::held() &&
    !theBehaviorStateInformation.remoteControlActive && !theSystemSensorData.emergencyMode &&
    !theEmergencyButtonState.buttonPressed && theFallDownState.state == FallDownState::upright &&
    (theMotionInfo.executedPhase == MotionPhase::stand || theMotionInfo.executedPhase == MotionPhase::walk) &&
    !theInterceptBallInformation.intercept && !theInterceptBallInformation.walkOutOfBallPath &&
    theRobotPose.quality != RobotPose::poor && theInertialData.angle.allFinite() &&
    theInertialData.gyro.allFinite() && theInertialData.angle.head<2>().norm() < .22f &&
    theInertialData.gyro.norm() < 2.5f;
  postKickInput = {theFrameInfo.time, theBallModel.timeWhenLastSeen, theMotionInfo.lastKickTimestamp,
    allowed, theFieldBall.positionRelative.x(), theFieldBall.positionRelative.y(),
    theFieldBall.velocityRelative.x(), theFieldBall.velocityRelative.y(),
    theOdometryData.translation.x(), theOdometryData.translation.y(), theOdometryData.rotation};
  postKickInput.logPolicy = WhiteFleetK1HotChase::logReacquireEnabled();
  postKickInput.bodyAllowed = allowed;
  // A brief upright kicking sway may revoke pursuit without erasing where
  // to LOOK. Hard body/owner/referee gates remain identical; this grace can
  // only request native Stand plus head gaze, never a walking command.
  if(postKickInput.logPolicy && ballAction && theCameraMatrix.isValid &&
     theGameState.state == GameState::playing && !theGameState.stopped &&
     theGameControllerData.stopped == 0 && theGameState.playerState == GameState::active &&
     !theGameState.isGoalkeeper() && !WhiteFleetBoosterPickupControl::held() &&
     !theBehaviorStateInformation.remoteControlActive && !theSystemSensorData.emergencyMode &&
     !theEmergencyButtonState.buttonPressed && theFallDownState.state == FallDownState::upright &&
     (theMotionInfo.executedPhase == MotionPhase::stand || theMotionInfo.executedPhase == MotionPhase::walk) &&
     !theInterceptBallInformation.intercept && !theInterceptBallInformation.walkOutOfBallPath &&
     theRobotPose.quality != RobotPose::poor && theInertialData.angle.allFinite() &&
     theInertialData.gyro.allFinite() && theInertialData.angle.head<2>().norm() < .35f &&
     theInertialData.gyro.norm() < 4.f)
    postKickInput.allowed = true;
  postKickInput.perceptValid = theCameraMatrix.isValid && theBallPercept.status == BallPercept::seen &&
    theBallPercept.positionInImage.allFinite() && std::isfinite(theBallPercept.radiusInImage) &&
    theBallPercept.radiusInImage > 1.f && theBallPercept.positionInImage.x() >= 0.f &&
    theBallPercept.positionInImage.y() >= 0.f && theBallPercept.positionInImage.x() < theCameraInfo.width &&
    theBallPercept.positionInImage.y() < theCameraInfo.height;
  postKickInput.perceptTime = postKickInput.perceptValid ? theFrameInfo.time : 0;
  postKickInput.perceptX = theBallPercept.positionOnField.x();
  postKickInput.perceptY = theBallPercept.positionOnField.y();
  const auto previous = postKickReacquire.mode();
  const auto recovery = postKickReacquire.tick(postKickInput);
  const unsigned previousPursuit=pursuitSession.witness();
  const bool continued=tactical && pursuitSession.update(theFrameInfo.time,recovery.pursuitTimestamp,
    theBallModel.timeWhenLastSeen,allowed && !held && theStrategyStatus.role==ActiveRole::toRole(ActiveRole::playBall),
    {theOdometryData.translation.x(),theOdometryData.translation.y()},theFieldBall.positionRelative.norm());
  if(tactical && ((previousPursuit!=0)!=continued))
    OUTPUT_TEXT("MATCH_HOT_CONTINUITY active=" << continued << " original_witness=" <<
      (continued?pursuitSession.witness():previousPursuit) << " ball_age=" << theFieldBall.timeSinceBallWasSeen <<
      " body_allowed=" << allowed << " role=" << static_cast<int>(theStrategyStatus.role) << " observation_hold=" << held);
  if(previous != postKickReacquire.mode())
  {
    ANNOTATION("WhiteFleetPostKickReacquire", static_cast<int>(postKickReacquire.mode()));
    OUTPUT_TEXT("POST_KICK_REACQUIRE mode=" << static_cast<int>(postKickReacquire.mode()) <<
      " ball_age=" << theFieldBall.timeSinceBallWasSeen <<
      " kick=" << theMotionInfo.lastKickTimestamp << " witness=" << recovery.pursuitTimestamp);
  }
  if(!recovery.gaze && continued)
  {
    theMotionRequest.postKickPursuitActive=true;
    theMotionRequest.postKickPursuitTimestamp=pursuitSession.witness();
    executeRequest();handleWalkSpeedAtFieldBorder();return true;
  }
  if(!recovery.gaze)
    return false;
  theMotionRequest.postKickPursuitTimestamp = continued ? pursuitSession.witness() : recovery.pursuitTimestamp;
  theMotionRequest.postKickPursuitActive=continued && !recovery.stopForVision && theMotionRequest.postKickPursuitTimestamp!=0;
  if(recovery.stopForVision)
  {
    // A predicted look direction is not permission to run towards an unseen
    // ball. Native Stand safely decelerates; no retained walking command.
    Stand();
    LookAtPoint({.target = {recovery.target.x, recovery.target.y, theBallSpecification.radius},
                 .speed = 180_deg});
  }
  else
  {
    executeRequest();
    handleWalkSpeedAtFieldBorder();
    // Exactly one native head request was counted by executeRequest. Replace
    // that request, rather than issuing/counter-incrementing a second skill.
    theHeadMotionRequest.mode = HeadMotionRequest::targetOnGroundMode;
    theHeadMotionRequest.target = {recovery.target.x, recovery.target.y, theBallSpecification.radius};
    theHeadMotionRequest.speed = 180_deg;
    theHeadMotionRequest.stopAndGoMode = false;
  }
  return true;
}

bool WhiteFleetSkillBehaviorControl::handleKickerPostKickHold()
{
  // 2026-11-10 robust kickoff: the kicker may not play the ball again before
  // another player did (rules). Without relying on team communication, the
  // designated kicker keeps away from the ball for 10 s after its own first
  // kick of the own kickoff: stands when the ball is near, otherwise walks to a
  // support spot on the far side with native obstacle avoidance.
  if(!WhiteFleetAutonomousKickoff::isKicker(theGameState.playerNumber)||theGameState.isGoalkeeper())
    return false;
  const unsigned now=theFrameInfo.time;
  const unsigned ev=autonomousKickoffObservedEvent;
  if(!ev){kickerHoldEvent=kickerKickAt=0;return false;}
  if(kickerHoldEvent!=ev){kickerHoldEvent=ev;kickerKickAt=0;kickerHoldBaseline=theMotionInfo.lastKickTimestamp;}
  if(!kickerKickAt&&theMotionInfo.lastKickTimestamp>kickerHoldBaseline&&theMotionInfo.lastKickTimestamp>=ev)
    kickerKickAt=theMotionInfo.lastKickTimestamp;
  // Start 400 ms after the kick so the kick motion itself is never interrupted.
  if(!kickerKickAt||now<kickerKickAt+400u||now-kickerKickAt>10000u||(theGameState.state!=GameState::playing&&
     theGameState.state!=GameState::ownKickOff)||theGameState.stopped||theGameState.playerState!=GameState::active||
     theFallDownState.state!=FallDownState::upright)
    return false;
  const Vector2f ball=theFieldBall.recentBallPositionRelative();
  const Vector2f support(std::min(2500.f,theFieldDimensions.xPosOpponentPenaltyArea-1000.f),1500.f);
  const Vector2f rel=theRobotPose.inverse()*support;
  if(ball.squaredNorm()<sqr(1200.f)||!rel.allFinite()||rel.squaredNorm()<sqr(150.f)||!theRobotPose.isFinite())
  {
    LookActive({.withBall=true});Stand();
  }
  else
  {
    WalkToPointObstacle({.target=Pose2f(Angle::normalize(-theRobotPose.rotation),rel),.speed={1.f,1.f,1.f},
        .reduceWalkSpeedType=ReduceWalkSpeedType::noChange,.disableObstacleAvoidance=false,.disableAligning=false,
        .targetOfInterest=ball});
    LookActive({.withBall=true});
  }
  if(!kickerHoldLog||now-kickerHoldLog>=1000u)
  {
    kickerHoldLog=now;
    std::printf("KICKER_POST_KICK_HOLD self=%d event=%u kick_at=%u age_ms=%u ball_dist=%.0f\n",
        theGameState.playerNumber,ev,kickerKickAt,now-kickerKickAt,ball.norm());
  }
  return true;
}

bool WhiteFleetSkillBehaviorControl::handleAutonomousKickoffReceive()
{
  const unsigned now=theFrameInfo.time;
  const bool own=theGameState.state==GameState::ownKickOff&&theGameState.isForOwnTeam();
  const bool playing=theGameState.state==GameState::playing;
  const bool halfClosed=own&&(theIllegalAreas.illegal&(1u<<IllegalAreas::opponentHalf));
  if(!WhiteFleetAutonomousKickoff::isReceiver(theGameState.playerNumber)||theGameState.isGoalkeeper()||(!own&&!playing))
  {
    autonomousReceiveEvent=0;autonomousReceiveSender=-1;
    autonomousRunAnchorEvent=0;
    autonomousReceiveFinishing=false;autonomousReceiveContactSeenAt=0;return false;
  }
  if(autonomousReceiveEvent&&autonomousReceiveSender!=WhiteFleetAutonomousKickoff::kickerNumber)
  {autonomousReceiveEvent=0;autonomousReceiveSender=-1;autonomousReceiveFinishing=false;autonomousReceiveContactSeenAt=0;}
  const float variance=theRobotPose.covariance(2,2);
  const bool bodyAllowed=!theGameState.stopped&&!theGameControllerData.stopped&&theGameState.playerState==GameState::active&&
      (SystemCall::getMode()!=SystemCall::physicalRobot||WhiteFleetSimPhysical::bodyAuthorized())&&
      !theSystemSensorData.emergencyMode&&!theEmergencyButtonState.buttonPressed&&
      !theBehaviorStateInformation.remoteControlActive&&!WhiteFleetBoosterPickupControl::held()&&!WhiteFleetBoosterRemoteStand::held()&&
      theFallDownState.state==FallDownState::upright&&
      (theMotionInfo.executedPhase==MotionPhase::stand||theMotionInfo.executedPhase==MotionPhase::walk);
  const bool poseAllowed=theRobotPose.isFinite()&&std::isfinite(variance)&&variance>=0.f&&variance<=.616850275f;
  const unsigned candidateEvent=own?theGameState.timeWhenStateStarted:autonomousKickoffObservedEvent;
  const bool lateJoin=playing&&candidateEvent&&candidateEvent<=now&&now-candidateEvent<=3200u&&
      autonomousEvent!=candidateEvent; // Original2s publication + existing1.2s packet age, never a new11s clock.
  if((own||lateJoin)&&candidateEvent&&autonomousReceiveEvent!=candidateEvent&&
     autonomousReceiveFinishedEvent!=candidateEvent)
  {
    for(const Teammate& teammate:theTeamData.teammates)
    {
      if(teammate.number!=WhiteFleetAutonomousKickoff::kickerNumber||teammate.isGoalkeeper||teammate.number==theGameState.playerNumber||
         teammate.number<Settings::lowestValidPlayerNumber||teammate.number>Settings::highestValidPlayerNumber||
         theGameState.ownTeam.isGoalkeeper(teammate.number)||
         theGameState.ownTeam.playerStates[teammate.number-Settings::lowestValidPlayerNumber]!=GameState::active||!teammate.theRobotStatus.isUpright||
         !TeamActivityPolicy::fresh(now,teammate.theFrameInfo.time,1200u)||
         teammate.theFrameInfo.time<candidateEvent||!teammate.theRobotPose.isFinite()||
         teammate.theBehaviorStatus.passTarget!=theGameState.playerNumber||
         !teammate.theBehaviorStatus.shootingTo.has_value())continue;
      const Vector2f target=teammate.theRobotPose*teammate.theBehaviorStatus.shootingTo.value();
      if(!target.allFinite()||target.x()<=theFieldDimensions.xPosHalfwayLine||
         target.x()>theFieldDimensions.xPosOpponentGoalLine-600.f||target.y()>=0.f||
         target.y()<-theFieldDimensions.yPosLeftTouchline+300.f)continue;
      autonomousReceiveEvent=candidateEvent;
      autonomousReceiveSender=teammate.number;autonomousReceiveTarget=target;
      autonomousReceiveKickAtStart=theMotionInfo.lastKickTimestamp;
      autonomousReceiveFinishing=false;
      autonomousReceiveContactSeenAt=0;
      autonomousRunArrivedEvent=0;
      autonomousShortCutEvent=autonomousShortBallFirstSeen=0;
      autonomousRunTrustedSince=autonomousRunBridgeWindows=autonomousRunBridgeMs=autonomousRunBridgeTick=0;
      std::printf("AUTONOMOUS_KICKOFF_RECEIVE self=%d sender=%d event=%u P=%.1f,%.1f packet=%u fixed_budget_ms=11000 ack=0\n",
          theGameState.playerNumber,autonomousReceiveSender,autonomousReceiveEvent,target.x(),target.y(),teammate.theFrameInfo.time);
      break;
    }
  }
  // 2026-11-10 robust kickoff: do not depend on team communication. If no
  // intent packet from the kicker arrived by the time the ball is released
  // (opponent half open), red2 starts its receive on its own, aimed at the
  // configured receive area; the run itself follows the live ball it sees.
  if(!autonomousReceiveEvent&&(own||lateJoin)&&candidateEvent&&autonomousReceiveFinishedEvent!=candidateEvent&&
     !halfClosed&&bodyAllowed&&
     theGameState.ownTeam.playerStates[WhiteFleetAutonomousKickoff::kickerNumber-Settings::lowestValidPlayerNumber]==GameState::active)
  {
    autonomousReceiveEvent=candidateEvent;
    autonomousReceiveSender=WhiteFleetAutonomousKickoff::kickerNumber;
    autonomousReceiveTarget=Vector2f(theBehaviorParameters.c15FixedReceiveBallX,theBehaviorParameters.c15FixedReceiveBallY);
    autonomousReceiveKickAtStart=theMotionInfo.lastKickTimestamp;
    autonomousReceiveFinishing=false;
    autonomousReceiveContactSeenAt=0;
    autonomousRunArrivedEvent=0;
    autonomousShortCutEvent=autonomousShortBallFirstSeen=0;
    autonomousRunTrustedSince=autonomousRunBridgeWindows=autonomousRunBridgeMs=autonomousRunBridgeTick=0;
    std::printf("AUTONOMOUS_KICKOFF_RECEIVE self=%d sender=%d event=%u P=%.1f,%.1f packet=0 self_start=1 fixed_budget_ms=11000 ack=0\n",
        theGameState.playerNumber,autonomousReceiveSender,autonomousReceiveEvent,
        autonomousReceiveTarget.x(),autonomousReceiveTarget.y());
  }
  if(!autonomousReceiveEvent)
  {
    if(own&&halfClosed&&autonomousKickoffObservedEvent==theGameState.timeWhenStateStarted&&bodyAllowed)
    {
      autonomousRunOnlyThisFrame=!theBehaviorParameters.c15ReceiverSimMaxRun;
      if(poseAllowed)prepareAutonomousKickoffSupport({theFieldDimensions.xPosHalfwayLine,theRobotPose.translation.y()});
      else {LookActive({.withBall=false});Stand();}
      return true; // Lost intent is not a request to walk sideways to the ordinary support formation.
    }
    return false;
  }
  const unsigned age=theBallModel.timeWhenLastSeen>0&&theBallModel.timeWhenLastSeen<=now?
      now-theBallModel.timeWhenLastSeen:std::numeric_limits<unsigned>::max();
  int native4a2Mask=-1;
  const auto logFlow=[&](int mode,const char* branch,Vector2f target)
  {
    // SIM tune 2026-11-10: per-event budget (before: only the first event ever logged).
    if(autonomousReceiveDiagnosticEvent!=autonomousReceiveEvent)
    {autonomousReceiveDiagnosticEvent=autonomousReceiveEvent;autonomousReceiveLogCount=0;autonomousReceiveLastLog=0;autonomousReceiveLastMode=-1;}
    if(autonomousReceiveLogCount>=64u||
       (mode==autonomousReceiveLastMode&&autonomousReceiveLastLog&&now-autonomousReceiveLastLog<250u))return;
    autonomousReceiveLastMode=mode;autonomousReceiveLastLog=now;++autonomousReceiveLogCount;
    std::printf("AUTONOMOUS_RX_FLOW event=%u now=%u branch=%s heading_var=%.6f ball_age=%u half_closed=%d target=%.1f,%.1f skill=%u motion_at_hook=%u kick_at_hook=%u length_at_hook=%.1f phase=%u last_kick_proxy=%u intent_fixed=1 budget_fixed=1 native4a2_mask=%d ball_dist=%.0f ball_seen_age=%u\n",
        autonomousReceiveEvent,now,branch,variance,age,halfClosed,target.x(),target.y(),static_cast<unsigned>(theSkillRequest.skill),
        static_cast<unsigned>(theMotionRequest.motion),static_cast<unsigned>(theMotionRequest.kickType),theMotionRequest.kickLength,
        static_cast<unsigned>(theMotionInfo.executedPhase),theMotionInfo.lastKickTimestamp,native4a2Mask,theBallModel.estimate.position.norm(),
        now>=theBallModel.timeWhenLastSeen?now-theBallModel.timeWhenLastSeen:0u);
  };
  if(!bodyAllowed)
  {
    autonomousRunAnchorEvent=0;
    LookActive({.withBall=true});Stand();logFlow(0,"body_or_referee_retire",autonomousReceiveTarget);
    autonomousReceiveFinishedEvent=autonomousReceiveEvent;autonomousReceiveEvent=0;
    return true;
  }
  const bool red2Native=playing&&theGameState.playerNumber==2;
  const bool fixedRunToArea=red2Native&&!theBehaviorParameters.c15ReceiverSimMaxRun;
  const bool contactProxy=theMotionInfo.lastKickTimestamp>autonomousReceiveKickAtStart&&
      theMotionInfo.lastKickTimestamp>=autonomousReceiveEvent;
  if(contactProxy&&autonomousReceiveFinishing&&!autonomousReceiveContactSeenAt)autonomousReceiveContactSeenAt=now;
  const bool contactEnd=contactProxy&&((!fixedRunToArea&&!autonomousReceiveFinishing)||
      (autonomousReceiveContactSeenAt&&now-autonomousReceiveContactSeenAt>350u));
  // SIM tune 2026-11-10: once finishing, a ball unseen for >800 ms hands red2
  // back to native play (search/chase) instead of standing until 11 s.
  const bool finishingBallLost=autonomousReceiveFinishing&&
      (theBallModel.timeWhenLastSeen<autonomousReceiveEvent||theBallModel.timeWhenLastSeen>now||
       now-theBallModel.timeWhenLastSeen>800u);
  if(autonomousReceiveEvent>now||now-autonomousReceiveEvent>11000u||
     theGameState.ownTeam.playerStates[autonomousReceiveSender-Settings::lowestValidPlayerNumber]!=GameState::active||
     contactEnd||finishingBallLost)
  {
    autonomousRunAnchorEvent=0;
    logFlow(1,"clock_or_roster_or_contact_end",autonomousReceiveTarget);
    autonomousReceiveFinishedEvent=autonomousReceiveEvent;autonomousReceiveEvent=0;
    lastReceivePassRequestTimestamp=0;receivePassPlayerNumber=-1;return false;
  }
  const Vector2f ballRef(theBehaviorParameters.c15FixedReceiveBallX,theBehaviorParameters.c15FixedReceiveBallY);
  const Vector2f bodyRef(ballRef.x()-650.f,ballRef.y());
  const Vector2f target=red2Native?bodyRef:autonomousReceiveTarget;
  // 4a2's fixed RUN -> ARRIVED split, with the agreed one-metre receive area.
  // Position gates the phase, never a ball estimate or a kick proxy.
  if(fixedRunToArea&&poseAllowed&&(theRobotPose.translation-ballRef).squaredNorm()<=sqr(1000.f))
    autonomousRunArrivedEvent=autonomousReceiveEvent;
  const bool fixedArrived=fixedRunToArea&&autonomousRunArrivedEvent==autonomousReceiveEvent;
  autonomousRunOnlyThisFrame=fixedRunToArea&&!fixedArrived;
  const auto guideTarget=[&](const Vector2f& origin) -> Vector2f
  {
    const float halfway=theFieldDimensions.xPosHalfwayLine;
    if(!red2Native||theBehaviorParameters.c15ReceiverSimMaxRun||target.x()-halfway<4500.f||
       target.y()>=0.f||target.y()-theFieldDimensions.yPosRightTouchline<1000.f)
      return target;
    // The successful 4a2 body traces go below the defender on the first
    // stretch. Bias that stretch only, then return continuously to the
    // original body target. Never move the passer's P or the one-metre handoff.
    const float bias=std::clamp((3900.f-(origin.x()-halfway))/1700.f,0.f,1.f);
    const float forward=std::min(target.x(),origin.x()+1800.f);
    return {target.x()+(forward-target.x())*bias,target.y()-320.f*bias};
  };
  const bool canAnchor=fixedRunToArea&&!fixedArrived&&!autonomousReceiveFinishing&&
      theGameControllerData.setPlay==SET_PLAY_NONE&&theRobotPose.isFinite()&&theRobotPose.quality!=RobotPose::poor&&
      theMotionInfo.executedPhase==MotionPhase::walk&&theMotionInfo.isMotionStable&&theOdometryData.isFinite()&&
      !theInterceptBallInformation.walkOutOfBallPath&&!theMotionRequest.shouldWalkOutOfBallLine;
  if(!poseAllowed)
  {
    // Covariance alone must not halt a continuous, independently bounded run.
    // The estimate remains unfit for global aiming or a kick. Use the last
    // trusted route in the odometry frame with the original native avoidance.
    const bool sameBridge=autonomousRunBridgeEvent==autonomousReceiveEvent&&autonomousRunBridgeAnchor==autonomousRunAnchorTime;
    const bool newBridge=autonomousRunBridgeEvent!=autonomousReceiveEvent||
      (autonomousRunBridgeWindows<2u&&autonomousRunTrustedSince&&autonomousRunAnchorTime>=autonomousRunTrustedSince&&
       autonomousRunAnchorTime-autonomousRunTrustedSince>=90u);
    const unsigned lastBridgeTime=sameBridge?autonomousRunBridgeTick:autonomousRunAnchorTime;
    const unsigned bridgeDt=now>=lastBridgeTime?now-lastBridgeTime:1001u;
    const bool secondBridge=autonomousRunBridgeEvent==autonomousReceiveEvent&&
      (autonomousRunBridgeWindows>=2u||!sameBridge);
    if(canAnchor&&std::isfinite(variance)&&variance>.616850275f&&variance<=.9f&&
       autonomousRunAnchorEvent==autonomousReceiveEvent&&now>=autonomousRunAnchorTime&&
       now-autonomousRunAnchorTime<=1000u&&autonomousRunAnchorJump==theRobotPose.timestampLastJump&&
       (sameBridge||newBridge)&&bridgeDt<=120u&&autonomousRunBridgeMs+bridgeDt<=1000u&&
       (!secondBridge||now-autonomousRunAnchorTime<=350u))
    {
      const Pose2f delta=autonomousRunAnchorOdo.inverse()*theOdometryData;
      const Pose2f predicted=autonomousRunAnchorPose*delta;
      const Vector2f toward=predicted.inverse()*guideTarget(predicted.translation);
      if((delta.translation-autonomousRunOdoProgress).squaredNorm()>=sqr(8.f))
      {
        autonomousRunOdoProgress=delta.translation;
        autonomousRunProgressAt=now;
      }
      if(delta.translation.x()>=-100.f&&delta.translation.squaredNorm()<=sqr(1900.f)&&std::abs(delta.rotation)<=24_deg&&
         now>=autonomousRunProgressAt&&now-autonomousRunProgressAt<=120u&&toward.x()>0.f&&
         (predicted.translation-ballRef).squaredNorm()>sqr(1000.f)&&toward.squaredNorm()>sqr(150.f)&&
         std::abs(toward.angle())<=35_deg&&
         (predicted.translation-theRobotPose.translation).squaredNorm()<=sqr(500.f)&&
         std::abs(Angle::normalize(predicted.rotation-theRobotPose.rotation))<=15_deg)
      {
        // Below the native 900 mm switch: stay in LibWalk's local obstacle
        // planner instead of asking the global path planner to use bad pose.
        const Vector2f local=toward.normalized(std::min(800.f,toward.norm()));
        const Vector2f end=predicted*local;
        bool clear=true;
        for(float f : {0.f,.25f,.5f,.75f,1.f})
        {
          const Vector2f p=predicted.translation+(end-predicted.translation)*f;
          clear=clear&&p.x()>theFieldDimensions.xPosHalfwayLine+700.f&&
              p.x()<theFieldDimensions.xPosOpponentGoalLine-1500.f&&
              p.y()>theFieldDimensions.yPosRightTouchline+1500.f&&p.y()<theFieldDimensions.yPosLeftTouchline-1500.f&&
              !theIllegalAreas.isPositionIllegal(p,450.f);
        }
        if(clear)
        {
          // One retry only after a sustained trusted recovery. Neither the
          // shared bridge budget nor the receive deadline is ever renewed.
          if(!sameBridge)++autonomousRunBridgeWindows;
          autonomousRunBridgeEvent=autonomousReceiveEvent;
          autonomousRunBridgeAnchor=autonomousRunAnchorTime;
          autonomousRunBridgeTick=now;
          autonomousRunBridgeMs+=bridgeDt;
          autonomousRunTrustedSince=0;
          WalkToPointObstacle({.target=Pose2f(local.angle(),local),.speed={1.f,1.f,1.f},
              .reduceWalkSpeedType=ReduceWalkSpeedType::noChange,.disableObstacleAvoidance=false,.disableAligning=false,
              .targetOfInterest={}});
          LookAtPoint({.target={local.x(),local.y(),theBallSpecification.radius}});
          logFlow(6,"pose_odometry_native_avoidance",end);
          return true;
        }
      }
    }
    autonomousRunAnchorEvent=0;
    autonomousRunTrustedSince=0;
    // SIM tune 2026-11-10: a poor global heading must not stop red2's landing
    // run while its own ball is fresh. Run to the ball's predicted stop point in
    // the robot frame (no global pose needed), head on the ball; normal flow
    // resumes as soon as the pose is acceptable again. SIM, team 5 / player 2 only.
    if(red2Native&&playing&&!halfClosed&&red2EnhancedRunAllowed()&&
       theBallModel.timeWhenLastSeen>=autonomousReceiveEvent&&age<=300u)
    {
      const Vector2f ballRel=theBallModel.estimate.position;
      Vector2f stopRel=ballRel;
      const Vector2f v=theBallModel.estimate.velocity;
      if(v.allFinite()&&std::isfinite(theBallSpecification.staticFriction)&&theBallSpecification.staticFriction<-.05f)
        stopRel=BallPhysics::getEndPosition(ballRel,v,theBallSpecification);
      if(ballRel.allFinite()&&stopRel.allFinite()&&stopRel.squaredNorm()<=sqr(9000.f))
      {
        // Arrive slightly behind the stop point, seen from the robot.
        const Vector2f target=stopRel.squaredNorm()>sqr(400.f)?stopRel-stopRel.normalized(300.f):stopRel;
        autonomousRunOnlyThisFrame=true;
        WalkToPointObstacle({.target=Pose2f(target.angle(),target),.speed={1.f,1.f,1.f},
            .reduceWalkSpeedType=ReduceWalkSpeedType::noChange,.disableObstacleAvoidance=red2IgnoreObstacles(),.disableAligning=false,
            .targetOfInterest={}});
        LookAtPoint({.target={ballRel.x(),ballRel.y(),theBallSpecification.radius}});
        logFlow(22,"sim_pose_poor_ball_run",theRobotPose*target);
        return true;
      }
    }
    LookActive({.withBall=false});Stand();
    logFlow(2,"pose_hold_not_retired",autonomousReceiveTarget);
    return true; // Original heading threshold still stops the body; the original deadline never renews.
  }
  // Coarse landing region only guides a support runner. Never override the
  // native elected ball-player's legal close-ball pursuit/shot with a waypoint.
  const bool nativeBallPlayer=theSkillRequest.skill==SkillRequest::shoot||theSkillRequest.skill==SkillRequest::pass||
      theSkillRequest.skill==SkillRequest::clear||theSkillRequest.skill==SkillRequest::dribble;
  const bool freshFoot=playing&&
      theBallModel.timeWhenLastSeen>0&&theBallModel.timeWhenLastSeen<=now&&
      now-theBallModel.timeWhenLastSeen<=250u&&theFieldBall.ballWasSeen(250)&&!observationHeld&&
      theBallModel.estimate.position.allFinite();
  // 4a2 KickoffThroughPassV8.cpp:103-108,176-190. Real event/PLAY above
  // replaces its C15 entry; no action/release/ball/controller field is forged.
  bool freshFootBall=false,nativeFresh=false;
  if(red2Native)
  {
    freshFootBall=theBallModel.timeWhenLastSeen>0&&theBallModel.timeWhenLastSeen<=now&&age<=250u&&
        theFieldBall.ballWasSeen(650)&&theFieldBall.positionRelative.allFinite()&&
        theFieldBall.positionOnField.allFinite()&&theBallModel.estimate.position.allFinite();
    const bool bodyInside=theFieldDimensions.isInsideField(theRobotPose.translation);
    const bool ballInside=theFieldBall.positionOnField.allFinite()&&theFieldDimensions.isInsideField(theFieldBall.positionOnField);
    const bool illegal=ballInside&&theIllegalAreas.isPositionIllegal(theFieldBall.positionOnField,150.f);
    const bool fieldFresh=theFieldBall.ballWasSeen(250);
    // In the original4a2 dedicated fixedSession (line73), predictive intercept
    // flags are excluded; the actual walk-out-of-ball-path veto remains.
    const bool intercept=theInterceptBallInformation.walkOutOfBallPath;
    const unsigned flowMask=(!freshFootBall?1u:0u)|(observationHeld?2u:0u)|(intercept?4u:0u)
        |(theRobotPose.quality==RobotPose::poor?8u:0u)|(!bodyInside?16u:0u)|(!ballInside?32u:0u)
        |(illegal?64u:0u)|(!fieldFresh?128u:0u);
    nativeFresh=flowMask==0;
    native4a2Mask=static_cast<int>(flowMask);
  }
  // Keep the normal long-ball RUN. Switch early only for a locally observed
  // pass that is falling short, while the ball is still in front of the runner.
  if(fixedRunToArea&&!fixedArrived&&autonomousShortCutEvent!=autonomousReceiveEvent)
  {
    const Vector2f ball=theBallModel.estimate.position;
    const Vector2f velocity=theBallModel.estimate.velocity;
    const unsigned seen=theBallModel.timeWhenLastSeen;
    bool shortOpportunity=false;
    if(nativeFresh&&age<=100u&&seen>=autonomousReceiveEvent&&seen>=theRobotPose.timestampLastJump&&
       !theMotionRequest.shouldWalkOutOfBallLine&&theGameControllerData.setPlay==SET_PLAY_NONE&&
       ball.x()>150.f&&ball.squaredNorm()<=sqr(1250.f)&&velocity.allFinite()&&velocity.squaredNorm()<=sqr(1500.f)&&
       std::isfinite(theBallSpecification.staticFriction)&&theBallSpecification.staticFriction<-.05f)
    {
      const Vector2f stop=theRobotPose*BallPhysics::getEndPosition(ball,velocity,theBallSpecification);
      const Vector2f onField=theRobotPose*ball;
      const auto inCutArea=[&](const Vector2f& p)
      {
        return p.allFinite()&&p.x()>theFieldDimensions.xPosHalfwayLine+700.f&&
               theFieldDimensions.isInsideField(p)&&!theIllegalAreas.isPositionIllegal(p,450.f);
      };
      if(stop.x()<=ballRef.x()-1100.f&&std::abs(stop.y()-ballRef.y())<=1200.f&&
         onField.x()>theRobotPose.translation.x()+100.f&&inCutArea(onField)&&inCutArea(stop)&&
         inCutArea((theRobotPose.translation+onField)*.5f))
      {
        bool hasPassedMarker=false,blocked=false;
        const auto inCorridor=[&](const Vector2f& p)
        {
          return p.x()>theRobotPose.translation.x()-3000.f&&p.x()<stop.x()+1600.f&&
                 p.y()>std::min(theRobotPose.translation.y(),onField.y())-900.f&&
                 p.y()<std::max(theRobotPose.translation.y(),onField.y())+900.f;
        };
        for(const auto& opponent:theGlobalOpponentsModel.opponents)
          if(opponent.position.allFinite()&&inCorridor(opponent.position))
          {
            if(opponent.position.x()>theRobotPose.translation.x()-500.f)blocked=true;
            else if(!opponent.unidentified)hasPassedMarker=true;
          }
        // Local fresh tracks can veto the tactical turn; native motion
        // avoidance remains active even when the tactical model permits it.
        for(const auto& obstacle:theObstacleModel.obstacles)
          if(!obstacle.isTeammate()&&obstacle.lastSeen<=now&&now-obstacle.lastSeen<=500u&&obstacle.center.allFinite())
          {
            const Vector2f p=theRobotPose*obstacle.center;
            if(inCorridor(p)&&p.x()>theRobotPose.translation.x()-500.f)blocked=true;
          }
        shortOpportunity=hasPassedMarker&&!blocked;
      }
    }
    if(!shortOpportunity)
      autonomousShortBallFirstSeen=0;
    else if(!autonomousShortBallFirstSeen||seen<autonomousShortBallFirstSeen||seen-autonomousShortBallFirstSeen>200u)
      autonomousShortBallFirstSeen=seen;
    else if(seen-autonomousShortBallFirstSeen>=65u)
    {
      autonomousShortCutEvent=autonomousReceiveEvent;
      logFlow(8,"short_ball_cut",theRobotPose*ball);
    }
  }
  const bool shortCut=fixedRunToArea&&autonomousShortCutEvent==autonomousReceiveEvent;
  const bool inReceivePhase=fixedArrived||shortCut;
  if(shortCut)
    autonomousRunOnlyThisFrame=false;
  const Vector2f ownBall=theBallModel.estimate.position;
  const Vector2f ownBallField=theRobotPose*ownBall;
  const bool ownLive=fixedRunToArea&&nativeFresh&&age<=150u&&
    theBallModel.timeWhenLastSeen>=autonomousReceiveEvent&&theBallModel.timeWhenLastSeen>=theRobotPose.timestampLastJump&&
    !theMotionRequest.shouldWalkOutOfBallLine&&theGameControllerData.setPlay==SET_PLAY_NONE;
  const auto legalLocalWalk=[&](const Vector2f& end)
  {
    if(!end.allFinite())return false;
    for(float f:{0.f,.25f,.5f,.75f,1.f})
    {
      const Vector2f p=theRobotPose.translation+(end-theRobotPose.translation)*f;
      if(!theFieldDimensions.isInsideField(p)||theIllegalAreas.isPositionIllegal(p,450.f))return false;
    }
    return true;
  };
  const auto localApproach=[&](const Vector2f& end,float facing)
  {
    const Vector2f local=theRobotPose.inverse()*end;
    if(local.squaredNorm()<=sqr(100.f)||local.squaredNorm()>sqr(800.f)||!legalLocalWalk(end))
      return false;
    const float fraction=std::clamp(ownBall.dot(local)/local.squaredNorm(),0.f,1.f);
    if((ownBall-local*fraction).squaredNorm()<std::min(sqr(250.f),ownBall.squaredNorm()))
      return false;
    autonomousRunOnlyThisFrame=true;
    WalkToPointObstacle({.target=Pose2f(facing,local),.speed={1.f,1.f,1.f},
      .reduceWalkSpeedType=ReduceWalkSpeedType::noChange,.disableObstacleAvoidance=false,.disableAligning=false,
      .targetOfInterest=ownBall});
    LookAtPoint({.target={ownBall.x(),ownBall.y(),theBallSpecification.radius}});
    return true;
  };
  const Vector2f goal(theFieldDimensions.xPosOpponentGoalLine,0.f);
  const bool terminal=ownLive&&ownBall.squaredNorm()<=sqr(1750.f)&&ownBall.x()>0.f&&
    ownBallField.x()>ballRef.x()-1500.f&&std::abs(ownBallField.y()-ballRef.y())<=1700.f&&
    (goal-ownBallField).x()>400.f;
  const Vector2f aim=goal-ownBallField;
  const bool behindForShot=terminal&&ownBallField.x()>theRobotPose.translation.x()+100.f&&
    std::abs(ownBallField.y()-theRobotPose.translation.y())<=480.f;
  bool closeDefender=false;
  if(ownLive&&ownBall.squaredNorm()<=sqr(1100.f)&&aim.x()>400.f)
    for(const auto& o:theObstacleModel.obstacles)
      if(!o.isTeammate()&&o.lastSeen>0&&o.lastSeen<=now&&now-o.lastSeen<=250u&&o.center.allFinite()&&
         (o.center-ownBall).squaredNorm()<=sqr(1500.f)&&(theRobotPose*o.center).x()>ownBallField.x())
        closeDefender=true;
  const auto nativeShotClear=[&]()
  {
    theKickAtGoalEvaluation.update();
    const auto& shot=theKickAtGoalEvaluation.kickInfo;
    if(!shot.isValid||!theIndirectKick.allowDirectKick)return false;
    if(!closeDefender)return true;
    const float heading=Angle::normalize(shot.kickDirection+theRobotPose.rotation);
    const Vector2f u(std::cos(heading),std::sin(heading));
    if(!u.allFinite()||u.x()<=0.f)return false;
    const float length=aim.x()/u.x();
    for(const auto& o:theObstacleModel.obstacles)
      if(o.lastSeen>0&&o.lastSeen<=now&&now-o.lastSeen<=250u&&o.center.allFinite())
      {
        const Vector2f p=theRobotPose*o.center-ownBallField;
        const float along=p.dot(u);
        if(along>0.f&&along<length)
        {
          const float radius=o.left.allFinite()&&o.right.allFinite()?
            std::clamp((o.left-o.right).norm()*.5f,180.f,450.f):300.f;
          if((p-u*along).norm()-radius-theBallSpecification.radius<100.f)return false;
        }
      }
    return true;
  };
  // SIM tune 2026-11-10 (user request): red2 ignores defenders, runs straight
  // to the predicted landing point of the live pass and enters the native shot
  // early. SIM-only, red team 5 / player 2, after the legal half release.
  const bool simLandingRun=red2Native&&playing&&!halfClosed&&red2EnhancedRunAllowed();
  if(simLandingRun)
  {
    if(simLandingEvent!=autonomousReceiveEvent)
    {
      simLandingEvent=autonomousReceiveEvent;simBlockedSince=0;
      simLanding=autonomousReceiveTarget.allFinite()?autonomousReceiveTarget:ballRef;
    }
    const bool ballUsable=theBallModel.timeWhenLastSeen>=autonomousReceiveEvent&&age<=300u&&
        theRobotPose.quality!=RobotPose::poor&&ownBall.allFinite()&&ownBallField.allFinite()&&
        theFieldDimensions.isInsideField(ownBallField);
    if(ballUsable)
    {
      // Predicted stop point of the live ball (native ball physics), else its position.
      Vector2f stop=ownBallField;
      const Vector2f v=theBallModel.estimate.velocity;
      if(v.allFinite()&&std::isfinite(theBallSpecification.staticFriction)&&theBallSpecification.staticFriction<-.05f)
        stop=theRobotPose*BallPhysics::getEndPosition(ownBall,v,theBallSpecification);
      if(stop.allFinite())
        simLanding={std::clamp(stop.x(),theFieldDimensions.xPosHalfwayLine+500.f,theFieldDimensions.xPosOpponentGoalLine-300.f),
                    std::clamp(stop.y(),theFieldDimensions.yPosRightTouchline+300.f,theFieldDimensions.yPosLeftTouchline-300.f)};
      // Early shot: fresh ball in front within 1.5 m -> native KickAtGoal path.
      // Only once the pass has slowed (<=1 m/s) or rolls toward red2, and is
      // >2.5 m into the opponent half: the just-kicked first pass rolling past
      // red2 at ~0.9 s must not start the shot (it made red2 side-step to
      // line up behind a ball rolling away at ~2.5 m/s).
      const Vector2f vRel=theBallModel.estimate.velocity;
      const bool ballSettling=vRel.allFinite()&&(vRel.squaredNorm()<=sqr(1000.f)||vRel.dot(ownBall)<0.f);
      if(!autonomousReceiveFinishing&&age<=150u&&ownBall.x()>0.f&&ownBall.squaredNorm()<=sqr(1500.f)&&
         ballSettling&&ownBallField.x()>theFieldDimensions.xPosHalfwayLine+2500.f&&
         theIndirectKick.allowDirectKick)
      {
        autonomousReceiveKickAtStart=theMotionInfo.lastKickTimestamp;
        autonomousReceiveFinishing=true;
        logFlow(20,"sim_early_shot",ownBallField);
      }
    }
    if(!autonomousReceiveFinishing)
    {
      const Vector2f toGoal=goal-simLanding;
      Vector2f body=simLanding-(toGoal.squaredNorm()>1.f?toGoal.normalized(330.f):Vector2f(330.f,0.f));
      // 2026-11-10 right-lane run: while still >2.5 m away, keep the run line at
      // y <= -1300 mm (right of the pass line). The early stop prediction drifts
      // toward the centre, where the defenders converge after the kick, and the
      // native obstacle avoidance then slowed red2 to <1 m/s and it lost the race.
      // Only the last 2.5 m go to the actual landing point. Avoidance stays on.
      if((body-theRobotPose.translation).squaredNorm()>sqr(2500.f))
        body.y()=std::min(body.y(),-1300.f);
      const Vector2f rel=theRobotPose.inverse()*body;
      if(rel.allFinite())
      {
        autonomousRunOnlyThisFrame=true;
        const float facing=rel.norm()>1200.f?rel.angle():Angle::normalize(toGoal.angle()-theRobotPose.rotation);
        WalkToPointObstacle({.target=Pose2f(facing,rel),.speed={1.f,1.f,1.f},
            .reduceWalkSpeedType=ReduceWalkSpeedType::noChange,.disableObstacleAvoidance=red2IgnoreObstacles(),.disableAligning=false,
            .targetOfInterest={}});
        if(ballUsable)LookAtPoint({.target={ownBall.x(),ownBall.y(),theBallSpecification.radius}});
        else LookActive({.withBall=true});
        logFlow(21,"sim_landing_run",body);
        return true;
      }
    }
  }
  // 4a2: a live ball already in the receiving feet's forward corridor
  // is the handoff, not reaching an extra field waypoint first.
  const bool liveBallHandoff=terminal&&!behindForShot&&!autonomousReceiveFinishing&&
    ownBallField.x()>theRobotPose.translation.x()+100.f&&
    ownBall.x()>=350.f&&ownBall.squaredNorm()<=sqr(1000.f)&&std::abs(ownBall.angle())<=35_deg&&
    theIndirectKick.allowDirectKick&&nativeShotClear();
  const bool near=(!fixedRunToArea||shortCut||(terminal?(behindForShot||liveBallHandoff):fixedArrived))&&(red2Native?nativeFresh:
      (freshFoot&&(nativeBallPlayer||(theRobotPose.translation-autonomousReceiveTarget).norm()<=2000.f)))&&
      ownBall.squaredNorm()<=sqr(1000.f);
  theBehaviorStatus.passOrigin=autonomousReceiveSender;
  // The 2.2 m limit remains for the old memory/local receive window, not
  // for an own, freshly observed ball that rolled forward past this area.
  // Use short native walking steps only; the original near-ball check
  // below still owns the one-way transition to a kick.
  if(fixedArrived&&ownLive&&!near&&!autonomousReceiveFinishing&&ownBall.x()>0.f&&
     ownBallField.x()>std::max(ballRef.x(),theRobotPose.translation.x()+100.f)&&
     (ownBall.squaredNorm()>sqr(2200.f)||(ownBallField-ballRef).squaredNorm()>sqr(2200.f))&&
     theFieldDimensions.isInsideField(ownBallField)&&!theIllegalAreas.isPositionIllegal(ownBallField,150.f))
  {
    const float step=std::min(790.f,ownBall.norm()-450.f);
    if(step>150.f)
    {
      const Vector2f end=theRobotPose*ownBall.normalized(step);
      if(localApproach(end,ownBall.angle()))
      {
        autonomousRunAnchorEvent=autonomousRunTrustedSince=0;
        logFlow(16,"terminal_live_overrun_chase",end);return true;
      }
    }
  }
  if(terminal&&!shortCut&&!near&&!autonomousReceiveFinishing)
  {
    // Aim for the rear side of a short-horizon intercept, not for the
    // fixed waypoint or the current centre of a moving ball.
    Vector2f intercept=ownBallField;
    const Vector2f v=theBallModel.estimate.velocity;
    if(v.allFinite()&&v.squaredNorm()<=sqr(1800.f)&&
       std::isfinite(theBallSpecification.staticFriction)&&theBallSpecification.staticFriction<-.05f)
      intercept=theRobotPose*BallPhysics::propagateBallPosition(ownBall,v,.25f,theBallSpecification);
    const Vector2f body=intercept-(goal-intercept).normalized(330.f);
    const Vector2f offset=body-theRobotPose.translation;
    if(offset.allFinite()&&offset.squaredNorm()>sqr(100.f))
    {
      const Vector2f end=theRobotPose.translation+offset.normalized(std::min(790.f,offset.norm()));
      if(localApproach(end,Angle::normalize(aim.angle()-theRobotPose.rotation)))
      {
        autonomousRunAnchorEvent=autonomousRunTrustedSince=0;
        logFlow(9,"terminal_receive_approach",end);return true;
      }
    }
  }
  if(near&&!autonomousReceiveFinishing)
  {
    if(fixedRunToArea)
      autonomousReceiveKickAtStart=theMotionInfo.lastKickTimestamp; // Only the newly entered native shot may end this phase.
    autonomousReceiveFinishing=true;
  }
  if(playing&&(autonomousReceiveFinishing||inReceivePhase))
  {
    autonomousRunAnchorEvent=0;
    // 4a2's one-way near-ball handoff, but no simulation forced-drive output:
    // never switch back to a far waypoint or optional PlayBall duel/dribble
    // because the team request changed. Native Goal/Indirect still decides
    // whether a shot is legal/available; a request is not a completed shot.
    // SIM tune 2026-11-10: in the landing run a 400 ms fresh own ball suffices to shoot
    // (observed ball ages of ~264 ms otherwise toggled shoot/stand right before the kick).
    const bool finishNow=autonomousReceiveFinishing&&(red2Native?(nativeFresh||(simLandingRun&&age<=400u)):freshFoot);
    if(finishNow)
    {
      // SIM tune 2026-11-10: at most one 600 ms reposition per kickoff, then shoot.
      const bool blockedBudget=!simLandingRun||!simBlockedSince||now-simBlockedSince<600u;
      if(closeDefender&&theIndirectKick.allowDirectKick&&blockedBudget&&!nativeShotClear())
      {
        if(simLandingRun&&!simBlockedSince)simBlockedSince=now;
        // Keep the native shot as the only kick path. If it is truly
        // unavailable under pressure, change position, not the kick target.
        Vector2f bestStep=theRobotPose.translation;
        float bestDistance=std::numeric_limits<float>::infinity();
        for(float side:{-1.f,1.f})
        {
          const Vector2f end=ownBallField+Vector2f(-380.f,side*350.f);
          const Vector2f move=end-theRobotPose.translation;
          if(move.squaredNorm()<=sqr(130.f)||move.squaredNorm()>sqr(790.f)||!legalLocalWalk(end))continue;
          const Vector2f rel=ownBallField-theRobotPose.translation;
          const float t=std::clamp(rel.dot(move)/move.squaredNorm(),0.f,1.f);
          if((rel-move*t).squaredNorm()<sqr(250.f))continue;
          bool clear=true;
          for(const auto& o:theObstacleModel.obstacles)
            if(o.lastSeen>0&&o.lastSeen<=now&&now-o.lastSeen<=250u&&o.center.allFinite()&&
               (end-theRobotPose*o.center).squaredNorm()<sqr(520.f))clear=false;
          if(clear&&move.squaredNorm()<bestDistance)
          {bestStep=end;bestDistance=move.squaredNorm();}
        }
        if(localApproach(bestStep,Angle::normalize(aim.angle()-theRobotPose.rotation)))
        {logFlow(11,"terminal_blocked_reposition",bestStep);return true;}
        autonomousRunOnlyThisFrame=true;
        LookActive({.withBall=true});Stand();
        logFlow(12,"terminal_blocked_hold",ownBallField);return true;
      }
      KickAtGoal();
      if(liveBallHandoff){logFlow(15,"terminal_4a2_live_ball_handoff",ownBallField);return true;}
    }
    else
    {
      // Arrival ends the fixed RUN, not the approach to the ball. Use the
      // native, already propagated estimate for a short local walk only;
      // never use it to latch finishing or issue a kick on stale vision.
      const Vector2f ball=theBallModel.estimate.position;
      if(inReceivePhase&&!autonomousReceiveFinishing&&!observationHeld&&
         !theInterceptBallInformation.walkOutOfBallPath&&!theMotionRequest.shouldWalkOutOfBallLine&&
         theGameControllerData.setPlay==SET_PLAY_NONE&&theRobotPose.quality!=RobotPose::poor&&
         theFieldDimensions.isInsideField(theRobotPose.translation)&&
         theBallModel.timeWhenLastSeen>=autonomousReceiveEvent&&theRobotPose.timestampLastJump<=theBallModel.timeWhenLastSeen&&
         age<=800u&&ball.allFinite()&&ball.squaredNorm()<=sqr(2200.f))
      {
        const Vector2f ballOnField=theRobotPose*ball;
        const float distance=ball.norm();
        const float step=std::min(800.f,distance-450.f);
        if(ballOnField.allFinite()&&(ballOnField-ballRef).squaredNorm()<=sqr(2200.f)&&
           theFieldDimensions.isInsideField(ballOnField)&&!theIllegalAreas.isPositionIllegal(ballOnField,150.f))
        {
          autonomousRunOnlyThisFrame=true;
          if(step>150.f)
          {
            const Vector2f local=ball.normalized(step);
            const Vector2f end=theRobotPose*local;
            bool clear=true;
            for(float f : {0.f,.25f,.5f,.75f,1.f})
            {
              const Vector2f p=theRobotPose.translation+(end-theRobotPose.translation)*f;
              clear=clear&&theFieldDimensions.isInsideField(p)&&!theIllegalAreas.isPositionIllegal(p,450.f);
            }
            if(clear)
            {
              WalkToPointObstacle({.target=Pose2f(local.angle(),local),.speed={1.f,1.f,1.f},
                  .reduceWalkSpeedType=ReduceWalkSpeedType::noChange,.disableObstacleAvoidance=false,.disableAligning=false,
                  .targetOfInterest={}});
              LookAtPoint({.target={ball.x(),ball.y(),theBallSpecification.radius}});
              logFlow(7,nativeFresh?"arrived_approach_fresh_ball":"arrived_approach_ball_memory",end);
              return true;
            }
          }
        }
      }
      if(fixedRunToArea)
        autonomousRunOnlyThisFrame=true; // A remembered ball cannot start the shot.
      LookActive({.withBall=true});Stand();
    }
    logFlow(3,finishNow?"native_finish":"native_finish_fresh_ball_hold",autonomousReceiveTarget);return true;
  }
  // 4a2:143-144. Decision port only: keep the shared pass P intact.
  if(halfClosed)
  {
    autonomousRunOnlyThisFrame=!theBehaviorParameters.c15ReceiverSimMaxRun;
    const Vector2f stage=prepareAutonomousKickoffSupport(target);
    logFlow(4,"legal_parallel_prepare_observe",stage);return true;
  }
  const Vector2f navigationTarget=guideTarget(theRobotPose.translation);
  const Vector2f relative=theRobotPose.inverse()*navigationTarget;
  const bool simDirect=red2Native&&WhiteFleetSimPhysical::simulated()&&
      Global::getSettings().teamNumber==5&&Global::getSettings().playerNumber==2&&
      theBehaviorParameters.c15ReceiverSimMaxRun&&!theInterceptBallInformation.walkOutOfBallPath&&
      !theMotionRequest.shouldWalkOutOfBallLine;
  if(simDirect)
  {
    // Original4a2:209-220 SIM-only output. Event/frame are the real receive
    // event and current cognition, not a forged C15 action or release.
    PublishMotion({.target=relative,.speed={1.f,1.f,1.f}});
    WalkToPose({.target={relative.angle(),relative},.speed={1.f,1.f,1.f},.obstacleAvoidance={},
        .keepTargetRotation=true,.targetOfInterest={},.sideWalkingRequest=SideWalkingRequest::allowed});
    theMotionRequest.shouldInterceptBall=false;
    theMotionRequest.fixedRunDriveEvent=autonomousReceiveEvent;
    theMotionRequest.fixedRunDriveFrame=now;
    theMotionRequest.fixedRunDriveSpeed=std::min(2000.f,theWalkingEngineOutput.maxSpeed.translation.x());
    theMotionRequest.fixedRunDriveCoordinateVersion=2;
  }
  else
  {
    if(canAnchor)
    {
      if(!autonomousRunTrustedSince)autonomousRunTrustedSince=now;
      autonomousRunAnchorPose=theRobotPose;
      autonomousRunAnchorOdo=theOdometryData;
      autonomousRunAnchorEvent=autonomousReceiveEvent;
      autonomousRunAnchorTime=autonomousRunProgressAt=now;
      autonomousRunAnchorJump=theRobotPose.timestampLastJump;
      autonomousRunOdoProgress=Vector2f::Zero();
    }
    else
      autonomousRunAnchorEvent=autonomousRunTrustedSince=0;
    WalkToPointObstacle({.target=Pose2f(relative.angle(),relative),.speed={1.f,1.f,1.f},
        .reduceWalkSpeedType=ReduceWalkSpeedType::noChange,.disableObstacleAvoidance=false,.disableAligning=false,
        .targetOfInterest=fixedRunToArea?std::optional<Vector2f>{}:std::optional<Vector2f>{theFieldBall.recentBallPositionRelative()}});
  }
  if(fixedRunToArea)
    LookAtPoint({.target={relative.x(),relative.y(),theBallSpecification.radius}});
  else if(red2Native)
  {
    // 4a2:212-213 head decision, unchanged for either selected motion output.
    const Vector2f head=freshFootBall&&!observationHeld?theBallModel.estimate.position:theRobotPose.inverse()*ballRef;
    LookAtPoint({.target={head.x(),head.y(),theBallSpecification.radius}});
  }
  else LookAtBallAndTarget({.startBall=true,.walkingDirection=relative});
  logFlow(5,simDirect?"SIM_ONLY_4a2_direct_run_v2":"original_receiving_target_native_walk",navigationTarget);
  return true;
}

bool WhiteFleetSkillBehaviorControl::handleC15Kickoff()
{
  if(theGameState.state!=GameState::ownKickOff&&theGameState.state!=GameState::playing)
    autonomousReceiveEvent=0; // A new set play cannot inherit the old run.
  if(WhiteFleetAutonomousKickoff::enabled(WhiteFleetSimPhysical::physical(),
       WhiteFleetK1V21MatchProfile::guardedPhysicalReceiver(),WhiteFleetSimPhysical::simulated(),
       Global::getSettings().teamNumber,std::getenv("MDFZ_AUTONOMOUS_KICKOFF_V4_ENABLE"))
     &&(theGameState.state==GameState::ownKickOff||theGameState.state==GameState::playing))
  {
    maintainAutonomousKickoffContext(false);
    // Strategy and DirectKickOff share the designated kicker; only2 can bind
    // reception from3. A reverse/already-bound intent cannot swallow3's shoot.
    // Preserve the original dispatcher and all later native guards.
    // Keep READY/SET preparation and all non-enrolled modes byte-for-byte below.
    static thread_local unsigned lastDispatchLog=0;
    if(!theGameState.isGoalkeeper()&&(!lastDispatchLog||theFrameInfo.time<lastDispatchLog||
       theFrameInfo.getTimeSince(lastDispatchLog)>=1000))
    {
      lastDispatchLog=theFrameInfo.time;
      std::printf("NATIVE_KICKOFF_DISPATCH self=%d state=%u skill=%u legacy_c15_bypassed=1 physical=%d\n",
          theGameState.playerNumber,static_cast<unsigned>(theGameState.state),
          static_cast<unsigned>(theSkillRequest.skill),WhiteFleetSimPhysical::physical());
    }
    if(handleKickerPostKickHold())return true;
    if(handleAutonomousKickoffReceive())return true;
    return false;
  }
  if(!WhiteFleetC15KickoffSwitch::enabled(theBehaviorParameters.matchTacticalRepair&&theBehaviorParameters.c15Kickoff)||
     theGameState.isGoalkeeper())return false;
  const bool own=theGameState.state==GameState::ownKickOff&&theGameState.isForOwnTeam();
  if(own)
  {
    // One entry only. Missing/ended requests cannot fall back to DirectKickOff.
    if(theSkillRequest.kickoffThrough&&!theSkillRequest.isolatedKickoffFinished&&
       theSkillRequest.kickoffThroughEvent==theGameState.timeWhenStateStarted)
      KickoffThroughPassV8();
    else {LookActive({.withBall=true});Stand();}
    return true;
  }
  if(theSkillRequest.kickoffThrough){KickoffThroughPassV8();return true;}
  return false;
}

bool WhiteFleetSkillBehaviorControl::suppressC15Receive(const Teammate& teammate) const
{
  if(WhiteFleetAutonomousKickoff::enabled(WhiteFleetSimPhysical::physical(),
       WhiteFleetK1V21MatchProfile::guardedPhysicalReceiver(),WhiteFleetSimPhysical::simulated(),
       Global::getSettings().teamNumber,std::getenv("MDFZ_AUTONOMOUS_KICKOFF_V4_ENABLE")))
  {
    if(theGameState.state==GameState::ownKickOff)return true; // Only the fresh addressed path above owns kickoff reception.
    if(theGameState.state==GameState::playing&&autonomousReceiveFinishedEvent&&
       teammate.number==autonomousReceiveSender&&teammate.theFrameInfo.time>=autonomousReceiveFinishedEvent&&
       teammate.theFrameInfo.time-autonomousReceiveFinishedEvent<=11000u&&
       teammate.theBehaviorStatus.shootingTo.has_value()&&
       (teammate.theRobotPose*teammate.theBehaviorStatus.shootingTo.value()-autonomousReceiveTarget).norm()<=300.f)
      return true; // Do not resurrect a completed run through the old 10s ReceivePass continuation.
  }
  return WhiteFleetC15KickoffSwitch::enabled(theBehaviorParameters.matchTacticalRepair&&theBehaviorParameters.c15Kickoff)&&
    theSkillRequest.kickoffThroughRetired&&theGameState.state==GameState::playing&&
    !theGameState.stopped&&!theGameState.isGoalkeeper()&&
    theGameState.playerNumber==theSkillRequest.kickoffThroughReceiver&&
    teammate.number==theSkillRequest.kickoffThroughKicker&&
    theSkillRequest.kickoffThroughKicker!=theSkillRequest.kickoffThroughReceiver&&
    theSkillRequest.kickoffThroughEvent>0&&theSkillRequest.kickoffThroughEvent<=theFrameInfo.time&&
    theFrameInfo.time-theSkillRequest.kickoffThroughEvent<=WhiteFleetC15KickoffSwitch::throughParams().total+10000u&&
    theSkillRequest.kickoffThroughTarget.allFinite()&&
    (!theSkillRequest.isolatedKickoffFinished||!teammate.theBehaviorStatus.shootingTo.has_value()||
      (teammate.theRobotPose*teammate.theBehaviorStatus.shootingTo.value()-theSkillRequest.kickoffThroughTarget).norm()<=300.f);
}

bool WhiteFleetSkillBehaviorControl::handleGoalKickObservation()
{
  const unsigned now=theFrameInfo.time,epoch=theSkillRequest.restartPassEpoch;
  const bool allowed=WhiteFleetMatchTacticalRepair::enabled(theBehaviorParameters.matchTacticalRepair)&&
    theGameState.isGoalkeeper()&&theGameState.isGoalKick()&&theGameState.isForOwnTeam()&&
    theSkillRequest.skill==SkillRequest::stand&&theSkillRequest.restartPassRequired&&theSkillRequest.restartPassGoalKick&&
    theSkillRequest.restartPassPasser==theGameState.playerNumber&&theSkillRequest.restartPassReason==WhiteFleetMatchTacticalRepair::waitObservation&&
    epoch==theGameState.timeWhenStateStarted&&!theGameState.stopped&&theGameControllerData.stopped==0&&
    theGameState.playerState==GameState::active&&!theBehaviorStateInformation.remoteControlActive&&!WhiteFleetBoosterPickupControl::held()&&
    !theSystemSensorData.emergencyMode&&!theEmergencyButtonState.buttonPressed&&theFallDownState.state==FallDownState::upright&&
    (theMotionInfo.executedPhase==MotionPhase::walk||theMotionInfo.executedPhase==MotionPhase::stand)&&
    theRobotPose.isFinite()&&theRobotPose.quality!=RobotPose::poor&&theCameraMatrix.isValid&&
    theBallModel.estimate.position.allFinite()&&std::isfinite(theFieldDimensions.xPosOwnPenaltyArea)&&
    std::isfinite(theFieldDimensions.yPosLeftPenaltyArea)&&std::isfinite(theBallSpecification.radius)&&theBallSpecification.radius>0.f&&
    theInertialData.angle.allFinite()&&theInertialData.gyro.allFinite()&&
    theInertialData.angle.head<2>().norm()<.22f&&theInertialData.gyro.norm()<2.5f;
  const bool currentBall=epoch&&theBallModel.estimate.position.allFinite()&&theBallModel.timeWhenLastSeen>=epoch&&theBallModel.timeWhenLastSeen<=now&&
    now-theBallModel.timeWhenLastSeen<=200u;
  const auto search=goalObservationSearch.update(now,epoch,allowed,currentBall,theRobotPose.rotation);
  if(!search.active)return false;
  bool turn=search.body&&theRobotPose.translation.x()>=theFieldDimensions.xPosOwnGoalLine+300.f&&
    std::abs(theRobotPose.translation.y())<=theFieldDimensions.yPosLeftTouchline-500.f&&
    WhiteFleetMatchTacticalRepair::validKeeperLimits(theBehaviorParameters.keeperPursuitOutsidePenalty,theBehaviorParameters.keeperPursuitBuffer)&&
    WhiteFleetMatchTacticalRepair::keeperMargin({theRobotPose.translation.x(),theRobotPose.translation.y()},theFieldDimensions.xPosOwnPenaltyArea,
      theFieldDimensions.yPosLeftPenaltyArea,theBehaviorParameters.keeperPursuitOutsidePenalty)>=theBehaviorParameters.keeperPursuitBuffer;
  const bool knownNear=theBallModel.timeWhenLastSeen>=epoch&&theBallModel.timeWhenLastSeen<=now&&
    theBallModel.estimate.position.allFinite()&&theBallModel.estimate.position.norm()<1000.f;
  turn=turn&&!knownNear; // Do not pivot feet over a last-known near ball just because its timestamp aged.
  for(const Obstacle& o:theObstacleModel.obstacles)
    if(!o.center.allFinite()||o.lastSeen>now||o.center.norm()<600.f)turn=false;
  if(turn)WalkAtRelativeSpeed({.speed=Pose2f(search.rotation,0.f,0.f)});
  else Stand();
  float side=theRobotPose.translation.y()>=0.f?1.f:-1.f;
  if(search.otherSide)side=-side;
  const Vector2f look=knownNear&&now-theBallModel.timeWhenLastSeen<=3000u?theBallModel.estimate.position:
    theRobotPose.inverse()*Vector2f(theFieldDimensions.xPosOwnPenaltyArea,side*theFieldDimensions.yPosLeftPenaltyArea);
  LookAtPoint({.target={look.x(),look.y(),theBallSpecification.radius},.speed=120_deg});
  return true; // Region search only: no translated chase, fresh-ball fabrication or kick.
}

void WhiteFleetSkillBehaviorControl::executeRequest()
{
  if(handleC15Kickoff())return;
  if(handleGoalKickObservation())return; // Called only below the original physical/manual/referee gates.
  if(handleRestartReceive())return; // Restart-only receiver lease beats a transient native playBall reassignment.
  if(theSkillRequest.skill == SkillRequest::none)
  {
    if(theFrameInfo.getTimeSince(timeWhenAnnouncedEmptySkillRequest) > 5000)
    {
      ANNOTATION("WhiteFleetBehavior", "Skill request is empty");
      SystemCall::say("Skill request is empty");
      timeWhenAnnouncedEmptySkillRequest = theFrameInfo.time;
    }
  }
  else if(theSkillRequest.skill == SkillRequest::shoot ||
          theSkillRequest.skill == SkillRequest::pass ||
          theSkillRequest.skill == SkillRequest::dribble ||
          theSkillRequest.skill == SkillRequest::clear)
  {
    PlayBall();
  }
  else if(theSkillRequest.skill == SkillRequest::waitShoot)
    GoToBallWaitAndKick();
  else
  {
    // Check if a pass is communicated
    if(theSkillRequest.kickoffThroughRetired&&receivePassPlayerNumber==theSkillRequest.kickoffThroughKicker)
    {lastReceivePassRequestTimestamp=0;receivePassPlayerNumber=-1;}
    for(const Teammate& teammate : theTeamData.teammates)
    {
      if(suppressC15Receive(teammate))continue;
      if(teammate.theBehaviorStatus.passTarget == theGameState.playerNumber)
      {
        // Communicated message is not too old
        if(theFrameInfo.getTimeSince(teammate.theFrameInfo.time) < ignoreReceivePassAfterTime)
        {
          lastReceivePassRequestTimestamp = theFrameInfo.time;
          receivePassPlayerNumber = teammate.number;
          theBehaviorStatus.passOrigin = teammate.number;
          ReceivePass({.playerNumber = teammate.number});
          return;
        }
        else // Communicated message is too old
        {
          lastReceivePassRequestTimestamp = 0;
          receivePassPlayerNumber = -1;
        }
      }
    }
    // Continue ReceivePass after communication stopped. Assume the sender just kicked the ball
    if(theFrameInfo.getTimeSince(lastReceivePassRequestTimestamp) < continueReceivePassTime && receivePassPlayerNumber != -1)
    {
      theBehaviorStatus.passOrigin = receivePassPlayerNumber;
      ReceivePass({.playerNumber = receivePassPlayerNumber});
      return;
    }
    receivePassPlayerNumber = -1;

    // slow head movements of LookActive down if the ball is far away
    isSlowingDownLookActive = (theFieldBall.positionOnField - theRobotPose.translation).squaredNorm() > sqr(2500.f + (isSlowingDownLookActive ? 0.f : 300.f));

    switch(theSkillRequest.skill)
    {
      case SkillRequest::stand:
        Stand();
        LookActive({.withBall = true,
                    .slowdownFactor = isSlowingDownLookActive ? 0.4f : 1.f});
        break;
      case SkillRequest::walk:
      {
        const Pose2f targetPose = theRobotPose.inverse() * theSkillRequest.target;
        const ReduceWalkSpeedType::ReduceWalkSpeedType reduceWalkSpeedType = theGameState.isGoalkeeper() ? ReduceWalkSpeedType::distanceBased : ReduceWalkSpeedType::normal;

        if(theFieldBall.ballWasSeen(7000) || theTeamBallModel.isValid)
        {
          WalkToPointObstacle({.target = theRobotPose.inverse() * theSkillRequest.target,
                               .reduceWalkSpeedType = reduceWalkSpeedType,
                               .disableObstacleAvoidance = theGameState.isGoalkeeper() && (!theGameState.isGoalKick() || !theGameState.isForOwnTeam()),
                               .targetOfInterest = theFieldBall.recentBallPositionRelative()}); // TODO: set the right parameters

          if(theMotionInfo.isMotion(MotionPhase::stand))
            LookActive({.withBall = true,
                        .slowdownFactor = isSlowingDownLookActive ? 0.4f : 1.f});
          else
            LookAtBallAndTarget({.startBall = true,
                                 .walkingDirection = targetPose.translation});
        }
        else
        {
          WalkToPointObstacle({.target = theRobotPose.inverse() * theSkillRequest.target,
                               .reduceWalkSpeedType = reduceWalkSpeedType,
                               .disableObstacleAvoidance = theGameState.isGoalkeeper()}); // TODO: set the right parameters
          LookActive();
        }
        break;
      }
      case SkillRequest::block:
        Block({.target = theRobotPose.inverse() * theSkillRequest.target.translation});
        break;
      case SkillRequest::mark:
        Mark({.target = theRobotPose.inverse() * theSkillRequest.target.translation});
        break;
      case SkillRequest::observe:
        ObservePoint({.target = theRobotPose.inverse() * theSkillRequest.target.translation});
        break;
      default:
        FAIL("Unknown skill request.");
    }
  }
}

void WhiteFleetSkillBehaviorControl::handleWalkSpeedAtFieldBorder()
{
  // TODO Special Handling for KickIns/CornerKicks
  Vector2f robotPose = theRobotPose.translation;
  const float clipRatio = Rangef::ZeroOneRange().limit(theFieldDimensions.clipToField(robotPose, fieldBorderOffset) / outSideFieldBorderRange);
  Pose2f maxWalkSpeed(std::min(std::abs(maxWalkSpeedAtFieldBorder.rotation / theWalkingEngineOutput.maxSpeed.rotation), std::abs(theMotionRequest.walkSpeed.rotation)) * (std::signbit(static_cast<float>(theMotionRequest.walkSpeed.rotation)) ? -1.f : 1.f),
                      std::min(std::abs(maxWalkSpeedAtFieldBorder.translation.x() / theWalkingEngineOutput.maxSpeed.translation.x()), std::abs(theMotionRequest.walkSpeed.translation.x())) * (std::signbit(theMotionRequest.walkSpeed.translation.x()) ? -1.f : 1.f),
                      std::min(std::abs(maxWalkSpeedAtFieldBorder.translation.y() / theWalkingEngineOutput.maxSpeed.translation.y()), std::abs(theMotionRequest.walkSpeed.translation.y())) * (std::signbit(theMotionRequest.walkSpeed.translation.y()) ? -1.f : 1.f));

  theMotionRequest.walkSpeed.translation = clipRatio * maxWalkSpeed.translation + (1.f - clipRatio) * theMotionRequest.walkSpeed.translation;
  theMotionRequest.walkSpeed.rotation = clipRatio * maxWalkSpeed.rotation + (1.f - clipRatio) * theMotionRequest.walkSpeed.rotation;
}
