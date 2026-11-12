/**
 * @file WhiteFleetSkillBehaviorControl.h
 *
 * This file declares the behavior control for the skill layer.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Math/Random.h"
#include "WhiteFleetC15ReceiverRoute.h"
#include "WhiteFleetC15EarlyChannel.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Representations/BehaviorControl/ActivationGraph.h"
#include "Representations/BehaviorControl/BehaviorStateInformation.h"
#include "Representations/BehaviorControl/BehaviorStatus.h"
#include "Representations/BehaviorControl/ClearTarget.h"
#include "Representations/BehaviorControl/DribbleTarget.h"
#include "Representations/BehaviorControl/ExpectedGoals.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/FieldInterceptBall.h"
#include "Representations/BehaviorControl/InterceptBallInformation.h"
#include "Representations/BehaviorControl/FieldRating.h"
#include "Representations/BehaviorControl/IndirectKick.h"
#include "Representations/BehaviorControl/IllegalAreas.h"
#include "Representations/BehaviorControl/KickAtGoalEvaluation.h"
#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Libraries/LibLookActive.h"
#include "Representations/BehaviorControl/Libraries/LibPosition.h"
#include "Representations/BehaviorControl/Libraries/LibWalk.h"
#include "Representations/BehaviorControl/PathPlanner.h"
#include "Representations/BehaviorControl/PassEvaluation.h"
#include "Representations/BehaviorControl/PassTargetEvaluation.h"
#include "Representations/BehaviorControl/SkillRequest.h"
#include "Representations/BehaviorControl/StrategyStatus.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/GameControllerData.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/BehaviorParameters.h"
#include "Representations/Configuration/CalibrationRequest.h"
#include "Representations/Configuration/CameraCalibration.h"
#include "Representations/Configuration/CameraCalibrationStatus.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/HeadLimits.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Configuration/RobotDimensions.h"
#include "Representations/Infrastructure/CameraInfo.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Infrastructure/JointAngles.h"
#include "Representations/Infrastructure/JoystickState.h"
#include "Representations/Infrastructure/RobotHealth.h"
#include "Representations/Infrastructure/EmergencyButtonState.h"
#include "Representations/Infrastructure/SensorData/KeyStates.h"
#include "Representations/Infrastructure/SensorData/SystemSensorData.h"
#include "Representations/Modeling/BallLostModel.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/GlobalTeammatesModel.h"
#include "Representations/Modeling/GlobalOpponentsModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/MotionControl/ArmMotionInfo.h"
#include "Representations/MotionControl/ArmMotionRequest.h"
#include "Representations/MotionControl/HeadMotionInfo.h"
#include "Representations/MotionControl/HeadMotionRequest.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Perception/ObstaclePercepts/RobotPercept.h"
#include "Representations/Perception/BallPercepts/BallPercept.h"
#include "Representations/Perception/ImagePreprocessing/CameraMatrix.h"
#include "Representations/Sensing/ArmContactModel.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Sensing/InertialData.h"
#include "Tools/BehaviorControl/HeadOrientation.h"
#include "Tools/BehaviorControl/Strategy/PositionRole.h"
#include "Tools/Motion/ReduceWalkSpeedType.h"
#include "Tools/BehaviorControl/SideWalkingRequest.h"
#include "WhiteFleetPostKickReacquire.h"
#include "WhiteFleetMatchTacticalRepair.h"
#include "WhiteFleetGoalKickExecution.h"
#include "Debugging/Annotation.h"
#include "Framework/Module.h"
#include "Platform/SystemCall.h"
#include "WhiteFleetBoosterPickupControl.h"
#include <regex> // not needed in header, but would otherwise be broken by CABSL
#include "Tools/BehaviorControl/Cabsl.h"

#ifndef FULL_HEADER
#include "SpeedUp.h"
#endif

class WhiteFleetSkillBehaviorControl;

MODULE(WhiteFleetSkillBehaviorControl,
{,
  REQUIRES(ArmContactModel),
  REQUIRES(ArmMotionInfo),
  REQUIRES(BallLostModel),
  REQUIRES(BallModel),
  REQUIRES(BallPercept),
  REQUIRES(BallSpecification),
  REQUIRES(BehaviorParameters),
  REQUIRES(CameraCalibration),
  REQUIRES(CameraCalibrationStatus),
  REQUIRES(CameraInfo),
  REQUIRES(CameraMatrix),
  REQUIRES(ClearTarget),
  REQUIRES(DribbleTarget),
  REQUIRES(EmergencyButtonState),
  REQUIRES(EnhancedKeyStates),
  REQUIRES(ExpectedGoals),
  REQUIRES(ExtendedGameState),
  REQUIRES(FallDownState),
  REQUIRES(FieldBall),
  REQUIRES(FieldInterceptBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FieldRating),
  REQUIRES(FrameInfo),
  REQUIRES(GameControllerData),
  REQUIRES(GameState),
  REQUIRES(GlobalTeammatesModel),
  REQUIRES(GlobalOpponentsModel),
  REQUIRES(HeadLimits),
  REQUIRES(HeadMotionInfo),
  REQUIRES(IllegalAreas),
  REQUIRES(IndirectKick),
  REQUIRES(InertialData),
  REQUIRES(InterceptBallInformation),
  REQUIRES(JointAngles),
  REQUIRES(JoystickState),
  REQUIRES(KeyStates),
  REQUIRES(KickAtGoalEvaluation),
  REQUIRES(KickInfo),
  REQUIRES(LibCheck),
  REQUIRES(LibLookActive),
  REQUIRES(LibPosition),
  REQUIRES(LibWalk),
  REQUIRES(MotionInfo),
  REQUIRES(RobotPercept),
  REQUIRES(ObstacleModel),
  REQUIRES(OdometryData),
  REQUIRES(PassEvaluation),
  REQUIRES(PassTargetEvaluation),
  REQUIRES(PathPlanner),
  REQUIRES(RobotDimensions),
  REQUIRES(RobotHealth),
  REQUIRES(RobotPose),
  REQUIRES(SkillRequest),
  REQUIRES(StrategyStatus),
  REQUIRES(SystemSensorData),
  REQUIRES(TeamData),
  REQUIRES(TeamBallModel),
  REQUIRES(WalkingEngineOutput),
  PROVIDES(ActivationGraph),
  REQUIRES(ActivationGraph),
  PROVIDES(ArmMotionRequest),
  PROVIDES(BehaviorStateInformation),
  PROVIDES(BehaviorStatus),
  PROVIDES(CalibrationRequest),
  PROVIDES(HeadMotionRequest),
  PROVIDES(MotionRequest),
  LOADS_PARAMETERS(
  {,
    (cabsl::Cabsl<WhiteFleetSkillBehaviorControl>::OptionInfos::Option) root, /**< The root option. */
    (int) continueReceivePassTime, /**< If the teammate no longer communicates a pass, continue receive pass for this time. */
    (int) ignoreReceivePassAfterTime, /**< If the communicated pass is too old, ignore it. */
    (Pose2f) maxWalkSpeedAtFieldBorder, /**< Max walk speed outside of the field. */
    (float) outSideFieldBorderRange, /**< Scale down walk speed over this distance outside the field. */
    (float) fieldBorderOffset, /**< Offset to field border to start scaling down speed. */
  }),
});

class WhiteFleetSkillBehaviorControl : public WhiteFleetSkillBehaviorControlBase, public cabsl::Cabsl<WhiteFleetSkillBehaviorControl>
{
public:
  /** Constructor. */
  WhiteFleetSkillBehaviorControl();

private:
  /**
   * Updates the activation graph.
   * @param activationGraph The provided activation graph.
   */
  void update(ActivationGraph& activationGraph) override;

  /**
   * Updates the arm motion request.
   * @param armMotionRequest The provided arm motion request.
   */
  void update(ArmMotionRequest& armMotionRequest) override { armMotionRequest = theArmMotionRequest; }

  /**
   * Updates the behavior state information.
   * @param behaviorStateInformation The provided behavior state information.
   */
  void update(BehaviorStateInformation& behaviorStateInformation) override
  {
    behaviorStateInformation = theBehaviorStateInformation;
  };

  /**
   * Updates the behavior status.
   * @param behaviorStatus The provided behavior status.
   */
  void update(BehaviorStatus& behaviorStatus) override { behaviorStatus = theBehaviorStatus; }

  /**
   * Updates the camera calibration request.
   * @param cameraCalibrationRequest The provided camera calibration request.
   */
  void update(CalibrationRequest& calibrationRequest) override { calibrationRequest = theCalibrationRequest; }

  /**
   * Updates the head motion request.
   * @param headMotionRequest The provided head motion request.
   */
  void update(HeadMotionRequest& headMotionRequest) override { headMotionRequest = theHeadMotionRequest; }

  /**
   * Updates the motion request.
   * @param motionRequest The provided motion request.
   */
  void update(MotionRequest& motionRequest) override { motionRequest = theMotionRequest; }

  unsigned lastReceivePassRequestTimestamp = 0;
  int receivePassPlayerNumber = -1;
  bool isSlowingDownLookActive = false;
  unsigned autonomousReceiveEvent=0,autonomousReceiveFinishedEvent=0,autonomousReceiveKickAtStart=0;
  int autonomousReceiveSender=-1;
  Vector2f autonomousReceiveTarget=Vector2f::Zero();
  unsigned autonomousReceiveDiagnosticEvent=0,autonomousReceiveLastLog=0,autonomousReceiveLogCount=0;
  int autonomousReceiveLastMode=-1;
  bool autonomousReceiveFinishing=false;
  unsigned autonomousReceiveContactSeenAt=0;
  // SIM tune 2026-11-10: per-event state of the landing run / decisive shot.
  unsigned simLandingEvent=0,simBlockedSince=0;
  unsigned kickerHoldEvent=0,kickerHoldBaseline=0,kickerKickAt=0,kickerHoldLog=0; // 2026-11-10 kicker post-kick hold
  Vector2f simLanding=Vector2f::Zero();
  Pose2f autonomousRunAnchorPose,autonomousRunAnchorOdo;
  Vector2f autonomousRunOdoProgress=Vector2f::Zero();
  unsigned autonomousRunAnchorEvent=0,autonomousRunAnchorTime=0,autonomousRunAnchorJump=0;
  unsigned autonomousRunBridgeEvent=0,autonomousRunBridgeAnchor=0,autonomousRunProgressAt=0;
  unsigned autonomousRunTrustedSince=0,autonomousRunBridgeWindows=0,autonomousRunBridgeMs=0,autonomousRunBridgeTick=0;
  unsigned autonomousRunArrivedEvent=0;
  unsigned autonomousShortCutEvent=0,autonomousShortBallFirstSeen=0;
  bool autonomousRunOnlyThisFrame=false;
  unsigned autonomousKickoffObservedEvent=0;
  void maintainAutonomousKickoffContext(bool retireActive);
  bool handleKickerPostKickHold(); // 2026-11-10: kicker never plays its own kickoff pass again for 10 s
  void publishAutonomousKickoffIntent();
  Vector2f prepareAutonomousKickoffSupport(Vector2f desired);
  bool handleAutonomousKickoffReceive();

protected:
  // CABSL vars reset after one inactive cycle. Keep the event intent on this
  // module; protected also permits the generated derived option wrapper.
  unsigned autonomousEvent=0;
  Vector2f autonomousTarget=Vector2f::Zero();
  float autonomousLength=0.f;
  int autonomousReceiver=-1;
  /** Executes the skill request. */
  void executeRequest();

  /**< Clips walk speed near field border. */
  void handleWalkSpeedAtFieldBorder();

  unsigned timeWhenAnnouncedEmptySkillRequest = 0; /**< The last time when the robot said that its skill request is empty. */
  unsigned lastBallOutOfFieldWaitTimestamp = 0; /**< The last time HandleBallOutOfField was active. */
  WhiteFleetBoosterPickupControl::Gate physicalPickupGate; /**< Survives CABSL initial-state fallbacks. */
  bool physicalPickupVisited = false;
  WhiteFleetPostKickReacquire::State postKickReacquire; /**< Persistent across CABSL fallbacks. */
  bool postKickPlayingVisited = false;
  WhiteFleetPostKickReacquire::Input postKickInput;
  WhiteFleetMatchTacticalRepair::PursuitSession pursuitSession;
  WhiteFleetMatchTacticalRepair::ObservationGuard observationGuard;
  bool observationHeld=false;
  WhiteFleetC15ReceiverRoute::State c15ReceiverRoute; /**< Event-local, survives CABSL fallbacks. */
  WhiteFleetC15EarlyChannel::State c15EarlyChannel; /**< Explicit opt-in off-ball corridor experiment only. */
  struct SimMaxRun{unsigned event=0,release=0,logs=0,lastLog=0,nearAt=0,candidateAt=0,nearLogs=0,lastNearLog=0,
    lastNearMask=~0u,lastNearMotion=0,prepLogs=0,lastPrepLog=0;
    bool active=false,done=false,near=false;Vector2f body,ball,contract;}c15SimMaxRun;
  unsigned lastDirectedPassDiagnostic=0;
  int directedPassDiagnosticReason=-1;
  bool handlePostKickReacquire();
  void publishRestartPassIntent();
  bool handleRestartReceive();
  WhiteFleetGoalKickExecution::ObservationSearch goalObservationSearch;
  bool handleGoalKickObservation();
  bool handleC15Kickoff();
  bool suppressC15Receive(const Teammate& teammate) const;

  ArmMotionRequest theArmMotionRequest; /**< The arm motion request that is modified by the behavior. */
  BehaviorStateInformation theBehaviorStateInformation; /**< Information about the state transitions of the behavior. */
  BehaviorStatus theBehaviorStatus; /**< The behavior status that is modified by the behavior. */
  CalibrationRequest theCalibrationRequest; /**< The camera calibration request that is modified by the behavior. */
  HeadMotionRequest theHeadMotionRequest; /**< The head motion request that is modified by the behavior. */
  MotionRequest theMotionRequest; /**< The motion request that is modified by the behavior. */

#include "Options/Options.h"
#include "Skills/Arms/Arms.h"
#include "Skills/Ball/Ball.h"
#include "Skills/Calibration/Calibration.h"
#include "Skills/Demo/Demo.h"
#include "Skills/Head/Head.h"
#include "Skills/Output/Output.h"
#include "Skills/Support/Support.h"
#include "Skills/Walk/Walk.h"
};
