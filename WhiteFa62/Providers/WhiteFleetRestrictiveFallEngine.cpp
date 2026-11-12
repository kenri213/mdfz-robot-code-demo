#include "WhiteFleetSimEntry.h"
/*
 * @file WhiteFleetRestrictiveFallEngine.cpp
 *
 * A minimized motion engine for falling.
 *
 * @author Philip Reichenberg
 */

#include "WhiteFleetRestrictiveFallEngine.h"
#include "Framework/Settings.h"
#include "Platform/SystemCall.h"
#include "Representations/Infrastructure/StiffnessData.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Tools/Motion/MotionUtilities.h"
#include <cmath>

MAKE_MODULE(WhiteFleetRestrictiveFallEngine);

void WhiteFleetRestrictiveFallEngine::update(FallGenerator& fallGenerator)
{
  WhiteFleetSimEntry::activate();
  static thread_local bool fleetProviderSeen=false;
  if(!fleetProviderSeen){WhiteFleetSimEntry::witness("WhiteFleetRestrictiveFallEngine");fleetProviderSeen=true;}
  fallGenerator.shouldCatchFall = [this](const MotionRequest& motionRequest)
  {
    const bool normalFallStart = theFallDownState.state == FallDownState::falling &&
                                 motionRequest.motion != MotionRequest::playDead &&
                                 theMotionInfo.executedPhase != MotionPhase::getUp &&
                                 theMotionInfo.executedPhase != MotionPhase::playDead &&
                                 theMotionInfo.executedPhase != MotionPhase::prepare;

    return normalFallStart;
  };

  fallGenerator.createPhase = [this](const MotionRequest&, const MotionPhase&)
  {
    return std::make_unique<WhiteFleetRestrictiveFallPhase>(*this);
  };
}

WhiteFleetRestrictiveFallPhase::WhiteFleetRestrictiveFallPhase(const WhiteFleetRestrictiveFallEngine& engine) :
  MotionPhase(MotionPhase::fall),
  engine(engine),
  startTime(engine.theFrameInfo.time)
{
  targetRequest.angles.fill(0_deg);
  lastRequest.angles = engine.theJointAngles.angles;
  VERIFY(InverseKinematic::calcLegJoints(Pose3f(Vector3f(0.f, engine.theWalkStepData.yHipOffset, -engine.standHeight)),
                                         Pose3f(Vector3f(0.f, -engine.theWalkStepData.yHipOffset, -engine.standHeight)),
                                         Vector2f(0.f, 0.f), targetRequest, engine.theRobotDimensions));

  lastRequest.stiffnessData.stiffnesses.fill(static_cast<int>(engine.stiffnessRange.max));
  targetRequest.stiffnessData.stiffnesses.fill(static_cast<int>(engine.stiffnessRange.max));
  targetRequest.angles[Joints::headPitch] = targetRequest.angles[Joints::headYaw] = 0_deg;
  targetRequest.angles[Joints::lShoulderPitch] = targetRequest.angles[Joints::rShoulderPitch] = 0_deg;
  targetRequest.angles[Joints::lShoulderRoll] = -83_deg;
  targetRequest.angles[Joints::lElbowPitch] = 90_deg;
  targetRequest.angles[Joints::lElbowYaw] = targetRequest.angles[Joints::rElbowYaw] = 0_deg;
  targetRequest.angles[Joints::rShoulderRoll] = 83_deg;
  targetRequest.angles[Joints::rElbowPitch] = 90_deg;
  startTime = engine.theFrameInfo.time;
  startTransitionTime = engine.theFrameInfo.time;
}

void WhiteFleetRestrictiveFallPhase::update()
{
  bool transitionCouldStart = false;

  // Referee STOP pauses play, not recovery from a fall. Keep the native
  // settling, allowGetUp and posture gates; emergency PlayDead still wins
  // in WhiteFleetMotionEngine. Upright motion remains held there until Resume.
  const bool recoveryAllowed = !engine.theGameState.stopped ||
                               (engine.theGameState.gameControllerActive && engine.theGameState.playerState == GameState::active);
  if(!(std::abs(engine.theInertialData.gyro.x()) < engine.maxGyroToStartGetUp &&  // gyros low
       std::abs(engine.theInertialData.gyro.y()) < engine.maxGyroToStartGetUp &&
       std::abs(engine.theInertialData.gyro.z()) < engine.maxGyroToStartGetUp &&
       ((recoveryAllowed && engine.allowGetUp && engine.theFallDownState.state == FallDownState::fallen) || // any pose or upright
        (std::abs(engine.theInertialData.angle.x()) < engine.uprightThreshold &&
         std::abs(engine.theInertialData.angle.y()) < engine.uprightThreshold &&
         (engine.theFallDownState.state == FallDownState::upright || engine.theFallDownState.state == FallDownState::staggering || engine.theFallDownState.state == FallDownState::pickedUp)))))
    startTransitionTime = engine.theFrameInfo.time;
  else
    transitionCouldStart = true;

  if((engine.theFrameInfo.getTimeSince(startTime) > engine.waitAfterFalling) &&
     engine.theFrameInfo.getTimeSince(startTransitionTime) > engine.transitionDelay)
    allowLeavingFallPhase = true;

  if((engine.theFrameInfo.getTimeSince(startTime) > engine.waitAfterFalling) && transitionCouldStart
     && engine.theFrameInfo.getTimeSince(lastSoundTimestamp) > engine.soundTimeDelay)
  {
    lastSoundTimestamp = engine.theFrameInfo.time;
    if(engine.theFallDownState.state == FallDownState::upright)
      SystemCall::say("Starting Stand");
    else
      SystemCall::say("Starting Get Up");
  }
}

bool WhiteFleetRestrictiveFallPhase::isDone(const MotionRequest& motionRequest) const
{
  // If the robot becomes upright / squatting on its own (or with human help) or a getUp is requested after the robot has reached the ground, the phase is done.
  return allowLeavingFallPhase && motionRequest.motion != MotionRequest::fallRequest;
}

void WhiteFleetRestrictiveFallPhase::calcJoints(const MotionRequest&, JointRequest& jointRequest, Pose2f& odometryOffset, MotionInfo& motionInfo)
{
  const Angle jointSpeed = engine.jointSpeed * Global::getSettings().motionCycleTime;
  const Rangea speedLimit(-jointSpeed, jointSpeed);
  const float stiffnessTorsoRatio = mapToRange(std::max(std::abs(engine.theInertialData.angle.x()), std::abs(engine.theInertialData.angle.y())), static_cast<float>(engine.stiffnessTorsoRange.min), static_cast<float>(engine.stiffnessTorsoRange.max), 1.f, 0.f);
  const int maxStiffness = static_cast<int>(mapToRange(stiffnessTorsoRatio, 0.f, 1.f, engine.stiffnessRange.min, engine.stiffnessRange.max));
  targetRequest.angles[Joints::headPitch] = mapToRange(stiffnessTorsoRatio, 0.f, 1.f, (engine.theInertialData.angle.y() > 0.f ? -1.f : 1.f) * static_cast<float>(engine.headFallAngle), 0.f);
  const bool useDamping = engine.theFrameInfo.getTimeSince(startTime) < engine.unstiffTime;
  FOREACH_ENUM(Joints::Joint, joint)
  {
    if(useDamping && joint >= Joints::firstArmJoint)
    {
      lastRequest.angles[joint] = engine.theJointAngles.angles[joint];
      lastRequest.stiffnessData.stiffnesses[joint] = 0;
    }
    else
    {
      const Rangea positionLimit(-engine.maxPositionDifference + engine.theJointAngles.angles[joint], engine.maxPositionDifference + engine.theJointAngles.angles[joint]);
      const Angle maxTargetPosition = positionLimit.limit(targetRequest.angles[joint]);
      lastRequest.angles[joint] += speedLimit.limit(maxTargetPosition - lastRequest.angles[joint]);
    }
  }

  ASSERT(lastRequest.isValid());

  FOREACH_ENUM(Joints::Joint, joint)
  {
    if(useDamping && joint >= Joints::firstArmJoint)
      break;
    lastRequest.stiffnessData.stiffnesses[joint] = std::min(maxStiffness, lastRequest.stiffnessData.stiffnesses[joint] + 1);
  }

  jointRequest = lastRequest;
  odometryOffset = Pose2f();
  motionInfo.isMotionStable = false;
}

std::unique_ptr<MotionPhase> WhiteFleetRestrictiveFallPhase::createNextPhase(const MotionPhase& defaultPhase) const
{
  if((std::abs(engine.theInertialData.angle.x()) < engine.uprightThreshold &&
      std::abs(engine.theInertialData.angle.y()) < engine.uprightThreshold) || defaultPhase.type == MotionPhase::playDead)
    return std::unique_ptr<MotionPhase>();
  return engine.theGetUpGenerator.createPhase(*this);
}
