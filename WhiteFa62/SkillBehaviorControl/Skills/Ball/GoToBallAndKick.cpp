/**
 * @file GoToBallAndKick.cpp
 *
 * This file implements the GoToBallAndKick skill.
 *
 * @author Arne Hasselbring
 */

#include "WhiteFleetSkillBehaviorControl.h"
#include "Tools/BehaviorControl/WalkSpeedConversion.h"
#include "WhiteFleetK1HotChase.h"

option((WhiteFleetSkillBehaviorControl) GoToBallAndKick,
       args((Angle) targetDirection,
            (KickInfo::KickType) kickType,
            (bool) lookActiveWithBall,
            (KickPrecision) alignPrecisely,
            (float) length,
            (const Pose2f&) speed,
            (ReduceWalkSpeedType::ReduceWalkSpeedType) reduceWalkSpeedType),
       defs((float)(1000.f) switchToPathPlannerDistance, /**< If the target is further away than this distance, the path planner is used. */
            (float)(900.f) switchToLibWalkDistance, /**< If the target is closer than this distance, LibWalk is used. */
            (float)(600.f) goalkeeperDoNotIgnoreObstaclesDistance, /**< If the target is further away than this distance, the goalkeeper will not ignore obstacles. */
            (float)(500.f) goalkeeperIgnoreObstaclesDistance), /**< If the target is closer than this distance, the goalkeeper will ignore obstacles. */
       vars((Angle)(0_deg) startRotation)) /**< The value of \c theOdometryData.rotation when the \c turnAfterKick state is entered. */
{
  const Pose2f kickPose = Pose2f(targetDirection, theFieldBall.positionRelative).rotate(theKickInfo[kickType].rotationOffset).translate(theKickInfo[kickType].ballOffset);

  ASSERT(reduceWalkSpeedType != ReduceWalkSpeedType::distanceBased);
  Pose2f walkingSpeedRatio = WalkSpeedConversion::convertSpeedRatio(reduceWalkSpeedType, speed, Pose2f(1.f, 1.f, 1.f), theFrameInfo, theGameState, theFieldBall, theWalkingEngineOutput, true);
  // Use the caller's existing forward limit. The native path planner, rotation,
  // lateral limits, acceleration envelope and close-ball alignment are retained.
  const bool regularPlay = theGameState.state == GameState::playing && !theGameState.stopped;
  const bool fieldPlayer = !theGameState.isGoalkeeper();
  const int hotKickAge = WhiteFleetK1HotChase::kickAge(theFrameInfo.time, theMotionInfo.lastKickTimestamp,
                                           theMotionRequest.postKickPursuitTimestamp,theMotionRequest.postKickPursuitActive);
  const bool chaseWindow = hotKickAge >= 0 &&
    WhiteFleetK1HotChase::pursue(WhiteFleetK1HotChase::enabled(), fieldPlayer, regularPlay, true,
      hotKickAge, theFieldBall.timeSinceBallWasSeen,
      theFieldBall.positionRelative.norm(), theFieldBall.positionRelative.x(), theFieldBall.positionRelative.angle(),
      theFieldBall.positionRelative.norm() > 1.f ? theFieldBall.velocityRelative.dot(theFieldBall.positionRelative.normalized()) : 0.f,
      theInertialData.angle.head<2>().norm(), theInertialData.gyro.norm(),theMotionRequest.postKickPursuitActive) && std::abs(targetDirection) < 45_deg;
  const bool fastApproach = WhiteFleetK1HotChase::fastApproach(WhiteFleetK1HotChase::enabled(), fieldPlayer, regularPlay,
    theFieldBall.timeSinceBallWasSeen, theFieldBall.positionRelative.norm(), theFieldBall.positionRelative.x(),
    theFieldBall.positionRelative.angle(), targetDirection);
  if(chaseWindow || fastApproach)
    walkingSpeedRatio.translation.x() = Rangef::ZeroOneRange().limit(speed.translation.x());

  initial_state(walkToBallFarRange)
  {
    transition
    {
      if(action_done)
      {
        if(theKickInfo[theMotionInfo.lastKickType].postRotationOffset != 0_deg)
        {
          startRotation = theOdometryData.rotation;
          goto turnAfterKick;
        }
        else
          goto done;
      }
      else if(kickPose.translation.norm() < goalkeeperIgnoreObstaclesDistance && theGameState.isGoalkeeper())
        goto walkToBallIgnoreObstacles;
      else if(kickPose.translation.norm() < switchToLibWalkDistance)
        goto walkToBallCloseRange;
    }

    action
    {
      PublishMotion({.target = kickPose.translation});
      auto obstacleAvoidance = thePathPlanner.plan(theRobotPose * kickPose, speed, /*ignoreBall*/true);
      if(lookActiveWithBall)
        LookActive({.withBall = true,
                    .onlyOwnBall = true});
      else
        GoToBallHeadControl({.distanceToTarget = kickPose.translation.norm(),
                             .lookAtKickTarget = true,
                             .kickTargetRelative = theFieldBall.positionRelative + Vector2f(theKickInfo[kickType].range.max, 0.f).rotated(targetDirection)});
      WalkToBallAndKick({.targetDirection = targetDirection,
                         .kickType = kickType,
                         .alignPrecisely = alignPrecisely,
                         .kickLength = length,
                         .speed = walkingSpeedRatio,
                         .obstacleAvoidance = obstacleAvoidance});
    }
  }

  state(walkToBallCloseRange)
  {
    transition
    {
      if(action_done)
      {
        if(theKickInfo[theMotionInfo.lastKickType].postRotationOffset != 0_deg)
        {
          startRotation = theOdometryData.rotation;
          goto turnAfterKick;
        }
        else
          goto done;
      }
      else if(kickPose.translation.norm() < goalkeeperIgnoreObstaclesDistance && theGameState.isGoalkeeper())
        goto walkToBallIgnoreObstacles;
      else if(kickPose.translation.norm() > switchToPathPlannerDistance)
        goto walkToBallFarRange;
    }

    action
    {
      PublishMotion({.target = kickPose.translation});
      auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(kickPose, /* rough: */ true, /* disableObstacleAvoidance: */ false, /* toBall: */ true);
      if(lookActiveWithBall)
        LookActive({.withBall = true,
                    .onlyOwnBall = true});
      else
        GoToBallHeadControl({.distanceToTarget = kickPose.translation.norm(),
                             .lookAtKickTarget = false,
                             .kickTargetRelative = theFieldBall.positionRelative + Vector2f(theKickInfo[kickType].range.max, 0.f).rotated(targetDirection)});
      WalkToBallAndKick({.targetDirection = targetDirection,
                         .kickType = kickType,
                         .alignPrecisely = alignPrecisely,
                         .kickLength = length,
                         .speed = walkingSpeedRatio,
                         .obstacleAvoidance = obstacleAvoidance});
    }
  }

  state(walkToBallIgnoreObstacles)
  {
    transition
    {
      if(action_done)
      {
        if(theKickInfo[theMotionInfo.lastKickType].postRotationOffset != 0_deg)
        {
          startRotation = theOdometryData.rotation;
          goto turnAfterKick;
        }
        else
          goto done;
      }
      else if(kickPose.translation.norm() > switchToPathPlannerDistance)
        goto walkToBallFarRange;
      else if(kickPose.translation.norm() > goalkeeperDoNotIgnoreObstaclesDistance || !theGameState.isGoalkeeper())
        goto walkToBallCloseRange;
    }

    action
    {
      PublishMotion({.target = kickPose.translation});
      auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(kickPose, /* rough: */ true, /* disableObstacleAvoidance: */ true, /* toBall: */ true);
      if(lookActiveWithBall)
        LookActive({.withBall = true,
                    .onlyOwnBall = true});
      else
        GoToBallHeadControl({.distanceToTarget = kickPose.translation.norm(),
                             .lookAtKickTarget = false,
                             .kickTargetRelative = theFieldBall.positionRelative + Vector2f(theKickInfo[kickType].range.max, 0.f).rotated(targetDirection)});
      WalkToBallAndKick({.targetDirection = targetDirection,
                         .kickType = kickType,
                         .alignPrecisely = alignPrecisely,
                         .kickLength = length,
                         .speed = walkingSpeedRatio,
                         .obstacleAvoidance = obstacleAvoidance,});
    }
  }

  target_state(turnAfterKick)
  {
    transition
    {
      if(action_done || theFieldBall.ballWasSeen(state_time - 300))
      {
        if(kickPose.translation.norm() < goalkeeperIgnoreObstaclesDistance && theGameState.isGoalkeeper())
          goto walkToBallFarRange;
        else if(kickPose.translation.norm() < switchToLibWalkDistance)
          goto walkToBallCloseRange;
        else
          goto walkToBallFarRange;
      }
    }
    action
    {
      PublishMotion({.target = kickPose.translation,
                     .speed = {0.f, 0.f, 0.f}});
      LookAtAngles({.pan = Angle::normalize(theKickInfo[theMotionInfo.lastKickType].postRotationOffset - (theOdometryData.rotation - startRotation)),
                    .tilt = 23_deg});
      TurnAngle({.angle = theKickInfo[theMotionInfo.lastKickType].postRotationOffset});
    }
  }

  target_state(done)
  {
    transition
    {
      if(kickPose.translation.norm() < goalkeeperIgnoreObstaclesDistance && theGameState.isGoalkeeper())
        goto walkToBallFarRange;
      else if(kickPose.translation.norm() < switchToLibWalkDistance)
        goto walkToBallCloseRange;
      else
        goto walkToBallFarRange;
    }

    action
    {
      PublishMotion({.target = kickPose.translation});
      auto obstacleAvoidance = theLibWalk.calcObstacleAvoidance(kickPose, /* rough: */ true, /* disableObstacleAvoidance: */ true, /* toBall: */ true);
      if(lookActiveWithBall)
        LookActive({.withBall = true,
                    .onlyOwnBall = true});
      else
        GoToBallHeadControl({.distanceToTarget = kickPose.translation.norm(),
                             .lookAtKickTarget = true,
                             .kickTargetRelative = theFieldBall.positionRelative + Vector2f(theKickInfo[kickType].range.max, 0.f).rotated(targetDirection)});
      WalkToBallAndKick({.targetDirection = targetDirection,
                         .kickType = kickType,
                         .alignPrecisely = alignPrecisely,
                         .kickLength = length,
                         .speed = walkingSpeedRatio,
                         .obstacleAvoidance = obstacleAvoidance});
    }
  }
}
