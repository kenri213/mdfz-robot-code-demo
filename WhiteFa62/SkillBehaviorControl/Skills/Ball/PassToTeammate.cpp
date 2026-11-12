/**
 * @file PassToTeammate.cpp
 *
 * This file defines an implementation of the PassToTeammate skill, that chooses a suitable target position, kick type and time of execution for passing the ball to a specific player while avoiding obstacles.
 *
 * @author Arne Hasselbring
 * @author Jo Lienhoop
 */

#include "WhiteFleetSkillBehaviorControl.h"
#include "Debugging/DebugDrawings.h"
#include "Debugging/DebugDrawings3D.h"
#include "Tools/BehaviorControl/KickSelection.h"
#include "Tools/BehaviorControl/SectorWheel.h"
#include "Tools/Modeling/BallPhysics.h"
#include "WhiteFleetMatchTacticalRepair.h"
#include "WhiteFleetFleetProgressPolicy.h"
#include "WhiteFleetGoalKickExecution.h"
#include "Tools/BehaviorControl/TeamActivityPolicy.h"
#include "Tools/Motion/KickLengthConverter.h"

option((WhiteFleetSkillBehaviorControl) PassToTeammate,
       args((int) playerNumber,
            (int) passedWaitingTime),
       load((std::vector<KickInfo::KickType>) allowedKicks, /**< The list of available kicks for passing */
            (bool) doSpecialKickChecks, /**< Makes kicks unavailable for certain situations (should be disabled when only a few kicks are allowed) */
            (float) ratingThreshold, /**< During kick-off and free kicks wait with kicking the ball if the teammate's pass position rating is worse */
            (Angle) maxAngleOffset, /**< Kick angle must not deviate more than this from the original angle to the target. This is half the angular range in which free sectors are considered */
            (Angle) maxAngleOffsetBehind, /**< maxAngleOffset when the ball is passed on the own goal side (instead of the opponent's goal side) of the pass target (from the perspective of the ball) */
            (Vector2f) defaultWaitingPositionOffset, /**< Distance of waiting position to the kick position offset to the ball if no kick is selected yet. */
            (float) targetOffsetToFieldBorder, /**< Pass target's minimum distance to field border, to prevent ball from rolling out of its boundary */
            (int) timeLeftToAdjust, /**< Time left until the game state ends below which the robot will choose the final pass target and adjust itself accordingly behind the ball before executing the pass */
            (int) minTimeWaiting, /**< Stand still for a minimum of this many seconds */
            (int) maxTimeWaiting, /**< Stand still for a maximum of this many seconds */
            (float) maxTimeWaitingFactor, /**< Maximum waiting time when multiplied with the time left in this half of the game */
            (float) averageKickExecutionTime, /**< Time it takes to perform most kick motions (from the KickInfo) */
            (Vector2f) averageKickBallOffset, /**< Position behind the ball needed to perform most kick motions (from the KickInfo) */
            (float) ignoreObstaclesThreshold, /**< Obstacle avoidance will be disabled when this close to the waiting position */
            (float) ignoreDynamicObstaclesThreshold, /**< Obstacle avoidance will be disabled when this close to the waiting position */
            (float) distanceForNormalWalk, /**< Reduce walking speed down to normal one once we are close to the target and the ball. */
            (bool) lookAhead, /**< Estimate the teammate's position into the future */
            (float) maxLookAheadTime, /**< When lookAhead, estimate the teammate's position a maximum of this many milliseconds into the future */
            (float) minRating,
            (Angle) maxHeadAngle, /**< maximum angle when looking around when waiting before executing a free kick */
            (Angle) headSpeed, /**< angle speed when looking around when waiting before executing a free kick*/
            (Angle) headTilt, /**< tilt of the head when looking around when waiting before executing a free kick */
            (float) ballDistanceForSlowWalk, /**< If the ball is close, walk slow. */
            (float) forcedMinimalPassKickLength), /**< For a direct pass, use this minimal kick length. */
       vars((const GlobalTeammatesModel::TeammateEstimate*)(nullptr) teammate,
            (bool)(false) isTargetFree,
            (Angle)(0_deg) kickAngle,
            (float)(0.f) kickLength,
            (KickInfo::KickType)(KickInfo::numOfKickTypes) lastKickType,
            (TargetEvaluation) lastTargetEvaluation))
{
  const Vector2f opponentGoal = Vector2f(theFieldDimensions.xPosOpponentGoalLine, 0.f);
  const Vector2i bottomLeftCorner = Vector2i(theFieldDimensions.xPosOwnGoalLine + targetOffsetToFieldBorder,
                                             theFieldDimensions.yPosRightTouchline + targetOffsetToFieldBorder);
  const Vector2i topRightCorner = Vector2i(theFieldDimensions.xPosOpponentGoalLine - targetOffsetToFieldBorder,
                                           theFieldDimensions.yPosLeftTouchline - targetOffsetToFieldBorder);

  const auto isKickOff = [&]() -> bool {return theGameState.isKickOff() && theGameState.isForOwnTeam();};
  const auto isFreeKick = [&]() -> bool {return theGameState.isFreeKick() && theGameState.isForOwnTeam();};

  const int minWaitingTime = minTimeWaiting - passedWaitingTime;
  const int maxWaitingTime = maxTimeWaiting - passedWaitingTime;
  const bool claimedGoalPass=theSkillRequest.restartPassRequired&&theSkillRequest.restartPassGoalKick;
  const bool directedPass=WhiteFleetMatchTacticalRepair::enabled(theBehaviorParameters.matchTacticalRepair) &&
    theSkillRequest.skill==SkillRequest::pass && theSkillRequest.passTarget==playerNumber &&
    (theSkillRequest.target.translation!=Vector2f::Zero()||claimedGoalPass);

  const auto reset = [&]
  {
    isTargetFree = false;
    kickAngle = 0_deg;
    kickLength = 0.f;
    lastTargetEvaluation.isValid = false;
  };

  // Check whether or not there is still enough time left to wait this many milliseconds during any restart in play (free kick, corner kick, goal kick, kick-in, kick-off).
  // TODO: Rewrite this to calculate a continuous time to wait instead of a hard decision to wait ("yes" or "no") the predefined time.
  const auto canWait = [&](const int timeToWait) -> bool
  {
    // Never wait in regular playing game state.
    if(!isFreeKick() && !isKickOff())
      return false;
    const int timeUntilSetplayEnds = -theFrameInfo.getTimeSince(theGameState.timeWhenStateEnds);
    const int timeUntilHalfEnds = -theFrameInfo.getTimeSince(theGameState.timeWhenPhaseEnds);
    // Gradually decrease the maximal waiting time based on the progress of the current setplay and overall game.
    return timeUntilSetplayEnds > timeToWait &&
    timeUntilHalfEnds * maxTimeWaitingFactor > timeToWait;
  };

  // Find the player to which the ball should be passed. Note that this skill can be called with an invalid pass target (e.g. p.playerNumber == 0) to utilize the waiting states during setplays. Therefore, no check whether or not the teammate was be found is required after this loop.
  const auto setTeammate = [&]
  {
    teammate = nullptr;
    for(const auto& t : theGlobalTeammatesModel.teammates)
    {
      if(t.playerNumber == playerNumber)
      {
        teammate = &t;
        lastTargetEvaluation.targetOnField = teammate->pose.translation;
        lastTargetEvaluation.isValid = true;
        break;
      }
    }
    if(!teammate)
    {
      lastTargetEvaluation.targetOnField = opponentGoal;
      lastTargetEvaluation.isValid = false;
    }
  };

  // Modify the kick target when the receiver changed from last frame or there was no last kick target.
  const auto keepLastKickTarget = [&]() -> bool
  {
    return teammate &&
    lastTargetEvaluation.teammate == teammate->playerNumber &&
    lastTargetEvaluation.bestShiftedTargetOnField != Vector2f::Zero() &&
    lastTargetEvaluation.isValid;
  };

  // Calculate the target pose in robot-relative coordinates for waiting behind the ball to minimize adjustment steps when executing the kick later on.
  const auto calculateWaitingPose = [&]() -> Pose2f
  {
    const Angle angleBallToGoal = (Vector2f(theFieldDimensions.xPosOpponentGoalLine, 0.f) - theFieldBall.recentBallPositionOnField()).angle();
    // Use parameters from the kick info to find the preferred waiting position behind the ball.
    const Vector2f kickBallOffset = defaultWaitingPositionOffset.rotated(angleBallToGoal);
    return Pose2f(angleBallToGoal - theRobotPose.rotation, theRobotPose.inverse() * (theFieldBall.recentBallPositionOnField() + kickBallOffset));
  };

  // Calculates the rating thresholds of the target position for the decision to wait behind the ball or kick it.
  const auto updateTargetRating = [&]
  {
    float interpolatedRatingThreshold = ratingThreshold;
    const int timeInState = theFrameInfo.getTimeSince(theGameState.timeWhenStateStarted);
    const int timeInStateLeft = theFrameInfo.getTimeSince(theGameState.timeWhenStateEnds);
    // Linear interpolation of state progress to required pass rating from maximum value to 0.f.
    if(isFreeKick() || isKickOff())
    {
      const int totalStateTime = std::max(timeInState - timeInStateLeft, 1);
      const float stateProgress = static_cast<float>(timeInState) / static_cast<float>(totalStateTime);
      interpolatedRatingThreshold *= (1.f - clip(stateProgress, 0.f, 1.f));
    }
    isTargetFree = thePassEvaluation.getRating(theFieldBall.recentBallPositionOnField(), lastTargetEvaluation.bestShiftedTargetOnField, false) > std::max(interpolatedRatingThreshold, minRating);
  };

  const auto updateKickParameters = [&]
  {
    if(!lastTargetEvaluation.isValid)
      return;
    kickAngle = Angle::normalize((lastTargetEvaluation.bestShiftedTargetOnField - theFieldBall.positionOnField).angle() - theRobotPose.rotation);
    kickLength = (lastTargetEvaluation.bestShiftedTargetOnField - theFieldBall.positionOnField).norm();
  };

  const auto updateKickTarget = [&]
  {
    const Vector2f& ballPositionOnField = theFieldBall.positionOnField;

    if(lookAhead && teammate)
    {
      // Approximate the time the pass will take by estimating the time to reach kick position, kick execution time and ball rolling time.
      const Angle estimatedKickAngle = Angle::normalize((lastTargetEvaluation.targetOnField - ballPositionOnField).angle() - theRobotPose.rotation);
      const Pose2f estimatedKickPose = Pose2f(estimatedKickAngle, ballPositionOnField).translate(averageKickBallOffset);
      const Vector2f estimatedBallVelocity(BallPhysics::velocityForDistance((lastTargetEvaluation.targetOnField - ballPositionOnField).norm(), theBallSpecification), 0.f);
      const float estimatedBallRollingTime = BallPhysics::computeTimeUntilBallStops(estimatedBallVelocity, theBallSpecification) * 1000.f;
      const float timeToReachKickPosition = KickSelection::calcTTRP(estimatedKickPose, theWalkingEngineOutput.maxSpeed);
      const unsigned int estimatedPassExecutionTime = static_cast<unsigned int>(clip(timeToReachKickPosition + averageKickExecutionTime + estimatedBallRollingTime, 0.f, maxLookAheadTime));
      lastTargetEvaluation.targetOnField = teammate->getFuturePosition(estimatedPassExecutionTime);
    }

    if(teammate)
      lastTargetEvaluation = thePassTargetEvaluation.getBestPassData(ballPositionOnField, lastTargetEvaluation.targetOnField, teammate->playerNumber);
    else
    {
      lastTargetEvaluation.bestShiftedTargetOnField = opponentGoal;
      lastTargetEvaluation.isValid = false;
    }

    Geometry::clipPointInsideRectangle(bottomLeftCorner, topRightCorner, lastTargetEvaluation.bestShiftedTargetOnField);
    updateTargetRating();
    updateKickParameters();
  };

  // Filters the allowed kicks based on the current situation to improve the overall accuracy of passing.
  const auto calcAvailableKicks = [&]() -> std::vector<KickInfo::KickType>
  {
    std::vector<KickInfo::KickType> kicks;
    kicks.reserve(KickInfo::numOfKickTypes);
    // Check for each allowed kick whether or not it should be made available in the given situation.
    for(const KickInfo::KickType kickType : allowedKicks)
    {
      // Makes some kicks unavailable or even mandatory for certain situations.
      if(doSpecialKickChecks)
      {
        switch(kickType)
        {
          case KickInfo::rlKickDynamicOld:
          {
            // This kick requires standing still and should only be used and preferred during any restart in play (free kick, corner kick, goal kick, kick-in) except for kick-offs.
            if(!isFreeKick() && !isKickOff())
              continue;
            // Force this kick
            return { KickInfo::rlKickDynamicOld };
          }
          default:
            break;
        }
      }
      kicks.push_back(kickType);
    }
    return kicks;
  };

  // Selects the kick type that would move the ball most quickly and accurately to the pass target based on the kick's range and the time to reach its required pose behind the ball.
  const auto selectKickType = [&]() -> KickInfo::KickType
  {
    KickInfo::KickType bestKick = KickInfo::numOfKickTypes;
    float bestRangeDeficiency = std::numeric_limits<float>::max();
    float bestTTRP;
    const auto kicks = calcAvailableKicks();
    for(const KickInfo::KickType kickType : kicks)
    {
      const KickInfo::Kick& kick = theKickInfo[kickType];
      const float rangeDeficiency = kick.range.isInside(kickLength) ? 0.f :
      std::max(0.f, (kickLength < kick.range.min ?
                     (kick.range.min - kickLength) :
                     (kickLength - kick.range.max)) - (kickType == lastKickType ? 200.f : 0.f));
      if(rangeDeficiency < bestRangeDeficiency)
      {
        bestRangeDeficiency = rangeDeficiency;
        bestTTRP = std::numeric_limits<float>::max();
      }
      else if(rangeDeficiency > bestRangeDeficiency)
        continue;
      const Pose2f kickPose = Pose2f(kickAngle, theFieldBall.positionRelative).rotate(kick.rotationOffset).translate(kick.ballOffset);
      const float ttrp = KickSelection::calcTTRP(kickPose, theWalkingEngineOutput.maxSpeed) - (kickType == lastKickType ? 500.f : 0.f);
      if(ttrp < bestTTRP)
      {
        bestKick = kickType;
        bestTTRP = ttrp;
      }
    }
    return bestKick;
  };

  const auto scaleKickStrength = [&](float& kickLength)
  {
    const float upperKickLength = std::max(forcedMinimalPassKickLength, 3.f * kickLength);
    const Angle interceptError = Angle::normalize((teammate->pose.translation - theFieldBall.positionRelative).angle() - (lastTargetEvaluation.bestShiftedTargetOnField - theFieldBall.positionRelative).angle());
    return mapToRange(std::abs(interceptError / 15_deg), 0.f, 1.f, upperKickLength, kickLength);
  };

  const auto draw = [&]
  {
    COMPLEX_DRAWING("option:PassToTeammate:target")
    {
      const Vector2f& ballPositionOnField = theFieldBall.positionOnField;
      CROSS("option:PassToTeammate:target", lastTargetEvaluation.targetOnField.x(), lastTargetEvaluation.targetOnField.y(), 100, 20, Drawings::solidPen, isTargetFree ? ColorRGBA::blue : ColorRGBA::red);
      ARROW("option:PassToTeammate:target", ballPositionOnField.x(), ballPositionOnField.y(), lastTargetEvaluation.bestShiftedTargetOnField.x(), lastTargetEvaluation.bestShiftedTargetOnField.y(), 20, Drawings::PenStyle::solidPen, lastTargetEvaluation.isAngleFree ? ColorRGBA::blue : ColorRGBA::red);
    }
    COMPLEX_DRAWING("option:PassToTeammate:precision")
    {
      const Vector2f& ballPositionOnField = theFieldBall.positionOnField;
      const Vector2f ballToTarget = lastTargetEvaluation.bestShiftedTargetOnField - ballPositionOnField;
      const Vector2f ballToTeammate = lastTargetEvaluation.targetOnField - ballPositionOnField;
      const Vector2f precisionLeft = ballPositionOnField + ballToTarget.rotated(lastTargetEvaluation.precisionRange.min);
      const Vector2f precisionRight = ballPositionOnField + ballToTarget.rotated(lastTargetEvaluation.precisionRange.max);
      LINE("option:PassToTeammate:target", ballPositionOnField.x(), ballPositionOnField.y(), precisionLeft.x(), precisionLeft.y(), 20, Drawings::PenStyle::dashedPen, lastTargetEvaluation.isAngleFree ? ColorRGBA::blue : ColorRGBA::red);
      LINE("option:PassToTeammate:target", ballPositionOnField.x(), ballPositionOnField.y(), precisionRight.x(), precisionRight.y(), 20, Drawings::PenStyle::dashedPen, lastTargetEvaluation.isAngleFree ? ColorRGBA::blue : ColorRGBA::red);
      const Vector2f maxAngleLeft = ballPositionOnField + ballToTeammate.rotated(maxAngleOffset);
      const Vector2f maxAngleRight = ballPositionOnField + ballToTeammate.rotated(-maxAngleOffset);
      LINE("option:PassToTeammate:target", ballPositionOnField.x(), ballPositionOnField.y(), maxAngleLeft.x(), maxAngleLeft.y(), 20, Drawings::PenStyle::dashedPen, ColorRGBA::red);
      LINE("option:PassToTeammate:target", ballPositionOnField.x(), ballPositionOnField.y(), maxAngleRight.x(), maxAngleRight.y(), 20, Drawings::PenStyle::dashedPen, ColorRGBA::red);
      const Vector2f maxAngleLeftBehind = ballPositionOnField + ballToTeammate.rotated(maxAngleOffsetBehind);
      const Vector2f maxAngleRightBehind = ballPositionOnField + ballToTeammate.rotated(-maxAngleOffsetBehind);
      LINE("option:PassToTeammate:target", ballPositionOnField.x(), ballPositionOnField.y(), maxAngleLeftBehind.x(), maxAngleLeftBehind.y(), 20, Drawings::PenStyle::dashedPen, ColorRGBA::orange);
      LINE("option:PassToTeammate:target", ballPositionOnField.x(), ballPositionOnField.y(), maxAngleRightBehind.x(), maxAngleRightBehind.y(), 20, Drawings::PenStyle::dashedPen, ColorRGBA::orange);
    }
    COMPLEX_DRAWING3D("option:PassToTeammate:trajectory")
    {
      const float radius = theBallSpecification.radius;
      CYLINDER_ARROW3D("option:PassToTeammate:trajectory",
                       (Vector3f() << theFieldBall.positionOnField, radius).finished(),
                       (Vector3f() << lastTargetEvaluation.bestShiftedTargetOnField, radius).finished(),
                       radius, radius, radius,
                       lastTargetEvaluation.isAngleFree ? ColorRGBA::fromTeamColor(theGameState.ownTeam.color(lastTargetEvaluation.teammate)) : ColorRGBA::red);
    }
  };

  setTeammate();
  if(!keepLastKickTarget())
    reset();

  common_transition
  {
    if(directedPass)goto directed;
    // In regular playing, always kick the ball immediately (without waiting).
    if(!isFreeKick() && !isKickOff())
    {
      if(teammate)
        goto kick;
      else
        goto abort;
    }
    // When a free kick started this frame, reset member variables and take the transitions from the initial state.
    if(theGameState.state != theExtendedGameState.stateLastFrame)
    {
      reset();
      if(canWait(minWaitingTime + timeLeftToAdjust))
        goto walk;
      else
        goto kick;
    }
  }

  // When the skill started this frame, reset member variables and decide whether to wait or adjust before executing the kick.
  initial_state(initial)
  {
    transition
    {
      reset();
      if(passedWaitingTime > 0)
        goto wait;
      if(canWait(minWaitingTime + timeLeftToAdjust))
        goto walk;
      else
        goto kick;
    }
  }

  // Walk to the waiting position while calculating the kick target once.
  state(walk)
  {
    transition
    {
      // When the waiting position is reached, wait behind the ball, if there's time to stand, otherwise adjust before executing the kick.
      if(action_done && canWait(minWaitingTime + timeLeftToAdjust))
        goto wait;
      if(!canWait(timeLeftToAdjust))
        goto kick;
    }

    action
    {
      LookActive({.withBall = true});

      // Calculate the kick target and kick type in the first frame of this state and when the passed-to player changed.
      updateKickTarget();
      lastKickType = selectKickType();

      const Pose2f targetPose = calculateWaitingPose();
      WalkToPoint({.target = targetPose,
                   .reduceWalkSpeedType = targetPose.translation.squaredNorm() > sqr(distanceForNormalWalk) ? ReduceWalkSpeedType::noChange : ReduceWalkSpeedType::slow,
                   .rough = targetPose.translation.squaredNorm() <= sqr(ignoreObstaclesThreshold),
                   .disableObstacleAvoidance = targetPose.translation.squaredNorm() <= sqr(ignoreDynamicObstaclesThreshold),
                   .disableAvoidFieldBorder = true,
                   .targetOfInterest = theFieldBall.recentBallPositionRelative()});
    }
  }

  // Stand at the waiting position behind the ball as long as there's is still enough time left and the pass situation could improve.
  state(wait)
  {
    transition
    {
      updateKickTarget();
      const bool waitedTooLong = !canWait(timeLeftToAdjust) ||
                                 state_time > maxWaitingTime;
      const bool waitedLongEnough = state_time > minWaitingTime;
      const bool passAvailabilityChanged = !keepLastKickTarget() ||
                                           (lastTargetEvaluation.isAngleFree && isTargetFree);
      if(waitedTooLong || (passAvailabilityChanged && waitedLongEnough))
      {
        if(!lastTargetEvaluation.isValid || lastTargetEvaluation.combinedRating < 0.1f)
          goto abort;
        else
          goto kick;
      }
    }

    action
    {
      Stand();
      LookLeftAndRight({.startLeft = true,
                        .maxPan = maxHeadAngle,
                        .tilt = headTilt,
                        .speed = headSpeed});
      if(lastTargetEvaluation.isValid && (lastTargetEvaluation.isAngleFree || isFreeKick()))
      {
        PassTarget({.passTarget = playerNumber,
                    .ballTarget = theRobotPose.inverse() * lastTargetEvaluation.bestShiftedTargetOnField});
      }
      lastKickType = KickInfo::numOfKickTypes;
    }
  }

  // Execute the kick immediately.
  state(kick)
  {
    transition
    {
      if(!teammate)
        goto abort;
    }

    action
    {
      // Keep the last kick target during a set play.
      if(state_time == 0 || (!isFreeKick() && !isKickOff()))
      {
        updateKickTarget();
        kickAngle = theRobotPose.rotation + kickAngle;
        kickLength = scaleKickStrength(kickLength);
      }

      // Set the most suitable kick and its parameters, then execute it.
      auto kickType = selectKickType();

      // TODO: Opponents running into the planned trajectory can regularly (unintentionally) block and intercept the ball, because the kick angle is not adjusted in the two'ish seconds it takes the robot to go to the ball and kick.
      if(lastTargetEvaluation.isValid && (lastTargetEvaluation.isAngleFree || isFreeKick()))
        GoToBallAndKick({.targetDirection = Angle::normalize(kickAngle - theRobotPose.rotation),
                         .kickType = kickType,
                         .length = kickLength,
                         .reduceWalkSpeedType = theGameState.isFreeKick() && theFieldBall.positionRelative.squaredNorm() < sqr(ballDistanceForSlowWalk) ? ReduceWalkSpeedType::slow : ReduceWalkSpeedType::noChange});
      else
        DribbleToGoal();
      if(lastTargetEvaluation.isValid && (lastTargetEvaluation.isAngleFree || isFreeKick()))
      {
        PassTarget({.passTarget = playerNumber,
                    .ballTarget = theRobotPose.inverse() * lastTargetEvaluation.bestShiftedTargetOnField});
      }
      lastKickType = kickType;
    }
  }

  state(directed)
  {
    transition
    {
      if(!directedPass)
      {
        reset();
        if(passedWaitingTime>0)goto wait;
        if(canWait(minWaitingTime+timeLeftToAdjust))goto walk;
        if(teammate)goto kick;
        goto abort;
      }
    }
    action
    {
      int reason=0; // 0 native safety blocked; 1 receiver/flight wait; 2 admitted.
      theBehaviorStatus.restartPassAdmission=1;
      [[maybe_unused]] float receiverEtaForLog=-1.f,arrivalForLog=-1.f;
      [[maybe_unused]] float requestedVelocityForLog=-1.f,packetAgeForLog=-1.f;
      [[maybe_unused]] int kickTypeForLog=-1,halfClosedForLog=0;
      const Vector2f target=theSkillRequest.target.translation;
      const Vector2f ball=theFieldBall.positionOnField;
      const auto peer=std::find_if(theTeamData.teammates.begin(),theTeamData.teammates.end(),
        [&](const Teammate& t){return t.number==playerNumber;});
      const bool realPeer=teammate && peer!=theTeamData.teammates.end() && peer->theRobotStatus.isUpright &&
        peer->theRobotPose.isFinite() && peer->theRobotPose.quality!=RobotPose::poor &&
        TeamActivityPolicy::fresh(theFrameInfo.time,peer->theFrameInfo.time,1200);
      const bool recovering=theSkillRequest.restartPassRequired&&theSkillRequest.restartPassPasser==theGameState.playerNumber&&
        theSkillRequest.restartPassEpoch&&theFrameInfo.time>=theSkillRequest.restartPassEpoch&&
        theFrameInfo.time-theSkillRequest.restartPassEpoch<
          (theSkillRequest.restartPassGoalKick?WhiteFleetMatchTacticalRepair::goalKickContractBudget:WhiteFleetMatchTacticalRepair::kickoffContractBudget)&&
        theGameState.state==GameState::playing;
      const bool backfield=theSkillRequest.backfieldStrongOutlet&&theGameState.state==GameState::playing&&ball.x()<0.f&&
        theBallModel.timeWhenLastSeen>0&&theBallModel.timeWhenLastSeen<=theFrameInfo.time&&
        theFrameInfo.time-theBallModel.timeWhenLastSeen<=200;
      bool safe=realPeer && theFieldBall.ballWasSeen(200) && ball.allFinite() && target.allFinite() &&
        !theGameState.stopped && theGameControllerData.stopped==0 &&
        (theGameState.isForOwnTeam()||recovering||backfield) && target.x()>ball.x()+500.f &&
        target.x()<theFieldDimensions.xPosOpponentGoalLine-600.f &&
        std::abs(target.y())<theFieldDimensions.yPosLeftTouchline-500.f;
      if(claimedGoalPass)
      {
        const bool goalContext=(theGameState.isGoalKick()&&theGameState.isForOwnTeam()&&
          theSkillRequest.restartPassEpoch==theGameState.timeWhenStateStarted)||recovering;
        safe=safe&&goalContext&&theGameState.playerState==GameState::active&&
          TeamActivityPolicy::fresh(theFrameInfo.time,theBallModel.timeWhenLastSeen,200)&&
          theBallModel.timeWhenLastSeen>=theSkillRequest.restartPassEpoch&&
          WhiteFleetGoalKickExecution::contract(theSkillRequest.restartPassTargetValid,theSkillRequest.restartPassPreparing,
          theGameState.isGoalkeeper(),theSkillRequest.restartPassPasser,theGameState.playerNumber,
          theSkillRequest.restartPassReceiver,playerNumber,theFrameInfo.time,theSkillRequest.restartPassEpoch,
          {target.x(),target.y()},{theSkillRequest.restartPassTarget.x(),theSkillRequest.restartPassTarget.y()});
      }
      if(theSkillRequest.backfieldStrongOutlet)
        safe=safe&&backfield&&WhiteFleetFleetProgressPolicy::frontSupply({ball.x(),ball.y()},{target.x(),target.y()},
          theFieldDimensions.xPosOpponentGoalLine,theFieldDimensions.yPosLeftTouchline);
      if(safe)
      {
        const float risk=thePassEvaluation.getRating(ball,target,false);
        theBehaviorStatus.restartPassSafety=std::isfinite(risk)?risk:-1.f;
        safe=std::isfinite(risk)&&risk>=.7f;
      }
      if(safe)
      {
        const auto evaluation=thePassTargetEvaluation.getBestPassData(ball,target,playerNumber);
        safe=evaluation.isValid && evaluation.isAngleFree && std::isfinite(evaluation.passRating)&&evaluation.passRating>=.7f &&
          (evaluation.bestShiftedTargetOnField-target).norm()<=300.f &&
          !(theSkillRequest.restartPassRequired&&theSkillRequest.restartPassPreparing);
      }
      if(safe)
      {
        const float length=(target-ball).norm();
        auto type=backfield||(theGameState.isGoalkeeper() && (theGameState.isGoalKick()||theSkillRequest.restartPassGoalKick))
          ? KickInfo::rlKickStrong:KickInfo::rlKickDynamic;
        float requestedVelocity=KickLengthConverter::requestedBallVelocity(type,length,theKickInfo,
          theBallSpecification,theWalkingEngineOutput.kickVelocityRange);
        float flight=KickLengthConverter::requestedFlightTime(requestedVelocity,length,theBallSpecification);
        if(type==KickInfo::rlKickDynamic && (!std::isfinite(flight) || flight>30.f))
        {
          type=KickInfo::rlKickStrong;
          requestedVelocity=KickLengthConverter::requestedBallVelocity(type,length,theKickInfo,
            theBallSpecification,theWalkingEngineOutput.kickVelocityRange);
          flight=KickLengthConverter::requestedFlightTime(requestedVelocity,length,theBallSpecification);
        }
        const float arrival=flight>=0.f && flight<30.f ? flight*1000.f:std::numeric_limits<float>::infinity();
        const float packetAge=static_cast<float>(theFrameInfo.getTimeSince(peer->theFrameInfo.time));
        const bool halfClosed=theGameState.isKickOff() && theGameState.isForOwnTeam() &&
          (theIllegalAreas.illegal & (1u<<IllegalAreas::opponentHalf));
        // Plan AFTER native release; never demand an illegal pre-touch run.
        // Reserve the existing peer freshness budget for release delivery.
        const float receiverEta=KickSelection::calcTTRP(peer->theRobotPose.inverse()*Pose2f(0.f,target),theWalkingEngineOutput.maxSpeed)
          +packetAge+(halfClosed?1200.f:0.f);
        requestedVelocityForLog=requestedVelocity;packetAgeForLog=packetAge;
        kickTypeForLog=static_cast<int>(type);halfClosedForLog=halfClosed;
        receiverEtaForLog=receiverEta;arrivalForLog=arrival;reason=1;
        theBehaviorStatus.restartPassAdmission=2;
        theBehaviorStatus.restartPassFlightMs=std::isfinite(arrival)?arrival:-1.f;
        theBehaviorStatus.restartPassEtaMs=std::isfinite(receiverEta)?receiverEta:-1.f;
        // Publish the REAL latched point before strike admission, using the
        // unchanged native budget. Preparation is legal before native release.
        PassTarget({.passTarget=playerNumber,.ballTarget=theRobotPose.inverse()*target});
        const Angle fieldAngle=(target-ball).angle();
        if(std::isfinite(arrival) && std::isfinite(receiverEta) && receiverEta<=arrival && length>=500.f && length<=12000.f)
        {
          reason=2;
          theBehaviorStatus.restartPassAdmission=3;
          GoToBallAndKick({.targetDirection=Angle::normalize(fieldAngle-theRobotPose.rotation),
            .kickType=type,.length=length,.reduceWalkSpeedType=ReduceWalkSpeedType::noChange});
        }
        else
        {
          const Vector2f waiting=ball+theKickInfo[type].ballOffset.rotated(fieldAngle);
          WalkToPoint({.target=theRobotPose.inverse()*Pose2f(fieldAngle,waiting),
            .targetOfInterest=theFieldBall.positionRelative});
          LookActive({.withBall=true});
        }
      }
      else {LookActive({.withBall=true});Stand();} // Never dribble/short-pass as a hidden fallback.
      if(reason!=directedPassDiagnosticReason && theFrameInfo.getTimeSince(lastDirectedPassDiagnostic)>=500)
      {
        lastDirectedPassDiagnostic=theFrameInfo.time;directedPassDiagnosticReason=reason;
        OUTPUT_TEXT("MATCH_DIRECTED_PASS reason=" << reason << " receiver=" << playerNumber <<
          " eta_ms=" << receiverEtaForLog << " flight_ms=" << arrivalForLog <<
          " target_x=" << target.x() << " target_y=" << target.y());
        ANNOTATION("DirectedPassAdmission", "reason=" << reason << " receiver=" << playerNumber <<
          " eta_ms=" << receiverEtaForLog << " flight_ms=" << arrivalForLog <<
          " requested_mm_s=" << requestedVelocityForLog << " kick_type=" << kickTypeForLog <<
          " peer_age_ms=" << packetAgeForLog << " half_closed=" << halfClosedForLog <<
          " target=" << target.x() << "," << target.y());
      }
    }
  }

  aborted_state(abort)
  {
    action
    {
      // TODO: Sometimes it might be better than dribbling to have an alternative pass target available. Consider passing a list of the best pass targets from the strategy layer as parameters to this skill.
      theClearTarget.update();
      if(theGameState.isFreeKick() && theGameState.isForOwnTeam() && theClearTarget.isValid)
      {
        ClearBall();
      }
      else
      {
        DribbleToGoal();
      }
      reset();
    }
  }

  thePassTargetEvaluation.setLastPassTarget(lastTargetEvaluation);
  draw();
}
