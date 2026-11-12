/**
 * @file DirectKickOff.cpp
 *
 * This ...
 *
 * @author Arne Hasselbring (from former KickoffStrikerCard.cpp in 2019)
 */

#include "WhiteFleetSkillBehaviorControl.h"
#include "WhiteFleetSimPhysical.h"
#include "Tools/BehaviorControl/SectorWheel.h"
#include "Debugging/DebugDrawings.h"
#include "WhiteFleetK1V21MatchProfile.h"
#include "WhiteFleetBoosterOfficialLiveTest.h"
#include "WhiteFleetBoosterRemoteStand.h"
#include "WhiteFleetBoosterPickupControl.h"
#include "WhiteFleetC15KickoffSwitch.h"
#include "WhiteFleetAutonomousKickoff.h"
#include "Tools/BehaviorControl/TeamActivityPolicy.h"
#include <cstdio>

namespace DirectKickOff
{
  struct ObstacleSector
  {
    Rangea sector; /**< The angular range relative to the ball that the obstacle blocks. */
    float distance; /**< The distance of the obstacle to the ball. */
    float x; /**< The x coordinate on field of the obstacle. */
  };
}
using namespace DirectKickOff;

option((WhiteFleetSkillBehaviorControl) DirectKickOff,
       load((float) hysteresisNumber, /**< A number which is used in various places to define a hysteresis offset. */
            (Angle) hysteresisAngle, /**< When sorting sectors by their opening angle, the one selected in the previous frame gets this as bonus. */
            (Angle) goalAngle, /**< No kicks towards the goal allowed. */
            (Angle) minOpeningAngle, /**< The minimum opening angle a sector must have to be considered. */
            (float) ballDistanceForSlowWalk, /**< If the ball is close, walk slow. */
            (float) xPoseForPassReceiver, /**< X coordinate for pass receiver. */
            (int) forceKickAfterTime, /**< Force a kick after this much time passed. */
            (bool) kickOutOfField, /**< Kick out of the field to force a set play. */
            (bool) kickOutOfField2Players, /**< Kick out of field if we have 2 or less players. */
            (float) maxKickLength), /**< Max kick length for normal kick off. */
       vars((bool)(false) wasActive, /**< Whether an in walk kick out of the center circle was already tried. */
            (KickInfo::KickType)(KickInfo::numOfKickTypes) kickType, /**< The kick type to try. */
            (Angle)(0_deg) targetAngle,
            (unsigned)(0) autonomousLastLog)) /**< The event intent belongs to the module, not resettable option vars. */
{
  const auto autonomousKickoff = [&]() -> bool
  {
    const bool physical=WhiteFleetSimPhysical::physical();
    if(!WhiteFleetAutonomousKickoff::enabled(physical,WhiteFleetK1V21MatchProfile::guardedPhysicalReceiver(),
         WhiteFleetSimPhysical::simulated(),Global::getSettings().teamNumber,
         std::getenv("MDFZ_AUTONOMOUS_KICKOFF_V4_ENABLE")))return false;
    // Shared playbook identity also protects this entry from an old/local
    // shoot request on the receiver. All referee/body/ball gates below remain.
    const bool owner=WhiteFleetAutonomousKickoff::isKicker(theGameState.playerNumber)&&
        theGameState.ownTeam.playerStates[WhiteFleetAutonomousKickoff::kickerNumber-Settings::lowestValidPlayerNumber]==GameState::active;
    const bool legal=theGameState.state==GameState::ownKickOff&&theGameState.isForOwnTeam()
        &&!theGameState.stopped&&!theGameControllerData.stopped&&theGameState.playerState==GameState::active;
    const bool body=(!physical||WhiteFleetSimPhysical::bodyAuthorized())&&!theSystemSensorData.emergencyMode
        &&!theEmergencyButtonState.buttonPressed&&!theBehaviorStateInformation.remoteControlActive
        &&!WhiteFleetBoosterPickupControl::held()&&!WhiteFleetBoosterRemoteStand::held()
        &&theFallDownState.state==FallDownState::upright
        &&(theMotionInfo.executedPhase==MotionPhase::stand||theMotionInfo.executedPhase==MotionPhase::walk);
    const bool fresh=theBallModel.timeWhenLastSeen>0&&theBallModel.timeWhenLastSeen<=theFrameInfo.time
        &&theFrameInfo.time-theBallModel.timeWhenLastSeen<=250u&&theFieldBall.ballWasSeen(250)
        &&theBallModel.estimate.position.allFinite()&&theFieldBall.positionRelative.allFinite()&&!observationHeld;
    // Position uncertainty must not require centimetre-perfect placement. A
    // seriously uncertain/nonfinite heading still cannot authorize a blind ray.
    const float headingVariance=theRobotPose.covariance(2,2);
    const bool headingUsable=std::isfinite(headingVariance)&&headingVariance>=0.f&&headingVariance<=.616850275f; // sigma <=45deg.
    const bool admitted=owner&&!theGameState.isGoalkeeper()&&legal&&body&&fresh&&theRobotPose.isFinite()&&headingUsable;
    const char* reason=!owner?"not_native_kicker":!legal?"referee_or_penalty":!body?"body_authority_or_fault":
        !fresh?"no_fresh_own_ball":!theRobotPose.isFinite()?"nonfinite_pose":!headingUsable?"heading_unreliable":"goalkeeper";
    WhiteFleetAutonomousKickoff::Intent pass;
    if(admitted)
    {
      const Vector2f ball=theRobotPose*theBallModel.estimate.position;
      const WhiteFleetKickoffThroughV8::Field field{theFieldDimensions.xPosOwnGoalLine,theFieldDimensions.xPosOpponentGoalLine,
          theFieldDimensions.xPosOpponentPenaltyArea,theFieldDimensions.yPosLeftTouchline,theFieldDimensions.yPosLeftGoal};
      if(autonomousEvent!=theGameState.timeWhenStateStarted)
      {
        std::vector<WhiteFleetAutonomousKickoff::Blocker> blockers;
        for(const Obstacle& obstacle:theObstacleModel.obstacles)
        {
          if(obstacle.isTeammate()||!TeamActivityPolicy::fresh(theFrameInfo.time,obstacle.lastSeen,600u)||
             !obstacle.center.allFinite()||!obstacle.left.allFinite()||!obstacle.right.allFinite())continue;
          const float xVariance=obstacle.covariance(0,0),yVariance=obstacle.covariance(1,1);
          if(!std::isfinite(xVariance)||!std::isfinite(yVariance)||xVariance<0.f||yVariance<0.f)continue;
          const float variance=std::max(xVariance,yVariance);
          const Vector2f center=theRobotPose*obstacle.center;
          const float radius=(obstacle.left-obstacle.right).norm()*.5f+theBallSpecification.radius+200.f+
                             2.f*std::sqrt(variance);
          if(center.allFinite()&&std::isfinite(radius))blockers.push_back({{center.x(),center.y()},radius});
        }
        autonomousReceiver=-1;
        if(!theGameState.ownTeam.isGoalkeeper(WhiteFleetAutonomousKickoff::receiverNumber)&&
           theGameState.ownTeam.playerStates[WhiteFleetAutonomousKickoff::receiverNumber-Settings::lowestValidPlayerNumber]==GameState::active)
          autonomousReceiver=WhiteFleetAutonomousKickoff::receiverNumber; // No ACK/readiness wait, no alternate recipient.
        WhiteFleetKickoffThroughV8::Point receiver{};bool receiverKnown=false;
        for(const Teammate& teammate:theTeamData.teammates)
          if(teammate.number==autonomousReceiver&&TeamActivityPolicy::fresh(theFrameInfo.time,teammate.theFrameInfo.time,1200u)&&
             teammate.theRobotPose.isFinite())
          {receiver={teammate.theRobotPose.translation.x(),teammate.theRobotPose.translation.y()};receiverKnown=true;break;}
        pass=WhiteFleetAutonomousKickoff::laneIntent({ball.x(),ball.y()},field,WhiteFleetC15KickoffSwitch::forwardPrepareParams(),
            theKickInfo,theBallSpecification,blockers,receiverKnown?&receiver:nullptr);
        if(pass.valid)
        {
          const float before=BallPhysics::velocityForDistance(pass.nominalLength,theBallSpecification);
          pass.nominalLength=WhiteFleetAutonomousKickoff::strongerNominal(pass,{ball.x(),ball.y()},field,
              WhiteFleetC15KickoffSwitch::forwardPrepareParams(),theKickInfo,theBallSpecification);
          const float after=BallPhysics::velocityForDistance(pass.nominalLength,theBallSpecification);
          std::printf("AUTONOMOUS_KICKOFF_POWER self=%d event=%u before_cmd=%.1f after_cmd=%.1f native_cap=%.1f model_unchanged=1 P_unchanged=1\n",
              theGameState.playerNumber,theGameState.timeWhenStateStarted,before,after,theKickInfo[KickInfo::rlKickDynamic].ballVelocity.max);
          autonomousEvent=theGameState.timeWhenStateStarted;
          autonomousTarget={pass.target.x,pass.target.y};autonomousLength=pass.nominalLength;
          std::printf("AUTONOMOUS_KICKOFF_LANE self=%d event=%u receiver=%d P=%.1f,%.1f observed=%u clearance_mm=%.1f velocity_prediction=0 ack_wait=0 frozen=1\n",
              theGameState.playerNumber,autonomousEvent,autonomousReceiver,pass.target.x,pass.target.y,
              pass.observedBlockers,pass.observedClearance);
        }
      }
      else
      {
        pass.valid=autonomousTarget.allFinite()&&std::isfinite(autonomousLength)&&autonomousLength>0.f;
        pass.target={autonomousTarget.x(),autonomousTarget.y()};pass.nominalLength=autonomousLength;
        pass.reason="native_frozen_lane_intent";
      }
      reason=pass.reason;
      if(pass.valid)
      {
        const Vector2f target(pass.target.x,pass.target.y);
        const Vector2f ray=target-ball;
        PassTarget({.passTarget=autonomousReceiver,.ballTarget=theRobotPose.inverse()*target}); // Existing addressed native intent, never an ACK/contact claim.
        GoToBallAndKick({.targetDirection=Angle::normalize(ray.angle()-theRobotPose.rotation),
            .kickType=KickInfo::rlKickDynamic,.lookActiveWithBall=false,
            .alignPrecisely=KickPrecision::normal,.length=pass.nominalLength});
      }
    }
    if(!admitted||!pass.valid){LookActive({.withBall=true});Stand();}
    if(!autonomousLastLog||theFrameInfo.getTimeSince(autonomousLastLog)>=1000)
    {
      autonomousLastLog=theFrameInfo.time;
      std::printf("AUTONOMOUS_KICKOFF self=%d request=%d reason=%s P=%.1f,%.1f nominal_mm=%.1f range_limited=%d peer_ack_required=0 opponent_observation_required=0 tactical_score_required=0 native_motion_unchanged=1\n",
          theGameState.playerNumber,admitted&&pass.valid,reason,pass.target.x,pass.target.y,pass.nominalLength,pass.rangeLimited);
    }
    return true;
  };
  initial_state(execute)
  {
    transition
    {
      if(theFrameInfo.getTimeSince(theExtendedGameState.timeWhenStateStarted[GameState::ownKickOff]) > forceKickAfterTime && wasActive)
        goto kick;
    }
    action
    {
      if(autonomousKickoff())return;
      if(theFrameInfo.getTimeSince(theExtendedGameState.timeWhenStateStarted[GameState::ownKickOff]) < forceKickAfterTime)
      {
        LookLeftAndRight({.tilt = theBehaviorParameters.defaultLookDownAngle});
        Stand();
      }
      else
      {
        // Prepare obstacle sectors.
        std::vector<ObstacleSector> obstacleSectors;
        for(const Obstacle& obstacle : theObstacleModel.obstacles)
        {
          const Vector2f obstacleOnField = theRobotPose * obstacle.center;
          if(obstacleOnField.x() < 0.f || obstacleOnField.squaredNorm() < sqr(theFieldDimensions.centerCircleRadius - 300.f))
            continue;

          const float width = (obstacle.left - obstacle.right).norm() + 6.f * theBallSpecification.radius;
          const float distance = std::sqrt(std::max((obstacleOnField - theFieldBall.positionOnField).squaredNorm() - sqr(width / 2.f), 1.f));
          if(distance < theBallSpecification.radius)
            continue;

          const float radius = std::atan(width / (2.f * distance));
          const Angle direction = (obstacleOnField - theFieldBall.positionOnField).angle();

          obstacleSectors.emplace_back();
          obstacleSectors.back().sector = Rangea(Angle::normalize(direction - radius), Angle::normalize(direction + radius));
          obstacleSectors.back().distance = distance;
          obstacleSectors.back().x = obstacleOnField.x();
          // If the previous target is inside this sector, artificially increase its x coordinate so it will potentially be culled before other sectors.
          if(wasActive && obstacleSectors.back().sector.isInside(targetAngle))
            obstacleSectors.back().x += hysteresisNumber;
        }

        obstacleSectors.emplace_back();
        obstacleSectors.back().sector = Rangea(-91_deg, -89_deg);
        obstacleSectors.back().distance = 1000.f;
        obstacleSectors.back().x = 0.f;

        obstacleSectors.emplace_back();
        obstacleSectors.back().sector = Rangea(89_deg, 91_deg);
        obstacleSectors.back().distance = 1000.f;
        obstacleSectors.back().x = 0.f;

        obstacleSectors.emplace_back();
        obstacleSectors.back().sector = Rangea(-goalAngle, goalAngle);
        obstacleSectors.back().distance = 1000.f;
        obstacleSectors.back().x = 0.f;

        // Sort obstacles according to their absolute x coordinate (to ease culling later on).
        std::sort(obstacleSectors.begin(), obstacleSectors.end(), [](const ObstacleSector& s1, const ObstacleSector& s2) { return s1.x < s2.x; });

        SectorWheel wheel;
        std::list<SectorWheel::Sector> sectors;
        bool isLargeEnough = false;
        do
        {
          wheel.begin(theFieldBall.positionOnField);
          for(const ObstacleSector& obstacleSector : obstacleSectors)
            wheel.addSector(obstacleSector.sector, obstacleSector.distance, SectorWheel::Sector::obstacle);
          sectors = wheel.finish();

          for(const SectorWheel::Sector& sector : sectors)
            if(sector.type == SectorWheel::Sector::free &&
               sector.angleRange.getSize() >= ((wasActive && sector.angleRange.isInside(targetAngle)) ? minOpeningAngle : Angle(minOpeningAngle * 2.f)))
            {
              isLargeEnough = true;
              break;
            }
        }
        while(!isLargeEnough && !obstacleSectors.empty() && (obstacleSectors.pop_back(), true));

        DRAW_SECTOR_WHEEL("option:DirectKickOff:wheel", sectors, theFieldBall.endPositionOnField);

        bool passTeammate = false;
        if(theGameState.ownTeam.numOfActivePlayers() > 2)
        {
          for(const auto& t : theGlobalTeammatesModel.teammates)
          {
            Vector2f teammatePose = t.pose.translation;
            if(teammatePose.x() > xPoseForPassReceiver)
            {
              targetAngle = teammatePose.y() > 0 ? 45_deg : -45_deg;
              passTeammate = true;
              break;
            }
          }
        }

        Angle bestOpeningAngle = 0_deg;
        Angle bestTargetAngle = 0_deg;
        Angle bestCenter = 180_deg;
        bool foundTeammateAngle = false;

        for(const SectorWheel::Sector& sector : sectors)
        {
          if(std::abs(sector.angleRange.getCenter()) > 90_deg
             || sector.type != SectorWheel::Sector::free)
            continue;
          const Angle openingAngle = sector.angleRange.getSize() + ((wasActive && sector.angleRange.isInside(targetAngle)) ? hysteresisAngle : 0_deg);
          const bool betterCenter = std::abs(sector.angleRange.getCenter()) < std::abs(bestCenter);
          const bool teammateSide = passTeammate && sector.angleRange.getCenter() * targetAngle >= 0.f;

          if((betterCenter && (!foundTeammateAngle || teammateSide))
             || (teammateSide && !foundTeammateAngle))
          {
            foundTeammateAngle |= sector.angleRange.getCenter() * targetAngle >= 0.f && passTeammate;
            bestCenter = sector.angleRange.getCenter();
            bestOpeningAngle = openingAngle;
            // kickOutOfField: as close to forward as possible, otherwise soft and far away from opponent
            bestTargetAngle = kickOutOfField || (kickOutOfField2Players && theGameState.ownTeam.numOfActivePlayers() <= 2) ? sector.angleRange.limit(0) : sector.angleRange.getCenter();
          }
        }

        ASSERT(bestOpeningAngle > 0_deg);

        targetAngle = bestTargetAngle;
        float kickLength = Vector2f(theFieldDimensions.xPosOpponentGoal / 2.f, theFieldDimensions.yPosLeftTouchline / 2.f).norm();
        kickType = (!kickOutOfField || (kickOutOfField2Players && theGameState.ownTeam.numOfActivePlayers() <= 2)) ? KickInfo::rlKickDynamicOld : KickInfo::rlKickDynamic;

        ASSERT(kickType != KickInfo::numOfKickTypes);
        GoToBallAndKick({.targetDirection = Angle::normalize(targetAngle - theRobotPose.rotation),
                         .kickType = kickType,
                         .lookActiveWithBall = true,
                         .length = kickLength,
                         .reduceWalkSpeedType = theFieldBall.positionRelative.squaredNorm() < sqr(ballDistanceForSlowWalk) ? ReduceWalkSpeedType::slow : ReduceWalkSpeedType::noChange});
        wasActive = true;
      }
    }
  }

  state(kick)
  {
    action
    {
      if(autonomousKickoff())return;
      const float kickLength = kickOutOfField || (kickOutOfField2Players && theGameState.ownTeam.numOfActivePlayers() <= 2)
                               ? std::numeric_limits<float>::max()
                               : std::min(maxKickLength, Vector2f(theFieldDimensions.xPosOpponentGoal / 2.f, theFieldDimensions.yPosLeftTouchline / 2.f).norm());
      kickType = (!kickOutOfField || (kickOutOfField2Players && theGameState.ownTeam.numOfActivePlayers() <= 2)) ? KickInfo::rlKickDynamicOld : KickInfo::rlKickDynamic;

      GoToBallAndKick({.targetDirection = Angle::normalize(targetAngle - theRobotPose.rotation),
                       .kickType = kickType,
                       .lookActiveWithBall = true,
                       .length = kickLength,
                       .reduceWalkSpeedType = theFieldBall.positionRelative.squaredNorm() < sqr(ballDistanceForSlowWalk) ? ReduceWalkSpeedType::slow : ReduceWalkSpeedType::noChange });
    }
  }
}
