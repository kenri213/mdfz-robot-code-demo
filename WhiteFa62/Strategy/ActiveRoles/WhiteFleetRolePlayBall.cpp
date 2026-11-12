/**
 * @file WhiteFleetRolePlayBall.cpp
 *
 * This file implements the default ball playing behavior that selects the action to be executed.
 *
 * @author Arne Hasselbring
 * @author Jo Lienhoop
 */

#include "Debugging/Modify.h"
#include "WhiteFleetBehavior.h"
#include "WhiteFleetRolePlayBall.h"
#include "Representations/BehaviorControl/DribbleTarget.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Infrastructure/GameState.h"
#include "WhiteFleetMatchTacticalRepair.h"
#include "Tools/BehaviorControl/TeamActivityPolicy.h"
#include "Tools/BehaviorControl/Strategy/Agent.h"

WhiteFleetRolePlayBall::WhiteFleetRolePlayBall()
{
  InMapFile stream("rolePlayBall.cfg");
  ASSERT(stream.exists());
  stream >> p;
}

void WhiteFleetRolePlayBall::reset()
{
  lastPassTarget = 0;
}

void WhiteFleetRolePlayBall::preProcess()
{
  MODIFY("behavior:WhiteFleetRolePlayBall", p);
  DECLARE_DEBUG_DRAWING("behavior:WhiteFleetRolePlayBall:ratings", "drawingOnField");
}

SkillRequest WhiteFleetRolePlayBall::execute(const Agent& self, const Agents& teammates)
{
  return smashOrPass(self, teammates);
}

SkillRequest WhiteFleetRolePlayBall::smashOrPass(const Agent& self, const Agents& teammates)
{
  SmashOrPassInfo info = getSmashOrPassInfo(self, teammates);
  if(theGameState.isFreeKick())
  {
    return SkillRequest::Builder::waitShoot(info.passTarget);
  }
  return info.request;
}

WhiteFleetRolePlayBall::SmashOrPassInfo WhiteFleetRolePlayBall::getSmashOrPassInfo(const Agent& self, const Agents& teammates)
{
  SmashOrPassInfo info;
  if(p.alwaysShoot)
  {
    info.request = SkillRequest::Builder::shoot();
    return info;
  }
  // Calculate a penalty for changing the decision of the shoot or pass target based on the distance to the ball and the penalty value range
  decisionPenalty = mapToRange(self.ballPosition.norm(), p.ballDistMaxPenalty, p.ballDistMinPenalty, p.maxPenalty, p.minPenalty);
  // Get the estimated probability of shooting a goal from the current ball position
  const Vector2f ballPosition = self.pose * self.ballPosition;
  // No goal shots allowed if indirect kick rule applies

  theKickAtGoalEvaluation.update();
  float bestActionRating = theKickAtGoalEvaluation.kickInfo.isValid ? theExpectedGoals.getRating(ballPosition, false, false, false) : 0.f;
  float maxPassDistance = p.maxPassDistance;
  float minPassDistance = p.minPassDistance;
  if(!theIndirectKick.allowDirectKick)
  {
    // Minimize the own goal rating because the rules do not allow this robot to perform a direct kick.
    bestActionRating = p.minRating;
    maxPassDistance = p.maxFreeKickDistance;
    minPassDistance = p.minFreeKickPassDistance;
  }
  else
  {
    if(bestActionRating > (((theGameState.isGoalKick() || theGameControllerData.setPlay == SET_PLAY_DIRECT_FREE_KICK) && theGameState.isForOwnTeam()) ? p.shootGoalKickThreshold : p.shootThreshold))
    {
      lastPassTarget = -1;
      info.request = SkillRequest::Builder::shoot();
      return info;
    }
    dribbleTarget = theDribbleTarget.getTarget(theFieldBall.positionOnField);
    if(!(theGameState.isFreeKick() && theGameState.isForOwnTeam()) && !Tactic::Position::isGoalkeeper(self.position))
    {
      // Use only 50% of change
      ASSERT(dribbleTarget.has_value());
      float newBestActionRating = 0.5f * (theExpectedGoals.getRating(dribbleTarget.value(), false, false, false) - bestActionRating) + bestActionRating;
      if(lastPassTarget > 0)
        newBestActionRating -= decisionPenalty;
      bestActionRating = std::max(bestActionRating, newBestActionRating);
      if(bestActionRating > p.shootThreshold)
      {
        lastPassTarget = -1;
        info.request = SkillRequest::Builder::shoot();
        return info;
      }
    }
  }
  // Apply the decision penalty when this action is different from the last frame
  if(lastPassTarget > 0)
    bestActionRating -= decisionPenalty;

  const float bestShootActionRating = bestActionRating;
  const Agent* passTarget = nullptr;
  // TODO: Should the search start with ballPosition or recentBallPositionOnField?
  findPassTarget(theFieldBall.recentBallPositionOnField(), teammates, minPassDistance, maxPassDistance, p.maxSearchDepth, passTarget, bestActionRating, decisionPenalty, {});
  if(!passTarget && theFrameInfo.getTimeSince(lastPassTargetTimestamp) < p.rememberPassTargetTime)
  {
    for(const auto& teammate : teammates)
    {
      if(teammate->number == lastPassTarget)
        passTarget = teammate;
    }
  }
  // PowerAdvance: ordinary long advance only; calibrated restarts and short
  // receiver passes retain their finite-length path. Use an already-valid
  // native Strong shot, not a blind promotion of a pressured Dynamic kick.
  {
    const Vector2f ball=theFieldBall.positionOnField;
    const bool nativeStrong=theKickAtGoalEvaluation.kickInfo.isValid &&
      theKickAtGoalEvaluation.kickInfo.kickType==KickInfo::rlKickStrong;
    const float angle=theKickAtGoalEvaluation.kickInfo.kickDirection+theRobotPose.rotation;
    const float dx=theFieldDimensions.xPosOpponentGoalLine-ball.x();
    const float forward=std::cos(angle);
    const Vector2f target(theFieldDimensions.xPosOpponentGoalLine,
      forward>.1f?ball.y()+dx*std::sin(angle)/forward:std::numeric_limits<float>::infinity());
    const float planned=passTarget?(passTarget->pose.translation-ball).norm():(target-ball).norm();
    bool pressureFree=target.allFinite()&&dx>0.f&&forward>.1f&&
      std::abs(target.y())<theFieldDimensions.yPosLeftGoal-theFieldDimensions.goalPostRadius-theBallSpecification.radius-150.f;
    for(const auto& opponent:theGlobalOpponentsModel.opponents)
      if(!opponent.position.allFinite() || (opponent.position-ball).norm()<800.f ||
         (opponent.position.x()>ball.x()&&opponent.position.x()<target.x()&&
          WhiteFleetMatchTacticalRepair::segmentDistance({opponent.position.x(),opponent.position.y()},
            {ball.x(),ball.y()},{target.x(),target.y()})<500.f))pressureFree=false;
    for(const Agent* peer:teammates)
      if(TeamActivityPolicy::fresh(theFrameInfo.time,peer->timestamp,1200)&&
         peer->pose.translation.x()>ball.x()&&peer->pose.translation.x()<target.x()&&
         WhiteFleetMatchTacticalRepair::segmentDistance({peer->pose.translation.x(),peer->pose.translation.y()},
           {ball.x(),ball.y()},{target.x(),target.y()})<400.f)pressureFree=false;
    if(WhiteFleetMatchTacticalRepair::powerAdvance(WhiteFleetMatchTacticalRepair::enabled(theBehaviorParameters.matchTacticalRepair),
       theGameState.state==GameState::playing&&!theGameState.stopped&&theGameControllerData.stopped==0&&
         theGameState.playerState==GameState::active,
       theIndirectKick.allowDirectKick,theRobotPose.quality!=RobotPose::poor&&theRobotPose.isFinite(),self.isUpright,
       theFieldBall.timeSinceBallWasSeen,{ball.x(),ball.y()},theFieldDimensions.xPosOpponentGoalLine,
       theFieldBall.positionRelative.norm(),theFieldBall.velocityRelative.norm(),nativeStrong,planned,pressureFree))
    {
      lastPassTarget=-1;info.request=SkillRequest::Builder::shoot();return info;
    }
  }
  // End PowerAdvance; original short-pass and control decisions follow.
  if(passTarget)
  {
    lastPassTarget = passTarget->number;
    info.passTarget = passTarget->number;
    info.request = SkillRequest::Builder::passTo(passTarget->number);
    return info;
  }

  bool doesPassTargetExist = false;
  for(const auto& teammate : teammates)
  {
    if(!Tactic::Position::isGoalkeeper(teammate->position))
    {
      doesPassTargetExist = true;
      break;
    }
  }

  lastPassTargetTimestamp = 0;
  lastPassTarget = -1;

  if(theIndirectKick.allowDirectKick && bestShootActionRating >= p.riskyShootThreshold)
  {
    info.request = SkillRequest::Builder::shoot();
    return info;
  }
  else if(theGameState.isFreeKick() && theGameState.isForOwnTeam())
  {
    int timeLeftInState = -theFrameInfo.getTimeSince(theGameState.timeWhenStateEnds);
    //clear the ball when there is no passtarget
    //when clearing the ball is not possible dribble by calling the pass skill
    theClearTarget.update();
    if(!doesPassTargetExist || // No need to wait for a teammate, that does not exist
       (timeLeftInState < p.waitForPassTargetEndTime && theClearTarget.isValid && theClearTarget.rating > bestShootActionRating))
    {
      info.request = SkillRequest::Builder::clear();
      return info;
    }
    else
    {
      // Here, the pass skill request is set with an invalid pass target on purpose. This way, the waiting states of the pass skill are utilized during setplays. Outside of setplays, the shoot skill request is set and the robot will dribble immediately without waiting behind the ball.
      info.passTarget = 0;
      info.request = SkillRequest::Builder::passTo(0);
      return info;
    }
  }
  else
  {
    const float clearRating = theClearTarget.rating;
    if(theIndirectKick.allowDirectKick)
    {
      lastPassTarget = bestActionRating > p.minRating ? -1 : 0;
      float shootRating = theExpectedGoals.getRating(ballPosition, false, false, false);
      if(shootRating >= clearRating + clearDribbleHysteresis)
      {
        info.request = SkillRequest::Builder::shoot();
        return info;
      }
    }
    if(bestShootActionRating > clearRating + clearDribbleHysteresis || !theClearTarget.isValid)
    {
      // If goalkeeper, just clear the ball
      if(Tactic::Position::isGoalkeeper(self.position) && theClearTarget.isValid)
      {
        info.request = SkillRequest::Builder::clear();
        return info;
      }
      clearDribbleHysteresis = -p.clearDribbleHysteresisValue;
      if(theIndirectKick.allowDirectKick)
        info.request = SkillRequest::Builder::shoot();
      else
      {
        if(!dribbleTarget.has_value())
          dribbleTarget = theDribbleTarget.getTarget(theFieldBall.positionOnField);
        ASSERT(dribbleTarget.has_value());
        info.request = SkillRequest::Builder::dribbleTo((dribbleTarget.value() - theRobotPose.translation).angle());
      }

      return info;
    }
    else
    {
      clearDribbleHysteresis = p.clearDribbleHysteresisValue;
      info.request = SkillRequest::Builder::clear();
      return info;
    }
  }
}

float WhiteFleetRolePlayBall::findPassTarget(const Vector2f& ballPosition, const Agents& teammates, const float minPassDistance, const float maxPassDistance, int remainingSearchDepth, const Agent*& passTarget, float& bestActionRating, const float decisionPenalty, const std::vector<int>& ballPossessionList)
{
  if(remainingSearchDepth <= 0)
    return 0.f;

  const float shotRating = bestActionRating;
  float bestCombinedRating = 0.f;
  // Find a pass target i.e. the teammate with the highest rating for a successful pass (from the current ball position) and a following goal shot or another pass recursively (from the teammate's current position)
  for(const Agent* teammate : teammates)
  {
    if(Tactic::Position::isGoalkeeper(teammate->position) || std::count(ballPossessionList.begin(), ballPossessionList.end(), teammate->number))
      continue;

    // PassToTeammates requires the pass target to be known in the GlobalTeammatesModel, therefore use its information for planning
    const GlobalTeammatesModel::TeammateEstimate* globalTeammate = nullptr;
    for(const auto& t : theGlobalTeammatesModel.teammates)
    {
      if(t.playerNumber == teammate->number)
      {
        globalTeammate = &t;
        break;
      }
    }
    if(!globalTeammate)
      continue;

    const bool samePassTarget = lastPassTarget > 0 && lastPassTarget == globalTeammate->playerNumber;
    const Vector2f& passTargetPosition = globalTeammate->pose.translation;
    const float passDistance = (passTargetPosition - ballPosition).squaredNorm();
    if(passDistance < sqr((p.minPassDistance) - (samePassTarget ? p.passDistanceSameTargetHysteresis : 0.f)) ||
       passDistance > sqr(maxPassDistance + (samePassTarget ? p.passDistanceSameTargetHysteresis : 0.f)))
      continue;

    // Evaluation for the pass
    const TargetEvaluation lastTargetEvaluation = thePassTargetEvaluation.getBestPassData(ballPosition, passTargetPosition, globalTeammate->playerNumber);

    const float passRating = lastTargetEvaluation.isAngleFree && lastTargetEvaluation.isValid ? lastTargetEvaluation.passRating : 0.f;
    // Pruning, if path can't be better than best rating
    if(passRating < bestActionRating)
      continue;

    // Evaluation for a goal
    const float goalRating = theExpectedGoals.getRating(passTargetPosition, false, true, false) * theExpectedGoals.getOpponentRating(passTargetPosition);

    // Add current player to list, which tracks who had the ball
    std::vector<int> currentBallPossessionList = ballPossessionList;
    currentBallPossessionList.push_back(globalTeammate->playerNumber);

    // The teammate's rating is the estimated probability that the ball would reach the given target position when passed AND the teammate would score a direct goal from the pass target position
    const float nextRating = p.discountFactor * findPassTarget(passTargetPosition, teammates, minPassDistance, maxPassDistance, remainingSearchDepth - 1, passTarget, bestActionRating, decisionPenalty, currentBallPossessionList);
    float combinedRating = passRating * std::max(goalRating, nextRating);

    draw(ballPosition, passTargetPosition, combinedRating, 0.f, false);

    if(remainingSearchDepth == p.maxSearchDepth)
    {
      // Apply the decision penalty when this action is different from the last frame
      const bool applyDecisionPenalty = lastPassTarget > 0 && lastPassTarget != globalTeammate->playerNumber;

      if(applyDecisionPenalty)
        combinedRating -= decisionPenalty;
      if(combinedRating > bestActionRating && combinedRating > shotRating * p.passShotRatio)
      {
        bestActionRating = combinedRating;
        passTarget = teammate;
      }
    }
    if(combinedRating > bestCombinedRating)
      bestCombinedRating = combinedRating;
  }
  return bestCombinedRating;
}

void WhiteFleetRolePlayBall::draw([[maybe_unused]] const Vector2f& passBasePosition, [[maybe_unused]] const Vector2f& passTargetPosition, [[maybe_unused]] const float value, [[maybe_unused]] const float penalty, const bool applyPenalty)
{
  COMPLEX_DRAWING("behavior:WhiteFleetRolePlayBall:ratings")
  {
    float maxPassDistance = (theGameState.isFreeKick() && theGameState.isForOwnTeam()) || !theIndirectKick.allowDirectKick ?
                            (theGameState.isGoalKick() ? p.maxGoalKickDistance : p.maxFreeKickDistance) :
                            p.maxPassDistance;
    CIRCLE("behavior:WhiteFleetRolePlayBall:ratings", passBasePosition.x(), passBasePosition.y(), maxPassDistance, 20, Drawings::solidPen, ColorRGBA::violet, Drawings::noPen, ColorRGBA::violet);
    LINE("behavior:WhiteFleetRolePlayBall:ratings", passBasePosition.x(), passBasePosition.y(), passTargetPosition.x(), passTargetPosition.y(), 20, Drawings::dashedPen, applyPenalty ? ColorRGBA::violet : ColorRGBA::blue);
    DRAW_TEXT("behavior:WhiteFleetRolePlayBall:ratings", passTargetPosition.x(), passTargetPosition.y(), 250, ColorRGBA::blue, value);
    if(applyPenalty)
      DRAW_TEXT("behavior:WhiteFleetRolePlayBall:ratings", passTargetPosition.x(), passTargetPosition.y() - 250, 250, ColorRGBA::violet, value - penalty);
  }
}
