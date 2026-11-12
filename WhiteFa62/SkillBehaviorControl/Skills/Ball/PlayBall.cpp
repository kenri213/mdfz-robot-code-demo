/**
 * @file PlayBall.cpp
 *
 * This file defines an implementation of a skill that plays the ball (under consideration of the skill request).
 *
 * @author Arne Hasselbring
 */

#include "WhiteFleetBackfieldOutlet.h"
#include <cstdio>
#include "WhiteFleetSkillBehaviorControl.h"
#include "WhiteFleetSimPhysical.h"
#include "WhiteFleetMatchTacticalRepair.h"
#include "WhiteFleetAutonomousKickoff.h"

option((WhiteFleetSkillBehaviorControl) PlayBall,
       load((float) goalPostHandlingAreaRadius,
            (float) goalPostHandlingAreaHysteresisMultiplier,
            (float) duelMinDistanceToClosest,
            (Angle) duelMinAngleToClosest))
{
  const auto activateDuel = [&]
  {
    if(theSkillRequest.mdfzRule || theSkillRequest.restartPassRequired ||
       (theSkillRequest.backfieldStrongOutlet&&theSkillRequest.skill==SkillRequest::pass) ||
       ((theGameState.isKickOff() || theGameState.isFreeKick()) && theGameState.isForOwnTeam()))
      return false;

    const auto [distanceToClosestObstacle, angleToClosestObstacle, smallestAngleToCloseObstacle] = [&]() -> std::tuple<float, Angle, Angle>
    {
      float x = std::numeric_limits<float>::max();
      Angle y = 0_deg;
      Angle z = 180_deg;
      for(const auto& o : theObstacleModel.obstacles)
      {
        const float d = (o.center - theFieldBall.positionRelative).squaredNorm();
        if(d < x)
        {
          x = d;
          y = o.center.angle();
          if(x < sqr(duelMinDistanceToClosest))
            z = std::min(Angle(std::abs(y)), z);
        }
      }
      return
      {
        std::sqrt(x), y, z
      };
    }();

    const bool obstacleClose = distanceToClosestObstacle < (duelMinDistanceToClosest * (1.f - (std::abs(angleToClosestObstacle) / duelMinAngleToClosest / 3.f)));
    const bool obstacleAngleClose = smallestAngleToCloseObstacle <= duelMinAngleToClosest;
    const bool ballSeen = theFieldBall.ballWasSeen(300);
    const bool ballPosXClose = between<float>(theFieldBall.positionRelative.x(), 0.f, 2000.f);
    const bool ballPosYClose = std::abs(theFieldBall.positionRelative.y()) < 500.f;
    const bool rotationToBallOk = std::abs(theFieldBall.positionRelative.angle()) < 80_deg;

    return obstacleClose && obstacleAngleClose &&
           ballSeen && ballPosXClose && ballPosYClose && rotationToBallOk;
  };

  const auto stopDuel = [&]
  {
    if(theSkillRequest.restartPassRequired ||
       (theSkillRequest.backfieldStrongOutlet&&theSkillRequest.skill==SkillRequest::pass) ||
       ((theGameState.isKickOff() || theGameState.isFreeKick()) && theGameState.isForOwnTeam()))
      return true;

    if(theFieldBall.positionRelative.x() > 2500.f)
      return true;

    for(const Obstacle& omo : theObstacleModel.obstacles)
    {
      const float distanceToObstacle = (omo.center - theFieldBall.positionRelative).norm();
      const float angleToObstacle = std::abs(omo.center.angle());

      if(distanceToObstacle < (duelMinDistanceToClosest * 1.2f * (1.f - (std::abs(angleToObstacle) / pi / 2.f))))
        return false;
    }
    return true;
  };

  const bool isLeft = theFieldBall.positionOnField.y() > 0.f;
  const bool tactical=WhiteFleetMatchTacticalRepair::enabled(theBehaviorParameters.matchTacticalRepair);
  const Vector2f usedGoalPost(std::min(theFieldDimensions.xPosOwnGoalPost, theFieldBall.positionOnField.x() - theFieldDimensions.goalPostRadius), isLeft ? theFieldDimensions.yPosLeftGoal : theFieldDimensions.yPosRightGoal);

  const auto backfield = [&](Angle& relDirection,float& length) -> bool
  {
    using namespace WhiteFleetBackfieldOutlet;
    const int v=variant();
    if(v==off||theGameState.isKickOff()||theGameState.isFreeKick()||theGameState.isPenaltyKick()||
       theGameState.isGoalkeeper()||!theFieldBall.ballWasSeen(300)||!theRobotPose.isFinite())return false;
    Input in;in.ball=theFieldBall.positionOnField;in.robot=theRobotPose.translation;in.halfWidth=theFieldDimensions.yPosLeftTouchline;
    for(const auto& o:theGlobalOpponentsModel.opponents)if(o.position.allFinite())in.opponents.push_back(o.position);
    for(const auto& o:theObstacleModel.obstacles)if(o.isOpponent()&&o.center.allFinite())in.opponents.push_back(theRobotPose*o.center);
    for(const Teammate& t:theTeamData.teammates)
      if(t.number==3&&t.number!=theGameState.playerNumber&&theFrameInfo.getTimeSince(t.theFrameInfo.time)<1200&&t.theRobotPose.isFinite())
      {in.frontKnown=true;in.front=t.theRobotPose.translation;}
    if(!active(in))return false;
    const float dir=direction(in,v,length);
    relDirection=Angle::normalize(dir-theRobotPose.rotation);
    static unsigned lastLog=0;
    if(SystemCall::getMode()==SystemCall::simulatedRobot&&theFrameInfo.getTimeSince(lastLog)>=1000)
    {
      lastLog=theFrameInfo.time;
      std::printf("BACKFIELD_OUTLET self=%d v=%d ball=%.0f,%.0f dir_deg=%.0f len=%.0f opp_mm=%.0f front=%d\n",theGameState.playerNumber,v,
        in.ball.x(),in.ball.y(),dir*57.29578f,length,nearestOpponent(in),int(in.frontKnown));
    }
    return true;
  };

  initial_state(executeSkillRequest)
  {
    transition
    {
      if(theGameState.isKickOff())
        goto kickOff;
      else if(!theGameState.isFreeKick() && (theFieldBall.positionOnField - usedGoalPost).squaredNorm() < sqr(goalPostHandlingAreaRadius))
        goto atOwnGoalPost;
      {Angle d=0_deg;float l=0.f;
       if(activateDuel()&&!backfield(d,l)) // a back-field outlet (tested variant) takes precedence over the duel
         goto zweikampf;}
    }
    action
    {
      // 改动：离对方球门 4 米以内拿到球，直接射门（不再传球或带球）
      const Vector2f opponentGoalCenter(theFieldDimensions.xPosOpponentGoalLine, 0.f);
      const bool nearOpponentGoal = (theFieldBall.positionOnField - opponentGoalCenter).norm() < 4000.f;
      Angle backfieldDirection=0_deg;float backfieldLength=0.f;
      if(nearOpponentGoal)
        KickAtGoal();
      else if(!theSkillRequest.mdfzRule&&backfield(backfieldDirection,backfieldLength))
        GoToBallAndKick({.targetDirection=backfieldDirection,.kickType=KickInfo::rlKickStrong,
          .alignPrecisely=KickPrecision::justHitTheBall,.length=backfieldLength});
      else
      switch(theSkillRequest.skill)
      {
        case SkillRequest::pass:
          PassToTeammate({.playerNumber = theSkillRequest.passTarget});
          break;
        case SkillRequest::dribble:
          GoToBallAndDribble({.targetDirection = Angle::normalize(theSkillRequest.target.rotation - theRobotPose.rotation)});
          break;
        case SkillRequest::clear:
          if(theSkillRequest.mdfzRule)
          {
            const Vector2f b=theFieldBall.positionOnField;
            const Vector2f t(std::min(b.x()+8000.f,theFieldDimensions.xPosOpponentPenaltyMark),b.y()*.5f);
            GoToBallAndKick({.targetDirection=Angle::normalize((t-b).angle()-theRobotPose.rotation),
              .kickType=KickInfo::rlKickStrong,.length=std::min(8000.f,std::max(3000.f,(t-b).norm()))});
          }
          else if(tactical && theGameState.isGoalkeeper() && theGameState.isGoalKick() && theGameState.isForOwnTeam())
          {
            theClearTarget.update();
            if(theClearTarget.isValid && theFieldBall.ballWasSeen(200) &&
               std::isfinite(theClearTarget.kickLength) && theClearTarget.kickLength>=500.f &&
               std::cos(theClearTarget.kickAngle+theRobotPose.rotation)>.15f)
              GoToBallAndKick({.targetDirection=theClearTarget.kickAngle,
                .kickType=KickInfo::rlKickStrong,.length=std::min(12000.f,theClearTarget.kickLength)});
            else {LookActive({.withBall=true});Stand();}
          }
          else if(tactical&&theSkillRequest.backfieldStrongOutlet)
          {
            theClearTarget.update();
            if(theClearTarget.isValid&&theFieldBall.ballWasSeen(200)&&std::isfinite(theClearTarget.kickLength)&&
               theClearTarget.kickLength>=4000.f&&std::cos(theClearTarget.kickAngle+theRobotPose.rotation)>.15f)
              GoToBallAndKick({.targetDirection=theClearTarget.kickAngle,
                .kickType=KickInfo::rlKickStrong,.length=std::min(12000.f,theClearTarget.kickLength)});
            else ClearBall(); // Native legal emergency clear only, never a short teammate pass.
          }
          else ClearBall();
          break;
        case SkillRequest::shoot:
        default:
          KickAtGoal();
          break;
      }
    }
  }

  state(atOwnGoalPost)
  {
    transition
    {
      if((theFieldBall.positionOnField - usedGoalPost).squaredNorm() > sqr(goalPostHandlingAreaRadius * goalPostHandlingAreaHysteresisMultiplier))
      {
        if(activateDuel())
          goto zweikampf;
        goto executeSkillRequest;
      }
    }
    action
    {
      HandleBallAtOwnGoalPost();
    }
  }

  state(zweikampf)
  {
    transition
    {
      if((theFieldBall.positionOnField - usedGoalPost).squaredNorm() < sqr(goalPostHandlingAreaRadius))
        goto atOwnGoalPost;
      {Angle d=0_deg;float l=0.f;
       if(theSkillRequest.mdfzRule||stopDuel()||backfield(d,l))
         goto executeSkillRequest;}
    }
    action
    {
      Zweikampf();
    }
  }

  state(kickOff)
  {
    transition
    {
      if(!theGameState.isKickOff())
        goto executeSkillRequest;
    }
    action
    {
      if(WhiteFleetAutonomousKickoff::enabled(WhiteFleetSimPhysical::physical(),
           WhiteFleetK1V21MatchProfile::guardedPhysicalReceiver(),WhiteFleetSimPhysical::simulated(),
           Global::getSettings().teamNumber,std::getenv("MDFZ_AUTONOMOUS_KICKOFF_V4_ENABLE"))
         &&theGameState.state==GameState::ownKickOff&&theGameState.isForOwnTeam())
        DirectKickOff();
      else if(tactical && theGameState.isForOwnTeam())
      {
        if(theSkillRequest.skill==SkillRequest::pass && theSkillRequest.passTarget>0 &&
           theSkillRequest.target.translation.allFinite() && theSkillRequest.target.translation.x()>0.f)
          PassToTeammate({.playerNumber=theSkillRequest.passTarget});
        else {LookActive({.withBall=true});Stand();}
      }
      else DirectKickOff();
    }
  }
}
