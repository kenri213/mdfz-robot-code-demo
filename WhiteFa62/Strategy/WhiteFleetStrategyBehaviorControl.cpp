#include "WhiteFleetSimEntry.h"
#include "WhiteFleetSimPhysical.h"
/**
 * @file WhiteFleetStrategyBehaviorControl.cpp
 *
 * This file implements a module that determines the strategy of the team.
 *
 * @author Arne Hasselbring
 */

#include "WhiteFleetStrategyBehaviorControl.h"
#include "Representations/Communication/TeamData.h"
#include "Tools/BehaviorControl/Strategy/BehaviorBase.h"
#include "Tools/Modeling/BallPhysics.h"
#include "Tools/BehaviorControl/TeamActivityPolicy.h"
#include "WhiteFleetDefenseCounterSwitch.h"
#include "WhiteFleetMatchTacticalRepair.h"
#include "Tools/BehaviorControl/KickSelection.h"
#include "WhiteFleetC15KickoffSwitch.h"
#include "WhiteFleetC10KickoffPlanner.h"
#include "WhiteFleetAutonomousKickoff.h"
#include "Representations/Configuration/KickLengthPair.h"
#include "Tools/Motion/KickLengthConverter.h"
#include "Debugging/Annotation.h"
#include <cstdio>
#include <cstring>
#include <algorithm>
#include "Platform/File.h"
#include <sstream>
#include <string>
#include <vector>

MAKE_MODULE(WhiteFleetStrategyBehaviorControl, WhiteFleetStrategyBehaviorControl::getExtModuleInfo);

WhiteFleetStrategyBehaviorControl::WhiteFleetStrategyBehaviorControl() :
  theBehavior(theBallSearchParticles, theBallSpecification, theBehaviorParameters,
              theExtendedGameState, theFieldBall, theFieldDimensions, theFieldInterceptBall,
              theFrameInfo, theGameState, theIndirectKick, theTeamBallModel, theWalkingEngineOutput)
{
  // MDFZ arena: a class's own formation. The file exists only in that class's scenario copy.
  File file("mdfzFormation.cfg", "r");
  if(file.exists())
  {
    char text[512] = {0};
    file.read(text, std::min<size_t>(file.getSize(), sizeof(text) - 1));
    mdfzFormation.enabled = std::strstr(text, "enabled = true;") != nullptr;
    if(const char* p = std::strstr(text, "followBall = "))
      mdfzFormation.followBall = std::max(0.f, std::min(.8f, static_cast<float>(std::atof(p + 13))));
    std::printf("MDFZ_FORMATION_CONFIG team=%d enabled=%d followBall=%.2f\n", Global::getSettings().teamNumber, mdfzFormation.enabled, mdfzFormation.followBall);
    std::fflush(stdout);
  }
  // MDFZ arena: a class's own rules (behaviour workshop). The file exists only in that class's scenario copy.
  File rulesFile("mdfzRules.cfg", "r");
  if(rulesFile.exists())
  {
    std::string text(std::min<size_t>(rulesFile.getSize(), 8192), '\0');
    rulesFile.read(&text[0], text.size());
    static const char* names[] = {"ballX", "ballY", "absBallY", "ownGoalDist", "oppGoalDist", "myBallDist", "oppBallDist", "scoreDiff"};
    static const char* ballActions[] = {"shoot", "clear", "pass", "dribble", "native"};
    static const char* supportActions[] = {"goto", "follow", "guard", "mark", "native"};
    std::istringstream lines(text);
    std::string line;
    while(std::getline(lines, line) && mdfzRules.rules.size() < 16)
    {
      std::istringstream in(line);
      std::string role, action, tok;
      if(!(in >> role >> action) || (role != "ball" && role != "support"))
        continue;
      MdfzRule rule;
      rule.ball = role == "ball";
      rule.text = line;
      rule.action = -1;
      for(int i = 0; i < 5; ++i)
        if(action == (rule.ball ? ballActions[i] : supportActions[i]))
          rule.action = i;
      if(rule.action < 0)
        continue;
      bool ok = true, inConds = false;
      int numArgs = 0;
      while(in >> tok)
      {
        if(tok == "if") {inConds = true; continue;}
        if(!inConds)
        {
          (numArgs++ == 0 ? rule.a : rule.b) = static_cast<float>(std::atof(tok.c_str())) * 1000.f; // metres -> mm
          continue;
        }
        const size_t op = tok.find_first_of("<>");
        if(op == std::string::npos) {ok = false; break;}
        MdfzCond cond;
        cond.name = -1;
        for(int i = 0; i < 8; ++i)
          if(tok.compare(0, op, names[i]) == 0 && std::strlen(names[i]) == op)
            cond.name = i;
        if(cond.name < 0) {ok = false; break;}
        cond.less = tok[op] == '<';
        cond.value = static_cast<float>(std::atof(tok.c_str() + op + 1)) * (cond.name == 7 ? 1.f : 1000.f); // m -> mm, goals as is
        rule.conds.push_back(cond);
      }
      if(ok)
        mdfzRules.rules.push_back(rule);
    }
    std::printf("MDFZ_RULES_CONFIG team=%d rules=%d\n", Global::getSettings().teamNumber, static_cast<int>(mdfzRules.rules.size()));
    std::fflush(stdout);
  }
}

std::vector<ModuleBase::Info> WhiteFleetStrategyBehaviorControl::getExtModuleInfo()
{
  auto result = WhiteFleetStrategyBehaviorControl::getModuleInfo();
  BehaviorBase::addToModuleInfo(result);
  return result;
}

void WhiteFleetStrategyBehaviorControl::update(SkillRequest& skillRequest)
{
  WhiteFleetSimEntry::activate();
  static thread_local bool fleetProviderSeen=false;
  if(!fleetProviderSeen){WhiteFleetSimEntry::witness("WhiteFleetStrategyBehaviorControl");fleetProviderSeen=true;WhiteFleetSimEntry::tactical(theBehaviorParameters);}
  if(!theGameState.isPlaying() || theGameState.playerState!=GameState::active)
    cornerPlan=CornerPlan{};
  wasPenalized = theExtendedGameState.wasPenalized() && theExtendedGameState.returnFromGameControllerPenalty && (theGameState.isPlaying() || theGameState.isReady() || theGameState.isSet());
  if(wasPenalized)
    wasPenalized = theFrameInfo.getTimeSince(theGameState.timeWhenPlayerStateStarted) < theBehaviorParameters.noSkillRequestAfterUnpenalizedTime;

  auto* self = updateAgents();

  theBehavior.preProcess();

  if(theGameState.playerState != GameState::active || wasPenalized ||
     theGameState.isPenaltyShootout() || theGameState.isInitial() || theGameState.isFinished())
  {
    // Reset provided representations.
    theStrategyStatus.proposedTactic = Tactic::none;
    theStrategyStatus.acceptedTactic = Tactic::none;
    theStrategyStatus.proposedMirror = false;
    theStrategyStatus.acceptedMirror = false;
    theStrategyStatus.proposedSetPlay = SetPlay::none;
    theStrategyStatus.acceptedSetPlay = SetPlay::none;
    theStrategyStatus.setPlayStep = -1;
    theStrategyStatus.position = Tactic::Position::none;
    theStrategyStatus.role = Role::none;
    skillRequest = SkillRequest::Builder::stand();
    outletPlan=OutletPlan{};supportHalf.reset();
    if(theGameState.isInitial()||theGameState.isFinished())
    {kickoffThroughSession.reset();kickoffThroughEvent=0;isolatedKickoffPhase.reset();}
    else if(isolatedKickoffPhase.event)isolatedKickoffPhase.finish("player_inactive_or_reentry");
  }
  else
  {
    ASSERT(self);

    // C15 owns the two field players BEFORE the ordinary role/request update.
    if(runIsolatedKickoff(skillRequest,*self))
    {outletPlan=OutletPlan{};supportHalf.reset();theBehavior.postProcess();return;}

    skillRequest = theBehavior.update(strategy, *self, agents);
    // This playbook has one shared kicker, not two independent local cost
    // elections. Ordinary team roles resume immediately after ownKickOff.
    const bool autonomousKickoff=WhiteFleetAutonomousKickoff::enabled(WhiteFleetSimPhysical::physical(),
        WhiteFleetK1V21MatchProfile::guardedPhysicalReceiver(),WhiteFleetSimPhysical::simulated(),
        Global::getSettings().teamNumber,std::getenv("MDFZ_AUTONOMOUS_KICKOFF_V4_ENABLE"))
        &&theGameState.state==GameState::ownKickOff&&theGameState.isForOwnTeam();
    if(autonomousKickoff&&!theGameState.ownTeam.isGoalkeeper(self->number))
    {
      const bool kicker=WhiteFleetAutonomousKickoff::isKicker(self->number)&&
          theGameState.ownTeam.playerStates[WhiteFleetAutonomousKickoff::kickerNumber-Settings::lowestValidPlayerNumber]==GameState::active;
      self->role=kicker?ActiveRole::toRole(ActiveRole::playBall):Role::none;
      skillRequest=kicker?SkillRequest::Builder::shoot():SkillRequest::Builder::stand();
    }

    theStrategyStatus.proposedTactic = self->proposedTactic;
    theStrategyStatus.acceptedTactic = self->acceptedTactic;
    theStrategyStatus.proposedMirror = self->proposedMirror;
    theStrategyStatus.acceptedMirror = self->acceptedMirror;
    theStrategyStatus.proposedSetPlay = self->proposedSetPlay;
    theStrategyStatus.acceptedSetPlay = self->acceptedSetPlay;
    theStrategyStatus.setPlayStep = self->setPlayStep;
    theStrategyStatus.position = self->position;
    theStrategyStatus.role = self->role;
    if(WhiteFleetDefenseCounterSwitch::enabled()&&!autonomousKickoff)
      applyDefenseCounter(skillRequest,*self);
    if(WhiteFleetC15KickoffSwitch::enabled(theBehaviorParameters.matchTacticalRepair&&theBehaviorParameters.c15Kickoff)&&
       (theGameState.state==GameState::setupOwnKickOff||theGameState.state==GameState::waitForOwnKickOff))
      applyKickoffThrough(skillRequest,*self); // Legal READY preparation and SET hold only.
    if(!autonomousKickoff&&!skillRequest.kickoffThrough&&!skillRequest.restartPassRequired)
    {
      {const bool outlet=applyBackfieldOutlet(skillRequest,*self);if(outlet&&mdfzFormation.enabled)++mdfzFormation.n[7];if(!outlet&&!applyMdfzRules(skillRequest,*self)&&!applyMdfzFormation(skillRequest,*self))applyOpponentSidePosition(skillRequest,*self);}
    }
    else {outletPlan=OutletPlan{};supportHalf.reset();}
  }

  if(WhiteFleetC15KickoffSwitch::enabled(theBehaviorParameters.matchTacticalRepair&&theBehaviorParameters.c15Kickoff)&&
     isolatedKickoffPhase.finished)
  {
    skillRequest.isolatedKickoffFinished=true;
    skillRequest.isolatedKickoffEvent=isolatedKickoffPhase.event;
    if(self&&kickoffThroughSession.hasContract&&self->number==kickoffThroughSession.receiver&&
       theGameState.state==GameState::playing)
    {
      skillRequest.kickoffThroughRetired=true;
      skillRequest.kickoffThroughKicker=kickoffThroughSession.sender;
      skillRequest.kickoffThroughReceiver=kickoffThroughSession.receiver;
      skillRequest.kickoffThroughEvent=kickoffThroughSession.event;
      skillRequest.kickoffThroughTarget={kickoffThroughSession.committed.target.x,kickoffThroughSession.committed.target.y};
    }
  }

  theBehavior.postProcess();

  if(theBehaviorParameters.requireFreshTeamActivity && theFrameInfo.getTimeSince(lastTeamActivityDiagnostic) >= 2000)
  {
    lastTeamActivityDiagnostic = theFrameInfo.time;
    int owner = -1;
    for(const Agent& agent : agents)
      if(Role::isActiveRole(agent.role) && agent.role != ActiveRole::toRole(ActiveRole::closestToTeamBall))
        owner = agent.number;
    std::printf("MATCH_STRATEGY pid=%d goalkeeper=%d playing=%d own_setplay=%d agents=%zu owner=%d position=%s role=%s skill=%s ball_age_ms=%d team_ball_valid=%d pose_mm_rad=%.1f,%.1f,%.3f\n",
                theGameState.playerNumber, theGameState.ownTeam.isGoalkeeper(theGameState.playerNumber),
                theGameState.isPlaying(), theGameState.isForOwnTeam(), agents.size(), owner,
                TypeRegistry::getEnumName(theStrategyStatus.position), TypeRegistry::getEnumName(theStrategyStatus.role),
                TypeRegistry::getEnumName(skillRequest.skill), theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen),
                theTeamBallModel.isValid, theRobotPose.translation.x(), theRobotPose.translation.y(), static_cast<float>(theRobotPose.rotation));
  }
}

bool WhiteFleetStrategyBehaviorControl::allowKickoffReceiverContactPosture(const WhiteFleetKickoffThroughV8::Params& params,const Agent& self)
{
  namespace K=WhiteFleetKickoffThroughV8;
  // The existing explicit forward-prepare profile is the only opt-in. Native
  // MotionInfo::isMotionStable is a phase flag, NOT independent balance proof.
  if(!theBehaviorParameters.c15ForwardPrepare||params.firstPassPrepareAdvance!=250.f)return false;
  const unsigned now=theFrameInfo.time,event=isolatedKickoffPhase.event;
  if(receiverContactPosture.event!=event)receiverContactPosture={event,0,0,0,false};
  const bool receiver=self.number==(params.kicker==2?3:2);
  const bool warning=theFallDownState.state==FallDownState::staggering;
  const bool episode=receiver&&event>0&&event==kickoffThroughSession.event&&kickoffThroughSession.hasContract
      &&kickoffThroughSession.stage==K::Stage::flight&&kickoffThroughSession.nativePursuitAt>0;
  const unsigned changed=theFallDownState.timestampSinceStateSwitch;
  const bool knownWarningTime=changed>0&&changed<=now;
  if(episode)
  {
    if((theFallDownState.state!=FallDownState::upright&&!warning)||theGameState.stopped
       ||theGameState.playerState!=GameState::active||theRobotPose.quality==RobotPose::poor||!theRobotPose.isFinite()
       ||isolatedKickoffPhase.clockFault
       ||(warning&&(theMotionInfo.executedPhase!=MotionPhase::walk||!theMotionInfo.isMotionStable
          ||theFallDownState.direction!=FallDownState::none||!knownWarningTime)))receiverContactPosture.blocked=true;
    // Start on the first warning, even while far/stale, not only when a later
    // near-ball observation would make the continuation convenient.
    if(warning&&knownWarningTime&&!receiverContactPosture.since)receiverContactPosture.since=changed;
  }
  const bool candidate=episode&&warning&&knownWarningTime&&now-changed<=100u
      &&!receiverContactPosture.blocked&&theFallDownState.direction==FallDownState::none
      &&theGameState.state==GameState::playing&&theGameState.playerState==GameState::active&&!theGameState.stopped
      &&event>0&&event==kickoffThroughSession.event&&!isolatedKickoffPhase.finished&&!isolatedKickoffPhase.clockFault
      &&isolatedKickoffPhase.lastAction==static_cast<int>(K::Action::pursue)
      &&kickoffThroughSession.hasContract&&kickoffThroughSession.stage==K::Stage::flight
      &&kickoffThroughSession.nativePursuitAt>0
      &&theMotionInfo.executedPhase==MotionPhase::walk&&theMotionInfo.isMotionStable
      &&theRobotPose.quality!=RobotPose::poor&&theRobotPose.isFinite()
      &&theFieldDimensions.isInsideField(theRobotPose.translation)
      &&theBallModel.timeWhenLastSeen>0&&theBallModel.timeWhenLastSeen<=now&&now-theBallModel.timeWhenLastSeen<=250u
      &&theFieldBall.ballWasSeen(250)&&theFieldBall.positionRelative.allFinite()&&theFieldBall.positionOnField.allFinite()
      &&theFieldDimensions.isInsideField(theFieldBall.positionOnField)
      &&theFieldBall.positionRelative.norm()<=K::captureDistance
      &&K::receiverBallInFront({theFieldBall.positionRelative.x(),theFieldBall.positionRelative.y()});
  const bool allowed=candidate&&receiverContactPosture.since>=event&&receiverContactPosture.since<=now
      &&now-receiverContactPosture.since<=100u;
  // Actual consumed guard values, bounded16/event. No pose/ball state is changed.
  if(receiver&&warning&&event&&receiverContactPosture.lastLog!=now&&receiverContactPosture.logs<16u)
  {
    receiverContactPosture.lastLog=now;++receiverContactPosture.logs;
    std::printf("C15_RECEIVER_CONTACT_POSTURE event=%u now=%u allowed=%d since=%u fall=%u direction=%u motion=%u stable_flag=%d candidate=%d phase_action=%d stage=%u stopped=%d ball_seen=%u range=%.1f changed=%u blocked=%d\n",
        event,now,allowed,receiverContactPosture.since,static_cast<unsigned>(theFallDownState.state),
        static_cast<unsigned>(theFallDownState.direction),static_cast<unsigned>(theMotionInfo.executedPhase),
        theMotionInfo.isMotionStable,candidate,isolatedKickoffPhase.lastAction,
        static_cast<unsigned>(kickoffThroughSession.stage),theGameState.stopped,theBallModel.timeWhenLastSeen,
        theFieldBall.positionRelative.norm(),changed,receiverContactPosture.blocked);
  }
  return allowed;
}

bool WhiteFleetStrategyBehaviorControl::runIsolatedKickoff(SkillRequest& request,Agent& self)
{
  // READ_ONLY_C15_ENTRY_BEGIN
  if(theGameState.state==GameState::ownKickOff && (self.number==2||self.number==3))
  {
    static thread_local unsigned c15EntryEvent=0;
    if(c15EntryEvent!=theGameState.timeWhenStateStarted)
    {c15EntryEvent=theGameState.timeWhenStateStarted;std::printf("FLEET_C15_ENTRY team=%d self=%d event=%u now=%u enabled=%d\n",Global::getSettings().teamNumber,self.number,c15EntryEvent,theFrameInfo.time,WhiteFleetC15KickoffSwitch::enabled(theBehaviorParameters.matchTacticalRepair&&theBehaviorParameters.c15Kickoff));std::fflush(stdout);}
  }
  // READ_ONLY_C15_ENTRY_END

  if(WhiteFleetAutonomousKickoff::enabled(WhiteFleetSimPhysical::physical(),
      WhiteFleetK1V21MatchProfile::guardedPhysicalReceiver(),WhiteFleetSimPhysical::simulated(),
      Global::getSettings().teamNumber,std::getenv("MDFZ_AUTONOMOUS_KICKOFF_V4_ENABLE")))
  {
    // Single physical path: native owner -> native first-pass skill. A failed
    // C15 transaction must not quarantine this kickoff or later ordinary PLAY.
    kickoffThroughSession.reset();kickoffThroughEvent=0;isolatedKickoffPhase.reset();
    return false;
  }
  if(!WhiteFleetC15KickoffSwitch::enabled(theBehaviorParameters.matchTacticalRepair&&theBehaviorParameters.c15Kickoff))return false;
  const auto p=WhiteFleetK1V21MatchProfile::guardedPrepare(WhiteFleetC15KickoffSwitch::configuredForwardPrepareParams(theBehaviorParameters.c15EarlyStart,theBehaviorParameters.c15RestoreProvenPool,theBehaviorParameters.c15DefenceAdaptiveAngle,theBehaviorParameters.c15InsideCornerLane,theBehaviorParameters.c15RaceRendezvous,theBehaviorParameters.c15DirectCenterKickoff,theBehaviorParameters.c15CenterFastKickoff,theBehaviorParameters.c15ReceiverPrepareHold,theBehaviorParameters.c15ReceiverBallApproach,theBehaviorParameters.c15ReceiverFollowFromRelease,theBehaviorParameters.c15StableKickAim,theBehaviorParameters.c15ReceiverEarlyNative,theBehaviorParameters.c15ReceiverGapWalk,theBehaviorParameters.c15FirstPassClearance,theBehaviorParameters.c15ForwardPrepare),theFieldDimensions.xPosHalfwayLine,theFieldDimensions.fieldLinesWidth,theBehaviorParameters.illegalAreasLargeDistance);
  const int receiver=p.kicker==2?3:2;
  if(self.number!=p.kicker&&self.number!=receiver)return false; // GK remains native.
  const bool own=theGameState.state==GameState::ownKickOff;
  const bool playing=theGameState.state==GameState::playing;
  const bool owns=isolatedKickoffPhase.tick(own,playing,theGameState.timeWhenStateStarted,theFrameInfo.time,p.total);
  if(!isolatedKickoffPhase.event)return false;
  const bool receiverContact=allowKickoffReceiverContactPosture(p,self);
  const bool simMaxReleased=theBehaviorParameters.c15ReceiverSimMaxRun&&WhiteFleetSimPhysical::simulated()
      &&Global::getSettings().teamNumber==5&&Global::getSettings().playerNumber==2&&self.number==receiver
      &&kickoffThroughSession.nativePursuitAt>0&&kickoffThroughSession.stage!=WhiteFleetKickoffThroughV8::Stage::ended;
  const bool unsafe=theGameState.stopped||(theRobotPose.quality==RobotPose::poor&&!simMaxReleased)||!theRobotPose.isFinite()
      ||(theFallDownState.state!=FallDownState::upright&&!receiverContact)||isolatedKickoffPhase.clockFault
      ||(theMotionInfo.executedPhase!=MotionPhase::stand&&theMotionInfo.executedPhase!=MotionPhase::walk);
  isolatedKickoffPhase.observeDeparture(theFrameInfo.time,theBallModel.timeWhenLastSeen,
      !unsafe&&theFieldBall.ballWasSeen(200)&&theFieldBall.positionOnField.allFinite()&&
      theFieldDimensions.isInsideField(theFieldBall.positionOnField),
      theFieldBall.positionOnField.norm()>500.f);
  const auto refuseFirstFoot=[&]()
  {
    self.role=Role::none;theStrategyStatus.role=Role::none;
    request=SkillRequest::Builder::stand();request.kickoffThrough=true;
    request.kickoffThroughAction=1;request.kickoffThroughEvent=isolatedKickoffPhase.event;
    request.kickoffThroughKicker=p.kicker;request.kickoffThroughReceiver=receiver;
    if(!isolatedKickoffPhase.refusalReported)
    {
      std::printf("C15_FAILED_START_HOLD self=%d event=%u now=%u reason=%s ordinary_update_skipped=1 ball_departed=0\n",
          self.number,isolatedKickoffPhase.event,theFrameInfo.time,isolatedKickoffPhase.reason);
      isolatedKickoffPhase.refusalReported=true;
    }
    // No contract publication, retry, ordinary owner or successful-kick claim.
    return true;
  };
  if(unsafe && (owns||own||(isolatedKickoffPhase.finished&&!isolatedKickoffPhase.ballDeparted)))
  {
    request=SkillRequest::Builder::stand();request.kickoffThrough=true;
    request.kickoffThroughAction=1;request.kickoffThroughEvent=isolatedKickoffPhase.event;
    request.kickoffThroughKicker=p.kicker;request.kickoffThroughReceiver=receiver;
    return true; // Dedicated hold also prevents stale ordinary ReceivePass.
  }
  if(!owns)
  {
    if(isolatedKickoffPhase.finished)
    {
      for(auto& agent:agents)
        if(SetPlay::isKickOff(agent.proposedSetPlay))
        {agent.proposedSetPlay=agent.acceptedSetPlay=SetPlay::none;agent.setPlayStep=-1;}
      if(!isolatedKickoffPhase.exitReported)
      {
        std::printf("C10_PHASE_EXIT self=%d event=%u now=%u elapsed_ms=%u reason=%s completed_proxy=0\n",
            self.number,isolatedKickoffPhase.event,theFrameInfo.time,theFrameInfo.time-isolatedKickoffPhase.event,isolatedKickoffPhase.reason);
        std::fflush(stdout); // Read-only first-episode stop observation, no motion/GC change.
        isolatedKickoffPhase.exitReported=true;
      }
    }
    // The phase is finished, but a stationary unplayed centre ball must not
    // silently select ordinary pass/clear. A referee context change resets it;
    // trusted actual departure allows normal play, with no stale C15 restart.
    if(isolatedKickoffPhase.finished&&!isolatedKickoffPhase.ballDeparted)return refuseFirstFoot();
    return false;
  }
  // Publish only the phase's explicit roles; clear stale ordinary set plays.
  self.proposedSetPlay=self.acceptedSetPlay=SetPlay::none;self.setPlayStep=-1;
  self.role=self.number==p.kicker&&!kickoffThroughSession.kickedAt?
      ActiveRole::toRole(ActiveRole::playBall):Role::none;
  theStrategyStatus.proposedSetPlay=theStrategyStatus.acceptedSetPlay=SetPlay::none;
  theStrategyStatus.setPlayStep=-1;theStrategyStatus.role=self.role;
  cornerPlan=CornerPlan{}; // No generic kickoff planner can run during C15.
  request=SkillRequest::Builder::stand();
  const bool handled=applyKickoffThrough(request,self);
  // A transient missing roster/peer at the first legal PLAY frame has not
  // attempted a contract or a kick. Keep the SAME event and original prepare
  // deadline; never turn zero elapsed time into a permanent failed episode.
  if(WhiteFleetK1V21MatchProfile::guardedPhysicalReceiver() && !kickoffThroughSession.event
     && theFrameInfo.time>=isolatedKickoffPhase.event
     && theFrameInfo.time-isolatedKickoffPhase.event<=p.maxPrepare)
  {
    self.role=Role::none;theStrategyStatus.role=Role::none;
    request=SkillRequest::Builder::stand();request.kickoffThrough=true;
    request.kickoffThroughAction=1;request.kickoffThroughEvent=isolatedKickoffPhase.event;
    request.kickoffThroughKicker=p.kicker;request.kickoffThroughReceiver=receiver;
    if(isolatedKickoffPhase.lastAction!=1)
      std::printf("C15_START_WAIT self=%d event=%u now=%u elapsed_ms=%u original_prepare_budget_ms=%u no_contract=1 no_kick=1\n",
          self.number,isolatedKickoffPhase.event,theFrameInfo.time,
          theFrameInfo.time-isolatedKickoffPhase.event,p.maxPrepare);
    isolatedKickoffPhase.lastAction=1;
    return true;
  }
  if(kickoffThroughSession.stage==WhiteFleetKickoffThroughV8::Stage::ended||!kickoffThroughSession.event)
  {
    isolatedKickoffPhase.finish(kickoffThroughSession.event?kickoffThroughSession.reason:"pair_unavailable_before_session");
    for(auto& agent:agents)
      if(SetPlay::isKickOff(agent.proposedSetPlay))
      {agent.proposedSetPlay=agent.acceptedSetPlay=SetPlay::none;agent.setPlayStep=-1;}
    self.role=Role::none;theStrategyStatus.role=Role::none;
    std::printf("C10_PHASE_EXIT self=%d event=%u now=%u elapsed_ms=%u reason=%s completed_proxy=%d\n",
        self.number,isolatedKickoffPhase.event,theFrameInfo.time,theFrameInfo.time-isolatedKickoffPhase.event,
        isolatedKickoffPhase.reason,kickoffThroughSession.settleAt!=0);
    std::fflush(stdout); // Read-only first-episode stop observation, no motion/GC change.
    isolatedKickoffPhase.exitReported=true;
    return !isolatedKickoffPhase.ballDeparted?refuseFirstFoot():false;
  }
  if(self.number==receiver&&(request.kickoffThroughAction==static_cast<int>(WhiteFleetKickoffThroughV8::Action::pursue)
      ||kickoffThroughSession.stage==WhiteFleetKickoffThroughV8::Stage::capture
      ||kickoffThroughSession.stage==WhiteFleetKickoffThroughV8::Stage::settling))
    self.role=ActiveRole::toRole(ActiveRole::playBall);
  theStrategyStatus.role=self.role;
  if(!handled)request=SkillRequest::Builder::stand();
  if(isolatedKickoffPhase.lastAction!=request.kickoffThroughAction)
  {
    std::printf("C10_PHASE_CONTROL self=%d event=%u now=%u action=%d ordinary_update_skipped=1 role=%d\n",
        self.number,isolatedKickoffPhase.event,theFrameInfo.time,request.kickoffThroughAction,static_cast<int>(self.role));
    isolatedKickoffPhase.lastAction=request.kickoffThroughAction;
  }
  return true;
}

bool WhiteFleetStrategyBehaviorControl::applyKickoffThrough(SkillRequest& request,const Agent& self)
{
  namespace Through=WhiteFleetKickoffThroughV8;
  if(!WhiteFleetC15KickoffSwitch::enabled(theBehaviorParameters.matchTacticalRepair&&theBehaviorParameters.c15Kickoff))return false;
  const auto params=WhiteFleetK1V21MatchProfile::guardedPrepare(WhiteFleetC15KickoffSwitch::configuredForwardPrepareParams(theBehaviorParameters.c15EarlyStart,theBehaviorParameters.c15RestoreProvenPool,theBehaviorParameters.c15DefenceAdaptiveAngle,theBehaviorParameters.c15InsideCornerLane,theBehaviorParameters.c15RaceRendezvous,theBehaviorParameters.c15DirectCenterKickoff,theBehaviorParameters.c15CenterFastKickoff,theBehaviorParameters.c15ReceiverPrepareHold,theBehaviorParameters.c15ReceiverBallApproach,theBehaviorParameters.c15ReceiverFollowFromRelease,theBehaviorParameters.c15StableKickAim,theBehaviorParameters.c15ReceiverEarlyNative,theBehaviorParameters.c15ReceiverGapWalk,theBehaviorParameters.c15FirstPassClearance,theBehaviorParameters.c15ForwardPrepare),theFieldDimensions.xPosHalfwayLine,theFieldDimensions.fieldLinesWidth,theBehaviorParameters.illegalAreasLargeDistance);
  if(params.isolated&&isolatedKickoffPhase.finished)return false;
  const int receiver=params.kicker==2?3:2;
  if(self.number!=params.kicker && self.number!=receiver)return false;
  const auto active=[&](int number)
  {
    const int i=number-Settings::lowestValidPlayerNumber;
    return i>=0 && i<static_cast<int>(theGameState.ownTeam.playerStates.size())
        && !theGameState.ownTeam.isGoalkeeper(number)
        && theGameState.ownTeam.playerStates[i]==GameState::active;
  };
  unsigned activeFields=0;
  for(unsigned i=0;i<theGameState.ownTeam.playerStates.size();++i)
    if(active(Settings::lowestValidPlayerNumber+static_cast<int>(i)))++activeFields;
  const bool pairActive=active(params.kicker)&&active(receiver)&&activeFields==2;
  const Through::Field field{theFieldDimensions.xPosOwnGoalLine,theFieldDimensions.xPosOpponentGoalLine,
    theFieldDimensions.xPosOpponentPenaltyArea,theFieldDimensions.yPosLeftTouchline,theFieldDimensions.yPosLeftGoal,theFieldDimensions.yPosLeftPenaltyArea};
  const bool receiverContact=allowKickoffReceiverContactPosture(params,self);
  const bool upright=(theFallDownState.state==FallDownState::upright||receiverContact)
      && (theMotionInfo.executedPhase==MotionPhase::stand || theMotionInfo.executedPhase==MotionPhase::walk);
  const bool simMaxReleased=theBehaviorParameters.c15ReceiverSimMaxRun&&WhiteFleetSimPhysical::simulated()
      &&Global::getSettings().teamNumber==5&&Global::getSettings().playerNumber==2&&self.number==receiver
      &&kickoffThroughSession.nativePursuitAt>0&&kickoffThroughSession.stage!=Through::Stage::ended;
  const bool poseValid=(theRobotPose.quality!=RobotPose::poor||simMaxReleased) && theRobotPose.isFinite();
  const auto& oldContract=kickoffThroughSession.committedPlan();
  const bool fixedRunProfile=theBehaviorParameters.c15ReceiverFixedRunToArrival&&theBehaviorParameters.c15ReceiverFixedDirect
      &&theBehaviorParameters.c15EarlyChannelReceive&&theBehaviorParameters.c15ReceiverLocalStep
      &&!theBehaviorParameters.c15ReceiverLowerChannel&&!theBehaviorParameters.c15ReceiverShortestIntercept
      &&params.firstPassPrepareAdvance==250.f&&self.number==receiver;
  const bool fixedReleasedSession=fixedRunProfile&&theGameState.state==GameState::playing&&kickoffThroughSession.hasContract
      &&kickoffThroughSession.nativePursuitAt>0&&kickoffThroughSession.stage!=Through::Stage::ended
      &&!isolatedKickoffPhase.finished&&kickoffThroughSession.event==kickoffThroughEvent;
  const bool ownRollingIntercept=(params.raceRendezvous||params.directCenter)&&self.number==receiver&&kickoffThroughSession.hasContract&&
      (theGameState.state==GameState::playing||theGameState.state==GameState::ownKickOff)&&theFieldBall.ballWasSeen(250)&&
      theBallModel.timeWhenLastSeen>0&&theBallModel.timeWhenLastSeen<=theFrameInfo.time&&
      theFrameInfo.time-theBallModel.timeWhenLastSeen<=250&&theFieldBall.positionOnField.allFinite()&&
      theFieldBall.positionRelative.allFinite()&&
      WhiteFleetC15RollingRendezvous::approachingOwnContract({theFieldBall.positionOnField.x(),theFieldBall.positionOnField.y()},
          kickoffThroughSession.launch,oldContract.target)&&
      (params.directCenter?Through::directCenterDomain(oldContract.target,field,params,Through::contractTolerance):
       WhiteFleetC15RollingRendezvous::endpoint(oldContract.target,field,params,Through::contractTolerance));
  const bool intercept=(theInterceptBallInformation.walkOutOfBallPath&&!simMaxReleased)||
      ((theInterceptBallInformation.intercept||theFieldInterceptBall.interceptBall||theFieldInterceptBall.predictedInterceptBall)&&!ownRollingIntercept&&!fixedReleasedSession);
  // Legal READY staging only, never pretend a receiver has already reached it.
  // This changes the two local kickoff requests, not team role/owner elections.
  if(theGameState.state==GameState::setupOwnKickOff || theGameState.state==GameState::waitForOwnKickOff)
  {
    kickoffThroughSession.reset();kickoffThroughEvent=0;
    if(!pairActive || !upright || !poseValid || intercept || theGameState.stopped)return false;
    int side=params.side;
    if(!side)
    {
      side=self.number==receiver?(theRobotPose.translation.y()<0.f?-1:1):(theStrategyStatus.acceptedMirror?-1:1);
      for(const auto& teammate:theTeamData.teammates)
        if(teammate.number==receiver && teammate.theRobotPose.isFinite() && teammate.theRobotStatus.isUpright)
          side=teammate.theRobotPose.translation.y()<0.f?-1:1;
    }
    const auto prep=Through::preparePoint(field,params,side);
    const auto raceTargets=WhiteFleetC15RollingRendezvous::targets(field,params);
    const bool raceReady=params.raceRendezvous&&!raceTargets.empty();
    const Through::Point readyTarget=params.directCenter?Through::directCenterTarget(field,params):
        raceReady?raceTargets.front():Through::Point{field.opponentPenalty-750.f,side*field.halfWidth*params.targetYFraction};
    const float centerAngle=std::atan2(readyTarget.y,readyTarget.x);
    const Vector2f point=self.number==params.kicker?
        (params.centerFast?Vector2f(-500.f*std::cos(centerAngle),-500.f*std::sin(centerAngle)):
         raceReady?Vector2f(-300.f,450.f):Vector2f(-500.f,0.f)):Vector2f(prep.x,prep.y);
    const float readyHeading=params.playbook&&self.number==receiver?
        WhiteFleetC9ForwardFlank::runHeading({point.x(),point.y()},readyTarget,side,params):
        raceReady?std::atan2(readyTarget.y,readyTarget.x):(Vector2f::Zero()-point).angle();
    request=theGameState.isSet()?SkillRequest::Builder::stand():
      SkillRequest::Builder::walkTo(Pose2f(readyHeading,point));
    request.kickoffThrough=true;
    request.kickoffThroughAction=theGameState.isSet()?1:2;
    request.kickoffThroughKicker=params.kicker;request.kickoffThroughReceiver=receiver;
    request.kickoffThroughEvent=theGameState.timeWhenStateStarted;
    request.kickoffThroughTarget=(raceReady||params.directCenter)?Vector2f(readyTarget.x,readyTarget.y):
      Vector2f(field.opponentGoal*params.targetXFraction,side*field.halfWidth*params.targetYFraction);
    request.kickoffThroughBodyTarget=point;
    request.kickoffThroughBallKnown=theFieldBall.ballWasSeen(650) && theFieldBall.positionOnField.allFinite();
    request.kickoffThroughBall=theFieldBall.positionOnField;
    return true;
  }
  const bool ownKickoff=theGameState.state==GameState::ownKickOff;
  const bool playing=theGameState.state==GameState::playing;
  if(ownKickoff && kickoffThroughEvent!=theGameState.timeWhenStateStarted)
    kickoffThroughEvent=theGameState.timeWhenStateStarted;
  if(!ownKickoff && !playing)
  {kickoffThroughSession.reset();kickoffThroughEvent=0;return false;}
  if(!kickoffThroughEvent)return false;

  Through::Input input;
  input.timeDiagnostic=false;
  input.c9Coherence=true;
  input.params=params;input.field=field;input.now=theFrameInfo.time;input.event=kickoffThroughEvent;
  input.self=self.number;input.receiver=receiver;input.enabled=true;input.ownKickoff=ownKickoff;input.playing=playing;
  input.rosterValid=pairActive;input.active=theGameState.playerState==GameState::active;
  input.upright=upright;input.poseValid=poseValid;input.stopped=theGameState.stopped;input.intercept=intercept;
  input.peerTimeout=theBehaviorParameters.teamActivityTimeout;
  input.position={theRobotPose.translation.x(),theRobotPose.translation.y()};
  if(params.stableKickAim)
  {input.fieldHeading=theRobotPose.rotation;input.fieldHeadingVariance=theRobotPose.covariance(2,2);}
  input.ball={theFieldBall.positionOnField.x(),theFieldBall.positionOnField.y()};
  input.ballRelative={theFieldBall.positionRelative.x(),theFieldBall.positionRelative.y()};
  input.localSeen=theBallModel.timeWhenLastSeen;input.localKick=theMotionInfo.lastKickTimestamp;
  input.fixedRunToArrival=fixedRunProfile;
  if(fixedReleasedSession&&(theMotionRequest.motion==MotionRequest::walkToBallAndKick||theMotionRequest.motion==MotionRequest::dribble)
     &&theMotionRequest.ballEstimateTimestamp>=kickoffThroughSession.nativePursuitAt
     &&theMotionRequest.ballEstimateTimestamp<=input.now&&input.now-theMotionRequest.ballEstimateTimestamp<=250u
     &&theMotionRequest.ballTimeWhenLastSeen>0&&theMotionRequest.ballTimeWhenLastSeen<=theMotionRequest.ballEstimateTimestamp
     &&theMotionRequest.ballEstimateTimestamp-theMotionRequest.ballTimeWhenLastSeen<=250u
     &&theMotionRequest.ballEstimate.position.allFinite()&&theMotionRequest.ballEstimate.position.norm()<=1000.f)
    input.fixedFootCommandAt=theMotionRequest.ballEstimateTimestamp;
  if(params.playbook)
  {
    input.odometry={theOdometryData.isFinite(),{theOdometryData.translation.x(),theOdometryData.translation.y()},theOdometryData.rotation};
    input.odometrySpeedBound=theWalkingEngineOutput.maxSpeed.translation.norm();
    input.odometryYawBound=theWalkingEngineOutput.maxSpeed.rotation;
  }
  input.nativeOwner=Role::isActiveRole(self.role) && self.role!=ActiveRole::toRole(ActiveRole::closestToTeamBall);
  if(params.isolated)input.nativeOwner=self.number==params.kicker;
  input.ballInsidePitch=Through::finite(input.ball) && input.ball.x>=field.ownGoal && input.ball.x<=field.opponentGoal
      && std::abs(input.ball.y)<=field.halfWidth;
  input.trackingReady=input.localSeen>0 && input.localSeen<=input.now && input.now-input.localSeen<=650
      && theFieldBall.ballWasSeen(650) && Through::finite(input.ballRelative);
  const auto peerFrom=[&](int number,const RobotPose& pose,const BehaviorStatus& behavior,
                          const FrameInfo& frame,const RobotStatus& status,const BallModel& ball,
                          const IndirectKick& indirect,const StrategyStatus& strategy)
  {
    Through::Peer result;result.present=true;result.number=number;result.stamp=frame.time;
    result.healthy=active(number)&&status.isUpright&&pose.isFinite()&&pose.quality!=RobotPose::poor;
    result.position={pose.translation.x(),pose.translation.y()};result.passTarget=behavior.passTarget;
    result.heading=pose.rotation;
    result.hasTarget=behavior.shootingTo.has_value() && behavior.shootingTo->allFinite();
    if(result.hasTarget){const Vector2f p=pose**behavior.shootingTo;result.target={p.x(),p.y()};}
    const Vector2f walking=pose*behavior.walkingTo;result.walkingTo={walking.x(),walking.y()};
    if(!Through::finite(result.walkingTo) || (result.hasTarget&&!Through::finite(result.target)))result.healthy=false;
    result.speed=behavior.speed;result.ballSeen=ball.timeWhenLastSeen;
    const Vector2f seen=pose*ball.estimate.position;result.ball={seen.x(),seen.y()};
    result.kickProxy=indirect.lastKickTimestamp;
    result.nativeOwner=Role::isActiveRole(strategy.role) && strategy.role!=ActiveRole::toRole(ActiveRole::closestToTeamBall);
    return result;
  };
  const int other=self.number==params.kicker?receiver:params.kicker;
  for(const auto& teammate:theTeamData.teammates)
    if(teammate.number==other)
      input.peer=peerFrom(other,teammate.theRobotPose,teammate.theBehaviorStatus,teammate.theFrameInfo,
          teammate.theRobotStatus,teammate.theBallModel,teammate.theIndirectKick,teammate.theStrategyStatus);
  input.sent=peerFrom(self.number,theSentTeamMessage.theRobotPose,theSentTeamMessage.theBehaviorStatus,
      theSentTeamMessage.theFrameInfo,theSentTeamMessage.theRobotStatus,theSentTeamMessage.theBallModel,
      theSentTeamMessage.theIndirectKick,theSentTeamMessage.theStrategyStatus);
  // Keep the raw packet for Session's bounded clock check, but never plan or
  // point the head using a future-dated pose/ball. A retained last-good packet
  // keeps its real timestamp and native liveness limit; it is not made fresh.
  const auto& evidencePeer=input.peer.present&&input.peer.stamp<=input.now?
      input.peer:kickoffThroughSession.latestPeer;
  const bool evidencePeerValid=evidencePeer.present&&evidencePeer.number==other&&evidencePeer.healthy
      &&evidencePeer.stamp>0&&evidencePeer.stamp<=input.now
      &&input.now-evidencePeer.stamp<=std::min(input.peerTimeout,10000u)
      &&Through::finite(evidencePeer.position);

  std::vector<Through::Obstacle> obstacles;
  for(const auto& obstacle:theObstacleModel.obstacles)
  {
    const Vector2f center=theRobotPose*obstacle.center,left=theRobotPose*obstacle.left,right=theRobotPose*obstacle.right;
    obstacles.push_back({{center.x(),center.y()},{left.x(),left.y()},{right.x(),right.y()},
      !obstacle.isTeammate(),obstacle.isTeammate(),2200.f,0.f});
  }
  for(const auto& opponent:theGlobalOpponentsModel.opponents)
    obstacles.push_back({{opponent.position.x(),opponent.position.y()},
      {opponent.position.x(),opponent.position.y()},{opponent.position.x(),opponent.position.y()},true,false,2200.f,300.f});
  unsigned freshOpponents=0,coherentOpponents=0;
  if(self.number==params.kicker&&evidencePeerValid&&input.trackingReady)
  {
    const KickLengthPair ordinaryKickPair;
    const auto rating=[&](Through::Point p){return WhiteFleetC9PassPlaybook::Ratings{
      thePassEvaluation.getRating({input.ball.x,input.ball.y},{p.x,p.y},false),
      theExpectedGoals.getRating({p.x,p.y},true,true,false)};};
    std::vector<Through::Point> motionPoints;
    std::vector<std::pair<std::size_t,std::size_t>> motionPairs;
    for(std::size_t localIndex=0;localIndex<theObstacleModel.obstacles.size();++localIndex)
    {
      const auto& o=theObstacleModel.obstacles[localIndex];
      if(!o.isOpponent()||!o.lastSeen||o.lastSeen>input.now||input.now-o.lastSeen>650||
         !o.center.allFinite()||!o.left.allFinite()||!o.right.allFinite())continue;
      ++freshOpponents;
      const Vector2f center=theRobotPose*o.center;
      // Association uses the same observed outline + 300mm global radius as
      // the unchanged planner. Never invent another opponent or treat unseen
      // players as free space. ALL observed obstacles still reach the planner.
      const float radius=300.f+std::max((o.center-o.left).norm(),(o.center-o.right).norm());
      if(!center.allFinite()||!std::isfinite(radius))continue;
      for(std::size_t globalIndex=0;globalIndex<theGlobalOpponentsModel.opponents.size();++globalIndex)
      {
        const auto& global=theGlobalOpponentsModel.opponents[globalIndex];
        if(!global.unidentified&&global.position.allFinite()&&(global.position-center).norm()<=radius)
        {
          ++coherentOpponents;
          obstacles[localIndex].angleSource=true;
          obstacles[theObstacleModel.obstacles.size()+globalIndex].angleSource=true;
          if(params.raceRendezvous)
          {
            motionPairs.emplace_back(localIndex,globalIndex);
            const Through::Point q{global.position.x(),global.position.y()};
            bool duplicate=false;for(auto p:motionPoints)duplicate|=Through::distance(p,q)<=1.f;
            if(!duplicate)motionPoints.push_back(q);
          }
          break;
        }
      }
    }
    if(params.raceRendezvous)
    {
      c15RaceMotion.observe(input.now,input.event,motionPoints);
      for(const auto& pair:motionPairs)
      {
        const auto& global=theGlobalOpponentsModel.opponents[pair.second];
        const float speed=c15RaceMotion.speed(input.now,{global.position.x(),global.position.y()});
        obstacles[pair.first].calibratedMotionSpeed=speed;
        obstacles[theObstacleModel.obstacles.size()+pair.second].calibratedMotionSpeed=speed;
      }
    }
    const auto& committed=kickoffThroughSession.committedPlan();
    if(!coherentOpponents)
      input.plan.reason="c15_wait_fresh_coherent_opponent";
    else if(params.raceRendezvous&&(input.now-evidencePeer.stamp>600||evidencePeer.ballSeen>input.now||
            !Through::sessionFresh(input.now,evidencePeer.ballSeen,250)))
      input.plan.reason="race_wait_fresh_ready_receiver";
    else if(Through::validPlan(committed))
    {
      const auto flight=Through::mappedFlight(input.ball,committed.target,committed.nominalLength,
        theKickInfo,ordinaryKickPair,theBallSpecification,params);
      input.plan=WhiteFleetC10KickoffPlanner::evaluate(input.ball,evidencePeer.position,evidencePeer.heading,
        committed.target,committed.side,field,obstacles,params,flight,theBallSpecification,
        theWalkingEngineOutput,rating);
    }
    else
    {
      // Default OFF; at most four actual enumerations per event, >=2s apart.
      // Sample only this real coherent/fresh planning call, never a fake peer.
      const bool sample=theBehaviorParameters.c15CandidateDiagnostics&&
        (c15CandidateAuditLatch.event!=input.event||(!c15CandidateAuditLatch.batches||
          (c15CandidateAuditLatch.batches<4&&input.now>=c15CandidateAuditLatch.last&&input.now-c15CandidateAuditLatch.last>=2000)))&&
        c15CandidateAuditLatch.select(input.now,input.event,evidencePeer.stamp);
      Through::CandidateAudit audit;
      input.plan=WhiteFleetC10KickoffPlanner::choose(input.ball,evidencePeer.position,evidencePeer.heading,
        field,obstacles,params,theKickInfo,ordinaryKickPair,theBallSpecification,theWalkingEngineOutput,rating,sample?&audit:nullptr);
      if(sample)
      {
        Through::printCandidateAudit(audit,Global::getSettings().teamNumber,self.number,input.now,input.event,
          evidencePeer.position,evidencePeer.heading,params,theWalkingEngineOutput.maxSpeed.translation.x(),
          theWalkingEngineOutput.maxSpeed.translation.y(),theWalkingEngineOutput.maxSpeed.rotation);
        Through::printRolloutAudit(audit,self.number,input.now,input.event,input.ball,field,params,-1);
      }
    }
  }
  input.planSafe=Through::validPlan(input.plan);input.receiverReachable=input.planSafe;
  const auto directive=kickoffThroughSession.update(input);
  if(directive.suppressLegacyReceive)
  {
    request.kickoffThroughRetired=true;
    request.kickoffThroughKicker=params.kicker;request.kickoffThroughReceiver=receiver;
    request.kickoffThroughTarget={directive.plan.target.x,directive.plan.target.y};
    request.kickoffThroughEvent=kickoffThroughEvent;
  }
  if(!directive.active&&!ownKickoff)return false;
  // Even cancellation in ownKickOff must not fall into the old sideways pass.
  request=SkillRequest::Builder::stand();request.kickoffThrough=true;
  request.kickoffThroughAction=directive.active?static_cast<int>(directive.action):1;
  request.kickoffThroughKicker=params.kicker;request.kickoffThroughReceiver=receiver;
  request.kickoffThroughEvent=kickoffThroughEvent;
  request.kickoffThroughTarget={directive.plan.target.x,directive.plan.target.y};
  request.kickoffThroughHasLaunch=kickoffThroughSession.hasContract;
  request.kickoffThroughLaunch={kickoffThroughSession.launch.x,kickoffThroughSession.launch.y};
  request.kickoffThroughAimValid=directive.aimValid;
  request.kickoffThroughAimTargetOdometry={directive.aimTargetOdometry.x,directive.aimTargetOdometry.y};
  request.kickoffThroughAimFrame=directive.aimFrame;
  if(receiverContact&&directive.active&&directive.action==Through::Action::pursue)
  {
    request.kickoffThroughPostureFrame=input.now;
    request.kickoffThroughPostureSince=receiverContactPosture.since;
  }
  if(directive.action==Through::Action::gapWalk&&directive.gapValid&&directive.gapFrame==input.now)
  {
    request.kickoffThroughGapValid=true;request.kickoffThroughGapSeen=directive.gapSeen;
    request.kickoffThroughGapFrame=directive.gapFrame;
    request.kickoffThroughGapBallOdometry={directive.gapBallOdometry.x,directive.gapBallOdometry.y};
    request.kickoffThroughGapTargetOdometry={directive.gapTargetOdometry.x,directive.gapTargetOdometry.y};
  }
  const auto body=directive.hasBodyTarget?directive.bodyTarget:
      directive.action==Through::Action::prepare?directive.plan.prepare:directive.plan.target;
  request.kickoffThroughBodyTarget={body.x,body.y};
  if(directive.action==Through::Action::support)
    request.kickoffThroughBodyTarget={std::max(field.ownGoal+1000.f,-1200.f),-directive.plan.side*1500.f};
  request.kickoffThroughBallKnown=input.trackingReady;
  request.kickoffThroughBall={input.ball.x,input.ball.y};
  if(!input.trackingReady && evidencePeerValid && evidencePeer.ballSeen>0 && evidencePeer.ballSeen<=input.now
      && input.now-evidencePeer.ballSeen<=650 && Through::finite(evidencePeer.ball))
  {request.kickoffThroughBallKnown=true;request.kickoffThroughBall={evidencePeer.ball.x,evidencePeer.ball.y};}
  request.kickoffThroughLength=directive.plan.nominalLength;
  request.kickoffThroughPublish=directive.publishOffer||directive.publishEcho;
  if(fixedRunProfile&&directive.active&&directive.action==Through::Action::pursue
     &&kickoffThroughSession.stage!=Through::Stage::ended&&kickoffThroughSession.hasContract
     &&kickoffThroughSession.nativePursuitAt>input.event&&kickoffThroughSession.nativePursuitAt<=input.now)
  {
    request.kickoffFixedRunFrame=input.now;
    request.kickoffFixedRunRelease=kickoffThroughSession.nativePursuitAt;
  }

  if(kickoffThroughLastAction!=request.kickoffThroughAction||theFrameInfo.getTimeSince(kickoffThroughLastLog)>=1000)
  {
    kickoffThroughLastLog=input.now;kickoffThroughLastAction=request.kickoffThroughAction;
    std::printf("FLEET_C15 event=%u self=%d action=%d reason=%s plan=%s P=%.1f,%.1f length=%.1f risk=%.3f capture_risk=%.3f peer_stamp=%u observed_opponents=%u coherent_opponents=%u global_opponents=%zu unknown_opponents=%d restored_pool=%d target_pool=%s adaptive_angle=%d race=%d command_mm_s=%.1f temporal_gap_s=%.3f capture_s=%.3f boundary_s=%.3f params_valid=%d field_valid=%d roster_valid=%d active_fields=%u\n",
      input.event,input.self,request.kickoffThroughAction,directive.reason,input.plan.reason,
      directive.plan.target.x,directive.plan.target.y,directive.plan.nominalLength,
      directive.plan.nativePass,directive.plan.nativeCapturePass,evidencePeerValid?evidencePeer.stamp:0,
      freshOpponents,coherentOpponents,theGlobalOpponentsModel.opponents.size(),theGlobalOpponentsModel.numOfUnknownOpponents,
      params.restoreProvenPool,!kickoffThroughSession.hasContract?"none":
        directive.plan.rollingRendezvous?"rolling_rendezvous":
        WhiteFleetC15DefenceAdaptive::insideDomain(kickoffThroughSession.launch,directive.plan.target,field,params,directive.plan.side)?"inside_corner":
        WhiteFleetC15DefenceAdaptive::tableTarget(directive.plan.target,field,params,directive.plan.side)?
          (WhiteFleetC9ForwardFlank::preferred(directive.plan.target,field)?"deep":"original_c14"):
          WhiteFleetC15DefenceAdaptive::domain(kickoffThroughSession.launch,directive.plan.target,field,params,directive.plan.side)?"adaptive":"deep",
      params.defenceAdaptiveAngle,params.raceRendezvous,directive.plan.initialSpeed,directive.plan.temporalGap,
      directive.plan.captureTime,directive.plan.controlBoundaryTime,
      Through::paramsValid(params),Through::fieldValid(field),pairActive,activeFields);
    if(params.receiverPrepareHold&&self.number==receiver)
    {
      if(c3WitnessEvent!=input.event){c3WitnessEvent=input.event;c3WitnessCount=0;}
      if(c3WitnessCount<32)
      {
        ++c3WitnessCount;
        const auto prep=Through::preparePoint(field,params,params.side);
        const bool motionStand=theMotionInfo.executedPhase==MotionPhase::stand;
        const bool moved=kickoffThroughSession.hasContract&&input.ballInsidePitch&&input.trackingReady&&
            Through::sessionFresh(input.now,input.localSeen,250)&&Through::finite(input.ballRelative)&&
            Through::distance(input.ball,kickoffThroughSession.launch)>300.f;
        std::printf("C3_START_WITNESS same_strategy_update=1 event=%u self=%d now=%u action=%d own=%d playing=%d prepare_distance_mm=%.3f hold_radius_mm=%.1f consumed_motion_phase=%d consumed_stand_at=%u consumed_phase_at=%u ball_seen=%u ball_motion_eligible=%d contract=%d peer_valid=%d peer_stamp=%u peer_kick_proxy=%u count=%u\n",
          input.event,input.self,input.now,request.kickoffThroughAction,input.ownKickoff,input.playing,
          Through::distance(input.position,prep),Through::receiverPrepareHoldRadius(params,motionStand),
          static_cast<int>(theMotionInfo.executedPhase),theMotionInfo.lastStandTimeStamp,theMotionInfo.lastMotionPhaseStarted,
          input.localSeen,moved,kickoffThroughSession.hasContract,evidencePeerValid,evidencePeer.stamp,evidencePeer.kickProxy,c3WitnessCount);
      }
    }
  }
  return true;
}

bool WhiteFleetStrategyBehaviorControl::applyBackfieldOutlet(SkillRequest& request,const Agent& self)
{
  const bool action=request.skill==SkillRequest::shoot||request.skill==SkillRequest::pass||
    request.skill==SkillRequest::clear||request.skill==SkillRequest::dribble||request.skill==SkillRequest::waitShoot;
  const bool context=WhiteFleetMatchTacticalRepair::enabled(theBehaviorParameters.matchTacticalRepair)&&
    theBehaviorParameters.backfieldStrongOutlet&&theGameState.state==GameState::playing&&!theGameState.stopped&&
    theGameState.playerState==GameState::active&&!request.restartPassRequired&&!request.kickoffThrough&&
    action&&self.role==ActiveRole::toRole(ActiveRole::playBall)&&
    theFieldBall.positionOnField.allFinite()&&theFieldBall.positionOnField.x()<0.f;
  const bool allowed=context&&self.isUpright&&
    theFallDownState.state==FallDownState::upright&&theRobotPose.isFinite()&&theRobotPose.quality!=RobotPose::poor&&
    theBallModel.timeWhenLastSeen>0&&theBallModel.timeWhenLastSeen<=theFrameInfo.time&&
    theFrameInfo.time-theBallModel.timeWhenLastSeen<=200&&theFieldBall.ballWasSeen(200)&&
    theFieldBall.positionOnField.allFinite()&&theFieldBall.velocityRelative.allFinite()&&
    theFieldDimensions.isInsideField(theFieldBall.positionOnField)&&
    theFieldBall.positionRelative.allFinite()&&theFieldBall.positionOnField.x()<0.f;
  if(context)
  {
    for(const Teammate& peer:theTeamData.teammates)
      if(peer.number!=self.number&&peer.theRobotStatus.isUpright&&peer.theRobotPose.isFinite()&&
         peer.theRobotPose.quality!=RobotPose::poor&&
         TeamActivityPolicy::fresh(theFrameInfo.time,peer.theFrameInfo.time,1200)&&
         peer.theBehaviorStatus.passTarget==self.number&&peer.theBehaviorStatus.shootingTo.has_value())
      {outletPlan=OutletPlan{};return false;} // Receiving an actual pass is not a new outlet decision.
    if((request.skill==SkillRequest::shoot||request.skill==SkillRequest::clear)&&
       theMotionInfo.lastKickTimestamp&&theMotionInfo.lastKickTimestamp<=theFrameInfo.time&&
       theFrameInfo.time-theMotionInfo.lastKickTimestamp<=5000&&
       theFieldBall.positionRelative.allFinite()&&theFieldBall.velocityRelative.allFinite()&&
       theFieldBall.positionRelative.norm()>1200.f&&
       theFieldBall.velocityRelative.dot(theFieldBall.positionRelative.normalized())>500.f)
    {outletPlan=OutletPlan{};return false;} // Keep the existing witnessed hot-chase/gaze executor; this grants no new motion permission.
  }
  if(!allowed)
  {
    outletPlan=OutletPlan{};
    if(context){request=SkillRequest::Builder::stand();return true;} // Unsafe own-half data cannot fall back to a light teammate pass.
    return false;
  }
  const unsigned now=theFrameInfo.time;
  const Vector2f ball=theFieldBall.positionOnField;
  if(self.isGoalkeeper&&!WhiteFleetMatchTacticalRepair::keeperMayPursue({ball.x(),ball.y()},
     {theRobotPose.translation.x(),theRobotPose.translation.y()},theFieldDimensions.xPosOwnPenaltyArea,
     theFieldDimensions.yPosLeftPenaltyArea,theBehaviorParameters.keeperPursuitOutsidePenalty,
     theBehaviorParameters.keeperPursuitBuffer,true))
  {
    outletPlan=OutletPlan{};request=SkillRequest::Builder::walkTo(Pose2f(0.f,self.baseTranslation));return true;
  }
  const auto safe=[&](const Agent& peer,Vector2f& target)
  {
    const auto packet=std::find_if(theTeamData.teammates.begin(),theTeamData.teammates.end(),
      [&](const Teammate& t){return t.number==peer.number;});
    if(peer.number==self.number||peer.isGoalkeeper||!peer.isUpright||!peer.pose.isFinite()||
       !TeamActivityPolicy::fresh(now,peer.timestamp,1200)||packet==theTeamData.teammates.end()||
       !packet->theRobotStatus.isUpright||!packet->theRobotPose.isFinite()||packet->theRobotPose.quality==RobotPose::poor||
       !TeamActivityPolicy::fresh(now,packet->theFrameInfo.time,1200))return false;
    if(!WhiteFleetFleetProgressPolicy::frontSupply({ball.x(),ball.y()},{target.x(),target.y()},
       theFieldDimensions.xPosOpponentGoalLine,theFieldDimensions.yPosLeftTouchline))return false;
    const float risk=thePassEvaluation.getRating(ball,target,false);
    if(!std::isfinite(risk)||risk<.7f)return false;
    const auto evaluation=thePassTargetEvaluation.getBestPassData(ball,target,peer.number);
    if(!evaluation.isValid||!evaluation.isAngleFree||!std::isfinite(evaluation.passRating)||evaluation.passRating<.7f||
       !evaluation.bestShiftedTargetOnField.allFinite()||(evaluation.bestShiftedTargetOnField-target).norm()>300.f)return false;
    const float length=(target-ball).norm();
    const float velocity=KickLengthConverter::requestedBallVelocity(KickInfo::rlKickStrong,length,theKickInfo,
      theBallSpecification,theWalkingEngineOutput.kickVelocityRange);
    const float flight=KickLengthConverter::requestedFlightTime(velocity,length,theBallSpecification)*1000.f;
    const float eta=KickSelection::calcTTRP(packet->theRobotPose.inverse()*Pose2f(0.f,target),theWalkingEngineOutput.maxSpeed)+
      static_cast<float>(theFrameInfo.getTimeSince(packet->theFrameInfo.time));
    return std::isfinite(flight)&&flight>0.f&&flight<30000.f&&std::isfinite(eta)&&eta<=flight;
  };
  if(outletPlan.born&&(now<outletPlan.born||now-outletPlan.born>=2500||
     (outletPlan.issued&&theMotionInfo.lastKickTimestamp>=outletPlan.issued)))
    outletPlan=OutletPlan{};
  const Agent* choice=nullptr;Vector2f target;
  if(outletPlan.receiver>0)
    for(const Agent& peer:agents)if(peer.number==outletPlan.receiver)
    {target=outletPlan.target;if(safe(peer,target))choice=&peer;break;}
  if(!choice&&!outletPlan.born)
  {
    float best=-std::numeric_limits<float>::infinity();
    for(const Agent& peer:agents)
    {
      const auto meet=WhiteFleetMatchTacticalRepair::farReceiverMeeting({ball.x(),ball.y()},{peer.pose.translation.x(),peer.pose.translation.y()});
      Vector2f point(meet.x,meet.y);
      if(safe(peer,point)&&point.x()>best){choice=&peer;target=point;best=point.x();}
    }
    if(choice){outletPlan.born=now;outletPlan.receiver=choice->number;outletPlan.target=target;}
  }
  if(choice)
  {
    request=SkillRequest::Builder::passTo(choice->number);request.target=Pose2f(0.f,target);
    request.backfieldStrongOutlet=true;if(!outletPlan.issued)outletPlan.issued=now;
  }
  else
  {
    // Native legal clear, not a short pass or a timed permission to kick blindly.
    request=SkillRequest::Builder::clear();
    request.backfieldStrongOutlet=true;
  }
  if(theFrameInfo.getTimeSince(lastOutletLog)>=500)
  {
    lastOutletLog=now;
    std::printf("FLEET_STRONG_OUTLET self=%d receiver=%d P=%.1f,%.1f admitted=%d native_clear=%d\n",
      self.number,choice?choice->number:-1,choice?target.x():0.f,choice?target.y():0.f,choice!=nullptr,choice==nullptr);
  }
  return true;
}

bool WhiteFleetStrategyBehaviorControl::applyMdfzRules(SkillRequest& request,const Agent& self)
{
  if(mdfzRules.rules.empty())return false;
  const unsigned now=theFrameInfo.time;
  const bool ballRole=Role::isActiveRole(self.role);
  if(theGameState.state!=GameState::playing||theGameState.stopped||theGameState.playerState!=GameState::active||
     self.isGoalkeeper||self.acceptedSetPlay!=SetPlay::none||request.backfieldStrongOutlet||!self.isUpright)
  {mdfzRules.active=-1;return false;}
  if(!ballRole)
    for(const Teammate& peer:theTeamData.teammates)
      if(peer.number!=self.number&&peer.theRobotStatus.isUpright&&
         TeamActivityPolicy::fresh(now,peer.theFrameInfo.time,1200)&&peer.theBehaviorStatus.passTarget==self.number)
      {mdfzRules.active=-1;return false;} // receiving a pass wins
  const Vector2f ball=theFieldBall.recentBallPositionOnField();
  if(!ball.allFinite()){mdfzRules.active=-1;return false;}
  const Vector2f ownGoal(theFieldDimensions.xPosOwnGoalLine,0.f),oppGoal(theFieldDimensions.xPosOpponentGoalLine,0.f);
  float oppBallDist=100000.f;const Vector2f* nearestOpp=nullptr;Vector2f oppPos;
  for(const auto& obstacle:theObstacleModel.obstacles)
    if(obstacle.isOpponent()&&obstacle.lastSeen&&obstacle.lastSeen<=now&&now-obstacle.lastSeen<=1000&&obstacle.center.allFinite())
    {
      const Vector2f p=theRobotPose*obstacle.center;
      if(p.allFinite()&&(p-ball).norm()<oppBallDist){oppBallDist=(p-ball).norm();oppPos=p;nearestOpp=&oppPos;}
    }
  const float values[8]={ball.x(),ball.y(),std::abs(ball.y()),(ball-ownGoal).norm(),(ball-oppGoal).norm(),
    (ball-theRobotPose.translation).norm(),oppBallDist,
    static_cast<float>(static_cast<int>(theGameState.ownTeam.score)-static_cast<int>(theGameState.opponentTeam.score))};
  const Agent* mate=nullptr;
  for(const Agent& peer:agents)
    if(peer.number!=self.number&&!peer.isGoalkeeper&&peer.isUpright)mate=&peer;
  const auto holds=[&](const MdfzRule& rule)
  {
    for(const MdfzCond& cond:rule.conds)
      if(cond.less?!(values[cond.name]<cond.value):!(values[cond.name]>cond.value))return false;
    if(rule.ball&&rule.action==2&&!mate)return false;      // pass needs an upright teammate
    if(!rule.ball&&rule.action==3&&!nearestOpp)return false; // mark needs a seen opponent
    return true;
  };
  int chosen=-1;
  // a rule that fired keeps running for 1 s while the robot keeps its role (no flicker between actions)
  if(mdfzRules.active>=0&&mdfzRules.rules[mdfzRules.active].ball==ballRole&&theFrameInfo.getTimeSince(mdfzRules.since)<1000)
    chosen=mdfzRules.active;
  else
    for(int i=0;i<static_cast<int>(mdfzRules.rules.size());++i)
      if(mdfzRules.rules[i].ball==ballRole&&holds(mdfzRules.rules[i])){chosen=i;break;}
  if(chosen!=mdfzRules.active){mdfzRules.active=chosen;mdfzRules.since=now;if(chosen>=0&&chosen<16)++mdfzRules.fired[chosen];
    if(chosen>=0)std::printf("MDFZ_RULE self=%d t=%u rule=%d %s\n",self.number,now,chosen,mdfzRules.rules[chosen].text.c_str());}
  if(theFrameInfo.getTimeSince(mdfzRules.lastLog)>=10000)
  {
    mdfzRules.lastLog=now;std::printf("MDFZ_RULES_STATS self=%d fired=",self.number);
    for(size_t i=0;i<mdfzRules.rules.size()&&i<16;++i)std::printf("%u%s",mdfzRules.fired[i],i+1<mdfzRules.rules.size()?",":"\n");
  }
  if(chosen<0)return false;
  const MdfzRule& rule=mdfzRules.rules[chosen];
  const auto clampField=[&](Vector2f p)
  {
    p.x()=std::max(theFieldDimensions.xPosOwnGoalArea+300.f,std::min(theFieldDimensions.xPosOpponentGoalLine-300.f,p.x()));
    p.y()=std::max(theFieldDimensions.yPosRightTouchline+300.f,std::min(theFieldDimensions.yPosLeftTouchline-300.f,p.y()));
    return p;
  };
  const auto walkFacingBall=[&](const Vector2f& p){const Vector2f t=clampField(p);return SkillRequest::Builder::walkTo(Pose2f(Angle((ball-t).angle()),t));};
  if(rule.ball)
    switch(rule.action)
    {
      case 0: request=SkillRequest::Builder::shoot();break;
      case 1: request=SkillRequest::Builder::clear();break;
      case 2: request=SkillRequest::Builder::passTo(mate->number);break;
      case 3: request=SkillRequest::Builder::dribbleTo(Angle((oppGoal-ball).angle()));break;
      default: break; // native: keep the team's own decision
    }
  if(rule.ball&&rule.action!=4)request.mdfzRule=true;
  else
    switch(rule.action)
    {
      case 0: request=walkFacingBall(Vector2f(rule.a,rule.b));break;
      case 1: request=walkFacingBall(Vector2f(ball.x()-rule.a,ball.y()));break;
      case 2: request=walkFacingBall(ownGoal+(ball-ownGoal).normalized(std::min(2500.f,(ball-ownGoal).norm()*.5f)));break;
      case 3: request=SkillRequest::Builder::mark(*nearestOpp);break;
      default: break;
    }
  return rule.action!=4; // native: the list ends here and the team's own logic (formation for support) decides
}

bool WhiteFleetStrategyBehaviorControl::applyMdfzFormation(SkillRequest& request,const Agent& self)
{
  // The supporting field player (not the goalkeeper, not the one playing the ball) goes to its tactic base
  // position (t211 / t112 of the class's formation), shifted towards the ball by followBall. Pass reception,
  // set plays (kick-off, free kicks), restarts and the back-field outlet keep priority (checked by the caller and below).
  if(!mdfzFormation.enabled)return false;
  const unsigned now=theFrameInfo.time;
  auto& n=mdfzFormation.n;
  if(theFrameInfo.getTimeSince(mdfzFormation.lastStats)>=10000)
  {
    mdfzFormation.lastStats=now;
    std::printf("MDFZ_FORMATION_STATS self=%d applied=%u notPlaying=%u keeper=%u active=%u setPlay=%u otherSkill=%u passTarget=%u outlet=%u\n",
      self.number,n[0],n[1],n[2],n[3],n[4],n[5],n[6],n[7]);
  }
  if(theGameState.state!=GameState::playing||theGameState.stopped||theGameState.playerState!=GameState::active){++n[1];return false;}
  if(self.isGoalkeeper){++n[2];return false;}
  if(Role::isActiveRole(self.role)){++n[3];return false;}
  if(self.acceptedSetPlay!=SetPlay::none){++n[4];return false;}
  if(request.backfieldStrongOutlet||!(request.skill==SkillRequest::walk||request.skill==SkillRequest::stand)||!self.baseTranslation.allFinite()){++n[5];return false;}
  for(const Teammate& peer:theTeamData.teammates)
    if(peer.number!=self.number&&peer.theRobotStatus.isUpright&&
       TeamActivityPolicy::fresh(now,peer.theFrameInfo.time,1200)&&peer.theBehaviorStatus.passTarget==self.number)
      {++n[6];return false;} // a teammate is passing to this robot: receiving wins
  const Vector2f ball=theFieldBall.recentBallPositionOnField();
  Vector2f target=self.baseTranslation;
  if(ball.allFinite())target+=mdfzFormation.followBall*(ball-self.baseTranslation);
  target.x()=std::max(theFieldDimensions.xPosOwnGoalArea+300.f,std::min(theFieldDimensions.xPosOpponentGoalLine-300.f,target.x()));
  target.y()=std::max(theFieldDimensions.yPosRightTouchline+300.f,std::min(theFieldDimensions.yPosLeftTouchline-300.f,target.y()));
  const Angle facing=Angle(ball.allFinite()?(ball-target).angle():0.f);
  request=SkillRequest::Builder::walkTo(Pose2f(facing,target));
  ++n[0];
  if(theFrameInfo.getTimeSince(mdfzFormation.lastLog)>=1000)
  {
    mdfzFormation.lastLog=now;
    std::printf("MDFZ_FORMATION self=%d tactic=%s base=%.0f,%.0f target=%.0f,%.0f pose=%.0f,%.0f\n",self.number,
      TypeRegistry::getEnumName(self.acceptedTactic),self.baseTranslation.x(),self.baseTranslation.y(),target.x(),target.y(),
      theRobotPose.translation.x(),theRobotPose.translation.y());
  }
  return true;
}

void WhiteFleetStrategyBehaviorControl::applyOpponentSidePosition(SkillRequest& request,const Agent& self)
{
  const unsigned now=theFrameInfo.time;
  const bool allowed=WhiteFleetMatchTacticalRepair::enabled(theBehaviorParameters.matchTacticalRepair)&&
    theBehaviorParameters.opponentSidePosition&&theGameState.state==GameState::playing&&!theGameState.stopped&&
    theGameState.playerState==GameState::active&&!self.isGoalkeeper&&!Role::isActiveRole(self.role)&&
    !request.restartPassRequired&&!request.kickoffThrough&&!request.backfieldStrongOutlet&&self.isUpright&&
    (request.skill==SkillRequest::walk||request.skill==SkillRequest::stand)&&
    theRobotPose.isFinite()&&theRobotPose.quality!=RobotPose::poor&&theFallDownState.state==FallDownState::upright&&
    theBallModel.timeWhenLastSeen>0&&theBallModel.timeWhenLastSeen<=now&&now-theBallModel.timeWhenLastSeen<=650&&
    theFieldBall.ballWasSeen(650)&&theFieldBall.positionOnField.allFinite()&&
    theFieldDimensions.isInsideField(theFieldBall.positionOnField);
  if(!allowed){supportHalf.reset();return;}
  for(const Teammate& peer:theTeamData.teammates)
    if(peer.number!=self.number&&peer.theRobotStatus.isUpright&&
       TeamActivityPolicy::fresh(now,peer.theFrameInfo.time,1200)&&peer.theBehaviorStatus.passTarget==self.number)
      return; // A real pass/receive lease is more important than this soft preference.
  const Vector2f ball=theFieldBall.positionOnField;
  const int half=supportHalf.update(now,theBallModel.timeWhenLastSeen,ball.x(),true);
  if(!half)return;
  if(half<0&&request.skill==SkillRequest::walk&&request.target.translation.x()>0.f)
    for(const Agent& keeper:agents)
      if(keeper.isGoalkeeper&&Role::isActiveRole(keeper.role)&&keeper.isUpright&&
         TeamActivityPolicy::fresh(now,keeper.timestamp,1200)&&
         TeamActivityPolicy::fresh(now,keeper.timeWhenBallLastSeen,1000)&&keeper.ballPosition.norm()<1000.f)
      {
        const Agent* outlet=nullptr;
        for(const Agent& peer:agents)
          if(!peer.isGoalkeeper&&peer.isUpright&&peer.pose.isFinite()&&
             TeamActivityPolicy::fresh(now,peer.timestamp,1200)&&
             (!outlet||peer.pose.translation.x()>outlet->pose.translation.x()||
              (peer.pose.translation.x()==outlet->pose.translation.x()&&peer.number<outlet->number)))
            outlet=&peer;
        if(outlet&&outlet->number==self.number)return; // Preserve ONE native forward outlet; the other field player covers.
      }
  Vector2f opponent;float association=std::numeric_limits<float>::infinity();unsigned observed=0;
  for(const auto& obstacle:theObstacleModel.obstacles)
    if(obstacle.isOpponent()&&obstacle.lastSeen&&obstacle.lastSeen<=now&&now-obstacle.lastSeen<=650&&obstacle.center.allFinite())
    {
      const Vector2f p=theRobotPose*obstacle.center;
      if(!p.allFinite()||std::abs(p.y())>=theFieldDimensions.yPosLeftTouchline||
         p.x()>=theFieldDimensions.xPosOpponentPenaltyArea||(p-ball).norm()>5000.f||obstacle.center.norm()>6000.f)continue;
      const float score=(p-ball).norm()+.2f*(p-theRobotPose.translation).norm();
      if(score<association){association=score;opponent=p;observed=obstacle.lastSeen;}
    }
  if(!observed)return; // The global opponent model has no age: do not forge freshness.
  const Vector2f native=request.skill==SkillRequest::walk?request.target.translation:self.baseTranslation;
  Vector2f bestTarget;float best=std::numeric_limits<float>::infinity();bool found=false;
  for(int side:{0,-1,1})
  {
    const auto p=WhiteFleetFleetProgressPolicy::sideTarget(half,{ball.x(),ball.y()},{opponent.x(),opponent.y()},
      {native.x(),native.y()},theFieldDimensions.xPosOwnGoalLine,theFieldDimensions.xPosOwnGoalArea,theFieldDimensions.xPosOpponentPenaltyArea,
      theFieldDimensions.yPosLeftTouchline,side);
    if(!WhiteFleetFleetProgressPolicy::finite(p))continue;
    Vector2f target(p.x,p.y);bool clear=true;
    for(const Agent& peer:agents)
      if(peer.number!=self.number&&peer.isUpright&&TeamActivityPolicy::fresh(now,peer.timestamp,1200)&&
         peer.pose.isFinite()&&(peer.pose.translation-target).norm()<900.f)clear=false;
    for(const auto& obstacle:theObstacleModel.obstacles)
      if(obstacle.lastSeen&&obstacle.lastSeen<=now&&now-obstacle.lastSeen<=650&&obstacle.center.allFinite()&&
         (theRobotPose*obstacle.center-target).norm()<650.f)clear=false;
    if(!clear)continue;
    const float score=.5f*(target-native).norm()+.2f*(target-theRobotPose.translation).norm()+
      (half<0&&side!=0?500.f:0.f); // Prefer the ball-goal line when it is free; sidestep real blockers.
    if(score<best){best=score;bestTarget=target;found=true;}
  }
  if(!found)return;
  request=SkillRequest::Builder::walkTo(Pose2f((ball-bestTarget).angle(),bestTarget));
  if(theFrameInfo.getTimeSince(lastSupportLog)>=500)
  {
    lastSupportLog=now;
    std::printf("FLEET_OPP_SIDE self=%d half=%d opponent=%.1f,%.1f seen=%u target=%.1f,%.1f native_illegal_avoidance_unchanged=1\n",
      self.number,half,opponent.x(),opponent.y(),observed,bestTarget.x(),bestTarget.y());
  }
}

void WhiteFleetStrategyBehaviorControl::applyDefenseCounter(SkillRequest& request,const Agent& self)
{
  using namespace WhiteFleetDefenseCounterMath;
  const auto point=[](const Vector2f& p){return Point{p.x(),p.y()};};
  const bool keeper=theGameState.ownTeam.isGoalkeeper(theGameState.playerNumber);
  const bool opponentKickoff=theGameState.isKickOff() && theGameState.isForOpponentTeam();
  if(theGameState.stopped)
  { recoveryScreen=false;WhiteFleetDefenseCounterSwitch::record(0);return; }
  if(WhiteFleetMatchTacticalRepair::enabled(theBehaviorParameters.matchTacticalRepair) && !keeper &&
     theGameState.isKickOff() && theGameState.isForOwnTeam() && request.skill==SkillRequest::walk)
  {
    const auto legal=WhiteFleetMatchTacticalRepair::kickoffStaging({request.target.translation.x(),request.target.translation.y()},
      theFieldDimensions.xPosHalfwayLine,theFieldDimensions.fieldLinesWidth,
      theBehaviorParameters.illegalAreasLargeDistance,theFieldDimensions.yPosLeftTouchline);
    request.target=Pose2f(0.f,legal.x,legal.y); // Body preparation only; never changes the ball landing plan.
  }
  if(WhiteFleetMatchTacticalRepair::enabled(theBehaviorParameters.matchTacticalRepair) && applyMatchRestart(request,self))
    return;
  if(!keeper && theGameState.isReady() && opponentKickoff)
  {
    if(WhiteFleetMatchTacticalRepair::enabled(theBehaviorParameters.matchTacticalRepair))
    {
      unsigned rank=0;
      for(int number=Settings::lowestValidPlayerNumber;number<theGameState.playerNumber;++number)
        if(!theGameState.ownTeam.isGoalkeeper(number))++rank;
      const auto slot=WhiteFleetMatchTacticalRepair::defensiveKickoffSlot(rank,theFieldDimensions.centerCircleRadius,
        theFieldDimensions.xPosOwnGoalLine,theFieldDimensions.yPosLeftTouchline);
      // Stable, separated targets, facing canonical opponent half (+X).
      // Native WalkToPointReady owns final orientation/stand and IllegalAreas.
      request=SkillRequest::Builder::walkTo(Pose2f(0.f,slot.x,slot.y));
    }
    else request=SkillRequest::Builder::walkTo(Pose2f(0.f,-1800.f,0.f));
    WhiteFleetDefenseCounterSwitch::record(3);return;
  }
  if(!theGameState.isPlaying() || theGameState.isFreeKick() || theGameState.isPenaltyKick() ||
     theGameState.stopped || theGameState.isPenaltyShootout())
  { recoveryScreen=false;WhiteFleetDefenseCounterSwitch::record(0);return; }
  if(theGameState.isKickOff() && theGameState.isForOwnTeam())
  { recoveryScreen=false;WhiteFleetDefenseCounterSwitch::record(0);return; }

  Point ball{},velocity{};
  bool haveBall=false,local=false;
  if(theFieldBall.ballWasSeen(650) && theFieldBall.positionOnField.allFinite() && theFieldBall.velocityOnField.allFinite())
  {
    ball=point(theFieldBall.positionOnField);velocity=point(theFieldBall.velocityOnField);
    haveBall=true;local=true;
  }
  else
  {
    unsigned newest=0;
    for(const Agent& agent:agents)
      if(agent.number!=self.number && agent.isUpright &&
         TeamActivityPolicy::fresh(theFrameInfo.time,agent.timestamp,1200) &&
         TeamActivityPolicy::fresh(theFrameInfo.time,agent.timeWhenBallLastSeen,1500) &&
         agent.timestamp>=newest && agent.ballPosition.allFinite() && agent.ballVelocity.allFinite())
      {
        ball=point(agent.pose*agent.ballPosition);
        velocity=point(agent.ballVelocity.rotated(agent.pose.rotation));
        newest=agent.timestamp;haveBall=true;
      }
  }
  if(!haveBall || !finite(ball) || !finite(velocity) ||
     std::abs(ball.x)>theFieldDimensions.xPosOpponentGoalLine+150.f ||
     std::abs(ball.y)>theFieldDimensions.yPosLeftTouchline+150.f)
  { recoveryScreen=false;WhiteFleetDefenseCounterSwitch::record(0);return; }
  const Point here=point(theRobotPose.translation);
  if(keeper)
  {
    if(theGameState.isKickOff() && theGameState.isForOwnTeam())
    { WhiteFleetDefenseCounterSwitch::record(0);return; }
    const bool nativeBallAction=request.skill==SkillRequest::shoot || request.skill==SkillRequest::pass ||
      request.skill==SkillRequest::clear || request.skill==SkillRequest::dribble || request.skill==SkillRequest::waitShoot;
    if(nativeBallAction || Role::isActiveRole(self.role))
    { WhiteFleetDefenseCounterSwitch::record(nativeBallAction?6:0);return; }
    if(!WhiteFleetDefenseCounterSwitch::keeperPredictionEnabled())
    { WhiteFleetDefenseCounterSwitch::record(0);return; }
    bool prediction=false;
    const Point target=WhiteFleetDefenseCounterMath::keeper(ball,velocity,theFieldDimensions.xPosOwnGoalLine,
      theFieldDimensions.yPosLeftGoal,prediction);
    const float facing=std::clamp(std::atan2(ball.y-target.y,ball.x-target.x),-.87f,.87f);
    request=SkillRequest::Builder::walkTo(Pose2f(facing,target.x,target.y));
    WhiteFleetDefenseCounterSwitch::record(prediction?2:1);return;
  }
  else if(opponentKickoff)
  {
    if(WhiteFleetMatchTacticalRepair::enabled(theBehaviorParameters.matchTacticalRepair))
    {
      unsigned rank=0;
      for(int number=Settings::lowestValidPlayerNumber;number<theGameState.playerNumber;++number)
        if(!theGameState.ownTeam.isGoalkeeper(number))++rank;
      const auto slot=WhiteFleetMatchTacticalRepair::defensiveKickoffSlot(rank,theFieldDimensions.centerCircleRadius,
        theFieldDimensions.xPosOwnGoalLine,theFieldDimensions.yPosLeftTouchline);
      request=SkillRequest::Builder::walkTo(Pose2f(0.f,slot.x,slot.y));
      WhiteFleetDefenseCounterSwitch::record(3);return;
    }
    const Point target=screen(ball,velocity,theFieldDimensions.xPosOwnGoalLine,
      theFieldDimensions.xPosOwnPenaltyArea,theFieldDimensions.yPosLeftTouchline,true);
    request=SkillRequest::Builder::walkTo(Pose2f((Vector2f(ball.x,ball.y)-theRobotPose.translation).angle(),target.x,target.y));
    WhiteFleetDefenseCounterSwitch::record(3);return;
  }
  else
  {
    float opponentDistance=100000.f;
    for(const auto& opponent:theGlobalOpponentsModel.opponents)
      if(!opponent.unidentified && opponent.position.allFinite())
        opponentDistance=std::min(opponentDistance,distance(ball,point(opponent.position)));
    // Only the existing active field player screens. Do not create a second owner.
    if(WhiteFleetDefenseCounterSwitch::recoveryEnabled() && local && Role::isActiveRole(self.role) && ball.x>theFieldDimensions.xPosOwnGoalLine+1100.f &&
       static_cast<int>(theFrameInfo.time-screenCooldownUntil)>=0 &&
       (!recoveryScreen || theFrameInfo.getTimeSince(recoveryScreenSince)<1800) &&
       shouldScreen(here,ball,velocity,opponentDistance,recoveryScreen))
    {
      if(!recoveryScreen) recoveryScreenSince=theFrameInfo.time;
      recoveryScreen=true;
      const Point target=screen(ball,velocity,theFieldDimensions.xPosOwnGoalLine,
        theFieldDimensions.xPosOwnPenaltyArea,theFieldDimensions.yPosLeftTouchline,false);
      request=SkillRequest::Builder::walkTo(Pose2f((Vector2f(ball.x,ball.y)-theRobotPose.translation).angle(),target.x,target.y));
      WhiteFleetDefenseCounterSwitch::record(4);return;
    }
    if(recoveryScreen) screenCooldownUntil=theFrameInfo.time+800;
    recoveryScreen=false;
    for(const Agent& agent:agents)
      if(agent.isGoalkeeper && Role::isActiveRole(agent.role) &&
         TeamActivityPolicy::fresh(theFrameInfo.time,agent.timestamp,1200) &&
         TeamActivityPolicy::fresh(theFrameInfo.time,agent.timeWhenBallLastSeen,1000) &&
         agent.ballPosition.norm()<1000.f && agent.ballVelocity.norm()<650.f &&
         !Role::isActiveRole(self.role))
      {
        if(counterOutletSide==0 || theFrameInfo.getTimeSince(counterOutletSideSince)>1500)
        {
          float best=-100000.f;int chosen=counterOutletSide?counterOutletSide:1;
          for(int side:{-1,1})
          {
            const Point candidate=outlet(ball,theFieldDimensions.xPosOpponentGoalLine,theFieldDimensions.yPosLeftTouchline,side);
            float clearance=3000.f;
            for(const auto& opponent:theGlobalOpponentsModel.opponents)
              clearance=std::min(clearance,distance(candidate,point(opponent.position)));
            const float rating=clearance-.15f*distance(here,candidate)+(side==counterOutletSide?300.f:0.f);
            if(rating>best){best=rating;chosen=side;}
          }
          counterOutletSide=chosen;counterOutletSideSince=theFrameInfo.time;
        }
        const Point target=outlet(ball,theFieldDimensions.xPosOpponentGoalLine,theFieldDimensions.yPosLeftTouchline,counterOutletSide);
        // Run toward the attacking half; native ReceivePass takes over when a
        // real pass is communicated. Do not backpedal the whole outlet route.
        request=SkillRequest::Builder::walkTo(Pose2f(0.f,target.x,target.y));
        WhiteFleetDefenseCounterSwitch::record(5);return;
      }
  }
  WhiteFleetDefenseCounterSwitch::record(0);
}

bool WhiteFleetStrategyBehaviorControl::applyMatchRestart(SkillRequest& request,const Agent& self)
{
  using namespace WhiteFleetMatchTacticalRepair;
  const unsigned now=theFrameInfo.time;
  const bool keeper=theGameState.ownTeam.isGoalkeeper(self.number);
  const bool ownGoalKick=theGameState.isGoalKick()&&theGameState.isForOwnTeam();
  const bool ownKickoff=theGameState.isKickOff()&&theGameState.isForOwnTeam();
  const bool ownRestart=ownGoalKick||ownKickoff;
  // C15 replaces the generic kickoff completely, including its retired event.
  // The repaired goal-kick contract and its PLAY confirmation tail stay native.
  if(WhiteFleetC15KickoffSwitch::enabled(theBehaviorParameters.matchTacticalRepair&&theBehaviorParameters.c15Kickoff)&&
     (ownKickoff||(!ownGoalKick&&!cornerPlan.goalKick)))
  {cornerPlan=CornerPlan{};return false;}
  if(keeper&&(ownKickoff||(!ownGoalKick&&(!cornerPlan.started||!cornerPlan.goalKick))))
  {cornerPlan=CornerPlan{};return false;} // The keeper does not own the field-player kickoff contract.
  if(!theGameState.isPlaying()||theGameState.isPenaltyShootout())
  {cornerPlan=CornerPlan{};return false;}
  const bool previousCommand=cornerPlan.commandAt==cornerPlan.strikeClock.last&&
    WhiteFleetGoalKickExecution::effectiveCommand(cornerPlan.commandAt,theMotionRequest.ballEstimateTimestamp,
      theMotionRequest.motion==MotionRequest::walkToBallAndKick&&theMotionRequest.kickType==KickInfo::rlKickStrong,
      cornerPlan.commandDirection,theMotionRequest.targetDirection,cornerPlan.commandLength,theMotionRequest.kickLength);
  if(theGameState.stopped)
  {if(cornerPlan.goalKick){cornerPlan.strikeClock.beginFrame(now,previousCommand);cornerPlan.strikeClock.suspend();}return false;}
  if((theGameState.isFreeKick()&&!ownGoalKick)||theGameState.isPenaltyKick()||
     (theGameState.isKickOff()&&!ownKickoff))
  {cornerPlan=CornerPlan{};return false;}
  if(ownRestart&&(!cornerPlan.started||cornerPlan.goalKick!=ownGoalKick||
                 cornerPlan.epoch!=theGameState.timeWhenStateStarted))
  {
    cornerPlan=CornerPlan{};
    cornerPlan.epoch=theGameState.timeWhenStateStarted;
    cornerPlan.started=cornerPlan.epoch?cornerPlan.epoch:now;
    cornerPlan.goalKick=ownGoalKick;
    cornerPlan.origin=Vector2f::Zero(); // Kickoff origin only; goal-kick origin is observed near the ball.
    cornerPlan.originKnown=ownKickoff;
  }
  if(!cornerPlan.started)return false;
  if(cornerPlan.goalKick)cornerPlan.strikeClock.beginFrame(now,previousCommand);

  const bool freshBall=theFieldBall.ballWasSeen(200)&&theFieldBall.positionOnField.allFinite()&&
    theRobotPose.isFinite()&&theRobotPose.quality!=RobotPose::poor;
  const Vector2f ball=theFieldBall.positionOnField;
  const bool insideBall=freshBall&&std::abs(ball.x())<theFieldDimensions.xPosOpponentGoalLine&&
    std::abs(ball.y())<theFieldDimensions.yPosLeftTouchline;
  // Goal-kick fallback: if the pass contract has not struck within 8 s (unsafe
  // receiver, stale peer, waiting arrival...), the keeper plays the ball forward
  // itself instead of standing until the 30 s restart expires. Prefer the front
  // forward (player 3) when it is a fresh upright teammate, else a long forward kick.
  // Triggered only by the referee still showing OUR goal kick (= ball not yet in play),
  // not by the contract or motion kick events (an issued strike that missed the ball
  // or never executed stalled the keeper at the penalty-area corner).
  if(keeper&&ownGoalKick&&cornerPlan.goalKick&&cornerPlan.started&&
     now>=cornerPlan.started&&now-cornerPlan.started>=goalKickFallbackMs&&theFieldBall.ballWasSeen(1000)&&
     theFieldBall.positionRelative.norm()<2500.f&&self.isUpright)
  {
    const auto front=std::find_if(agents.begin(),agents.end(),[&](const Agent& a)
      {return a.number==goalKickFrontForward&&a.number!=self.number&&a.isUpright&&TeamActivityPolicy::fresh(now,a.timestamp,1500);});
    // The aimed pass can take long to align (one drill goal kick never struck in 22 s):
    // after goalKickPassMs of fallback the keeper just kicks long towards the opponent half.
    const bool passing=front!=agents.end()&&now-cornerPlan.started<goalKickFallbackMs+goalKickPassMs;
    request=passing?SkillRequest::Builder::passTo(goalKickFrontForward):SkillRequest::Builder::shoot(); // shoot = long forward kick; clear() may stand when its angle is not forward enough
    if(now-goalKickFallbackLog>=1000)
    {
      goalKickFallbackLog=now;
      std::printf("GOALKICK_FALLBACK self=%d epoch=%u waited_ms=%u action=%s ball=%.0f,%.0f\n",self.number,cornerPlan.epoch,
        now-cornerPlan.started,passing?"pass_front_forward":"long_forward",ball.x(),ball.y());
    }
    return true;
  }
  const auto packet=[&](int number)->const Teammate*
  {
    const auto p=std::find_if(theTeamData.teammates.begin(),theTeamData.teammates.end(),
      [&](const Teammate& t){return t.number==number;});
    return p==theTeamData.teammates.end()?nullptr:&*p;
  };
  const auto freshPeer=[&](const Agent& peer)
  {
    const Teammate* p=packet(peer.number);
    return peer.number!=self.number&&!peer.isGoalkeeper&&peer.isUpright&&peer.pose.isFinite()&&p&&
      TeamActivityPolicy::fresh(now,peer.timestamp,1200)&&p->theRobotStatus.isUpright&&
      p->theRobotPose.isFinite()&&p->theRobotPose.quality!=RobotPose::poor&&
      TeamActivityPolicy::fresh(now,p->theFrameInfo.time,1200);
  };
  float safety=-1.f;
  int nativeStage=0;
  float nativeRating=-1.f,nativeShift=-1.f;
  const auto safeTarget=[&](const Vector2f& target,int number)
  {
    safety=-1.f;nativeStage=0;nativeRating=nativeShift=-1.f;
    if(!freshBall||!target.allFinite()||target.x()<=ball.x()+500.f||
       target.x()>=theFieldDimensions.xPosOpponentGoalLine-600.f||
       std::abs(target.y())>=theFieldDimensions.yPosLeftTouchline-500.f)return false;
    safety=thePassEvaluation.getRating(ball,target,false);
    if(!std::isfinite(safety)){safety=-1.f;return false;}
    if(safety<.7f){nativeStage=1;return false;}
    const auto native=thePassTargetEvaluation.getBestPassData(ball,target,number);
    nativeRating=native.passRating;
    nativeShift=(native.bestShiftedTargetOnField-target).norm();
    nativeStage=!native.isValid?2:!native.isAngleFree?3:
      (!std::isfinite(native.passRating)||native.passRating<.7f)?4:
      (!native.bestShiftedTargetOnField.allFinite()||nativeShift>300.f)?5:6;
    // combinedRating includes future-goal xG, not just pass safety. Preserve
    // the actual native safety probability instead of a scoring-benefit gate.
    return native.isValid&&native.isAngleFree&&std::isfinite(native.passRating)&&native.passRating>=.7f&&
      native.bestShiftedTargetOnField.allFinite()&&(native.bestShiftedTargetOnField-target).norm()<=300.f;
  };
  const auto stamp=[&](int reason,bool preparingOnly)
  {
    request.restartPassRequired=true;request.restartPassPreparing=preparingOnly;
    request.restartPassGoalKick=cornerPlan.goalKick;request.restartPassPasser=cornerPlan.passer;
    request.restartPassReceiver=cornerPlan.receiver;request.restartPassTarget=cornerPlan.target;
    request.restartPassEpoch=cornerPlan.epoch;request.restartPassReason=reason;
    request.restartPassSafety=safety;request.restartPassReadySince=cornerPlan.readySince;
    request.restartPassTargetValid=cornerPlan.goalKick&&cornerPlan.selected;
    request.restartPassStrikeElapsed=cornerPlan.goalKick?cornerPlan.strikeClock.elapsed:0;
    request.restartPassNativeStage=cornerPlan.goalKick&&cornerPlan.selected?nativeStage:0;
    if(cornerPlan.goalKick&&cornerPlan.passer==self.number)
    {
      const int key=reason*100+nativeStage*10+int(cornerPlan.strikeClock.active);
      if(key!=cornerPlan.lastPhaseKey&&(!cornerPlan.lastPhaseDiagnostic||now-cornerPlan.lastPhaseDiagnostic>=500u))
      {
        ANNOTATION("GoalKickPhase", "epoch=" << cornerPlan.epoch << " receiver=" << cornerPlan.receiver <<
          " reason=" << reason << " native_stage=" << nativeStage << " native_rating=" << nativeRating <<
          " shift_mm=" << nativeShift << " strike_active=" << cornerPlan.strikeClock.active <<
          " strike_elapsed_ms=" << cornerPlan.strikeClock.elapsed << " ready_since=" << cornerPlan.readySince <<
          " ball_mm=" << theFieldBall.positionRelative.norm() << " target=" << cornerPlan.target.x() << "," << cornerPlan.target.y());
        cornerPlan.lastPhaseKey=key;cornerPlan.lastPhaseDiagnostic=now;
      }
    }
  };
  const auto prepare=[&](int reason)
  {
    if(cornerPlan.goalKick)cornerPlan.strikeClock.suspend();
    if(keeper&&cornerPlan.goalKick&&insideBall&&theFieldBall.positionRelative.norm()<=650.f&&
       validKeeperLimits(theBehaviorParameters.keeperPursuitOutsidePenalty,theBehaviorParameters.keeperPursuitBuffer)&&
       keeperMargin({theRobotPose.translation.x(),theRobotPose.translation.y()},theFieldDimensions.xPosOwnPenaltyArea,
         theFieldDimensions.yPosLeftPenaltyArea,theBehaviorParameters.keeperPursuitOutsidePenalty)>=0.f&&
       (reason==waitSafety||reason==waitArrival||reason==waitPeer||reason==waitObservation))
    {
      // Revoked near-foot permission is Stand, not a new retreat/turn request
      // whose moving foot can tap the ball while the Strong policy is cancelled.
      request=SkillRequest::Builder::stand();
      stamp(reason,cornerPlan.selected&&!cornerPlan.retired&&cornerPlan.passer==self.number);return true;
    }
    if(keeper&&insideBall&&keeperMayPursue({ball.x(),ball.y()},
       {theRobotPose.translation.x(),theRobotPose.translation.y()},theFieldDimensions.xPosOwnPenaltyArea,
       theFieldDimensions.yPosLeftPenaltyArea,theBehaviorParameters.keeperPursuitOutsidePenalty,
       theBehaviorParameters.keeperPursuitBuffer,true))
    {
      const float angle=cornerPlan.selected?(cornerPlan.target-ball).angle():0.f;
      Vector2f position=ball-Vector2f(400.f,0.f).rotated(angle);
      const auto bounded=keeperPreparation({position.x(),position.y()},theFieldDimensions.xPosOwnPenaltyArea,
        theFieldDimensions.yPosLeftPenaltyArea,theBehaviorParameters.keeperPursuitOutsidePenalty,
        theBehaviorParameters.keeperPursuitBuffer);
      position={bounded.x,bounded.y};
      position.x()=std::max(position.x(),theFieldDimensions.xPosOwnGoalLine+300.f);
      request=SkillRequest::Builder::walkTo(Pose2f(angle,position));
    }
    else if(!keeper&&cornerPlan.passer==self.number&&insideBall)
    {
      const float angle=cornerPlan.selected?(cornerPlan.target-ball).angle():0.f;
      request=SkillRequest::Builder::walkTo(Pose2f(angle,ball-Vector2f(400.f,0.f).rotated(angle)));
    }
    else if(!keeper)
    {
      Vector2f desired=reason==waitPeer&&cornerPlan.receiver==self.number?theRobotPose.translation:
        (cornerPlan.selected&&cornerPlan.receiver==self.number?cornerPlan.target:self.baseTranslation);
      // During OUR goal kick only, ordinary playBall must not send the spare
      // field player onto the keeper's ball. A real addressed receiver keeps P.
      const bool frontForward=ownGoalKick&&!keeper&&self.number==goalKickFrontForward&&
        !(cornerPlan.selected&&cornerPlan.receiver==self.number);
      if(frontForward)
        desired=Vector2f(goalKickFrontX,(insideBall&&ball.y()>0.f?-1.f:1.f)*goalKickFrontY); // outlet in the opponent half, ball-far side
      if(!frontForward&&ownGoalKick&&!(cornerPlan.selected&&cornerPlan.receiver==self.number)&&
         std::any_of(agents.begin(),agents.end(),[&](const Agent& a){return a.number!=self.number&&!a.isGoalkeeper;}))
      {
        if(!self.isUpright||!theRobotPose.isFinite()||theRobotPose.quality==RobotPose::poor)
        {request=SkillRequest::Builder::stand();stamp(waitObservation,false);return true;}
        Vector2f yieldBall=ball;
        bool haveYieldBall=insideBall;
        unsigned yieldSeen=insideBall?theBallModel.timeWhenLastSeen:0;
        const Teammate* keeperIntent=nullptr;
        for(const Teammate& p:theTeamData.teammates)
          if(p.isGoalkeeper&&p.theRobotStatus.isUpright&&p.theRobotPose.isFinite()&&
             p.theRobotPose.quality!=RobotPose::poor&&
             TeamActivityPolicy::fresh(now,p.theFrameInfo.time,1200)&&p.theFrameInfo.time>=cornerPlan.epoch)
          {
            // Shared vision is POSITIONING evidence only, never kick permission.
            // The spare player's own camera can be stale while facing away.
            if(TeamActivityPolicy::fresh(now,p.theBallModel.timeWhenLastSeen,300)&&p.theBallModel.timeWhenLastSeen>=cornerPlan.epoch&&
               p.theBallModel.estimate.position.allFinite()&&p.theBallModel.estimate.position.norm()<=1500.f)
            {
              const Vector2f seen=p.theRobotPose*p.theBallModel.estimate.position;
              if(seen.allFinite()&&std::abs(seen.x())<theFieldDimensions.xPosOpponentGoalLine&&
                 std::abs(seen.y())<theFieldDimensions.yPosLeftTouchline)
              {yieldBall=seen;yieldSeen=p.theBallModel.timeWhenLastSeen;haveYieldBall=true;}
            }
            if(p.theBehaviorStatus.passTarget>0&&p.theBehaviorStatus.passTarget!=self.number&&
               p.theBehaviorStatus.shootingTo.has_value())keeperIntent=&p;
          }
        if(!haveYieldBall)
        {
          // Hold a previous defensive TARGET across normal ~0.5s packets, not
          // a fabricated fresh ball/peer. Old observations never renew this age.
          if(cornerPlan.yieldSeen&&now>=cornerPlan.yieldSeen&&now-cornerPlan.yieldSeen<=750u&&cornerPlan.yieldTarget.allFinite())
          {request=SkillRequest::Builder::walkTo(Pose2f(0.f,cornerPlan.yieldTarget));stamp(preparing,false);return true;}
          request=SkillRequest::Builder::stand();stamp(waitObservation,false);return true;
        }
        bool yield=(desired-yieldBall).norm()<1800.f||(theRobotPose.translation-yieldBall).norm()<1800.f;
        if(keeperIntent)
        {
          const Vector2f target=keeperIntent->theRobotPose*keeperIntent->theBehaviorStatus.shootingTo.value();
          yield=yield||WhiteFleetGoalKickExecution::blocksLane({yieldBall.x(),yieldBall.y()},{target.x(),target.y()},
            {desired.x(),desired.y()})||WhiteFleetGoalKickExecution::blocksLane({yieldBall.x(),yieldBall.y()},{target.x(),target.y()},
            {theRobotPose.translation.x(),theRobotPose.translation.y()});
        }
        if(yield)
        {
          const auto cover=WhiteFleetGoalKickExecution::coverPoint({yieldBall.x(),yieldBall.y()},theFieldDimensions.xPosOwnGoalLine,
            theFieldDimensions.yPosLeftTouchline);
          desired={cover.x,cover.y};
          cornerPlan.yieldTarget=desired;cornerPlan.yieldSeen=yieldSeen;
        }
      }
      if(ownKickoff)
      {
        const auto p=kickoffStaging({desired.x(),desired.y()},theFieldDimensions.xPosHalfwayLine,
          theFieldDimensions.fieldLinesWidth,theBehaviorParameters.illegalAreasLargeDistance,
          theFieldDimensions.yPosLeftTouchline);
        request=SkillRequest::Builder::walkTo(Pose2f(0.f,p.x,p.y));
      }
      else request=SkillRequest::Builder::walkTo(Pose2f(0.f,desired));
    }
    else if(theRobotPose.isFinite()&&theRobotPose.quality!=RobotPose::poor&&self.baseTranslation.allFinite()&&
      validKeeperLimits(theBehaviorParameters.keeperPursuitOutsidePenalty,theBehaviorParameters.keeperPursuitBuffer)&&
      keeperMargin({theRobotPose.translation.x(),theRobotPose.translation.y()},theFieldDimensions.xPosOwnPenaltyArea,
        theFieldDimensions.yPosLeftPenaltyArea,theBehaviorParameters.keeperPursuitOutsidePenalty)<0.f)
      request=SkillRequest::Builder::walkTo(Pose2f(0.f,self.baseTranslation)); // Preserve native keeper return, never blind chase outside the bound.
    else request=SkillRequest::Builder::stand(); // No usable observation: gaze/search, not a fabricated ball.
    stamp(reason,cornerPlan.selected&&!cornerPlan.retired&&cornerPlan.passer==self.number);
    return true;
  };

  // An already-announced pair is a RESTART-local lease, not a second general
  // owner election. A receiver must not turn native playBall into a reverse pass.
  if(!keeper&&(!cornerPlan.selected||cornerPlan.passer!=self.number))
    for(const Teammate& p:theTeamData.teammates)
      if(p.number!=self.number&&p.theRobotStatus.isUpright&&p.theRobotPose.isFinite()&&
         p.theRobotPose.quality!=RobotPose::poor&&TeamActivityPolicy::fresh(now,p.theFrameInfo.time,1200)&&
         p.theFrameInfo.time>=cornerPlan.epoch&&p.theBehaviorStatus.passTarget==self.number&&
         p.theBehaviorStatus.shootingTo.has_value()&&(!cornerPlan.goalKick||p.isGoalkeeper))
      {
        const Vector2f target=p.theRobotPose*p.theBehaviorStatus.shootingTo.value();
        if(!target.allFinite()||target.x()>=theFieldDimensions.xPosOpponentGoalLine-600.f||
           std::abs(target.y())>=theFieldDimensions.yPosLeftTouchline-500.f)continue;
        cornerPlan.selected=true;cornerPlan.passer=p.number;cornerPlan.receiver=self.number;
        cornerPlan.target=target;cornerPlan.lastIntent=p.theFrameInfo.time;
        break;
      }
  if(cornerPlan.selected&&cornerPlan.receiver==self.number&&!cornerPlan.complete)
  {
    if(!ownRestart&&now-cornerPlan.lastIntent>1200)
    {cornerPlan=CornerPlan{};request.restartPassReason=waitPeer;return false;}
    if(!ownRestart&&freshBall&&theFieldBall.positionRelative.norm()<800.f)
    {cornerPlan.complete=true;request.restartPassReason=receiving;return false;} // Visual acquisition is NOT passer-release evidence.
    return prepare(now-cornerPlan.lastIntent<=1200?receiving:waitPeer);
  }

  if(cornerPlan.selected&&!cornerPlan.retired&&cornerPlan.passer==self.number&&freshBall&&
     restartReleased(now,cornerPlan.issued,theMotionInfo.lastKickTimestamp,theBallModel.timeWhenLastSeen,
       {cornerPlan.origin.x(),cornerPlan.origin.y()},{cornerPlan.target.x(),cornerPlan.target.y()},{ball.x(),ball.y()}))
  {
    cornerPlan.complete=true;cornerPlan.inFlightUntil=now+5000;
  }
  if(cornerPlan.complete)
  {
    const bool originalPasser=cornerPlan.passer==self.number;
    if(keeper&&ownGoalKick)
      request=SkillRequest::Builder::walkTo(Pose2f(0.f,self.baseTranslation));
    const float ownDistance=(ball-theRobotPose.translation).norm();
    if(freshBall&&ownDistance>1200.f)cornerPlan.departed=true;
    const bool returned=cornerPlan.departed&&freshBall&&ownDistance<800.f;
    const auto receiver=std::find_if(agents.begin(),agents.end(),[&](const Agent& p){return p.number==cornerPlan.receiver;});
    const bool received=receiver!=agents.end()&&freshPeer(*receiver)&&
      TeamActivityPolicy::fresh(now,receiver->timeWhenBallLastSeen,300)&&receiver->ballPosition.norm()<800.f;
    if(originalPasser&&!received&&!returned&&receiver!=agents.end()&&freshPeer(*receiver)&&now<cornerPlan.inFlightUntil)
    {
      request.restartPassInFlight=true;request.restartPassReceiver=cornerPlan.receiver;
      request.restartPassTarget=cornerPlan.target;request.restartPassEpoch=cornerPlan.epoch;
      request.restartPassPasser=cornerPlan.passer;request.restartPassGoalKick=cornerPlan.goalKick;
    }
    request.restartPassReason=originalPasser?ourRelease:receiving;
    return keeper&&ownGoalKick; // Keep the keeper from starting another free-kick shot after the real pass.
  }
  // Motion can report a contact before the camera frame has advanced beyond
  // that timestamp. Wait only for confirmation, never issue another strike
  // or fall through to waitShoot during this bounded cross-thread gap.
  const unsigned event=theMotionInfo.lastKickTimestamp;
  const bool recentEvent=cornerPlan.issued&&event>=cornerPlan.issued&&
    (event>now?event-now<=250u:now-event<750u);
  const bool sameNearUnconfirmed=keeper&&!ownRestart&&cornerPlan.goalKick&&!cornerPlan.retired&&
    event<=now&&
    theGameState.state==GameState::playing&&cornerPlan.selected&&cornerPlan.issued&&cornerPlan.readySince&&
    now>=cornerPlan.readySince&&!cornerPlan.strikeClock.exhausted()&&
    now>=cornerPlan.started&&now-cornerPlan.started<goalKickContractBudget&&insideBall&&
    theFieldBall.positionRelative.norm()<=650.f&&(ball-cornerPlan.origin).norm()<=1000.f&&self.isUpright;
  if(sameNearUnconfirmed&&!cornerPlan.unconfirmedAt)cornerPlan.unconfirmedAt=now;
  const bool recoverSameGoalPass=sameNearUnconfirmed&&cornerPlan.unconfirmedAt&&
    now>=cornerPlan.unconfirmedAt&&now-cornerPlan.unconfirmedAt<=750u;
  if(keeper&&!ownRestart&&cornerPlan.goalKick&&!cornerPlan.retired&&freshBall&&recentEvent&&
     now>=cornerPlan.issued&&!cornerPlan.strikeClock.exhausted()&&
     now>=cornerPlan.started&&now-cornerPlan.started<goalKickContractBudget)
  {
    request=SkillRequest::Builder::walkTo(Pose2f(0.f,self.baseTranslation));
    stamp(waitObservation,true);return true;
  }
  // External play cancels an UNRELEASED contract, never signs it as our kick.
  // A goal-kick placement while the same restart is active cannot enter here.
  if(!recoverSameGoalPass&&!ownRestart&&cornerPlan.originKnown&&freshBall&&(ball-cornerPlan.origin).norm()>500.f)
  {
    const int reason=cornerPlan.issued?unconfirmedRelease:externalRelease;
    cornerPlan=CornerPlan{};request.restartPassReason=reason;return false;
  }
  if(!recoverSameGoalPass&&keeper&&!ownRestart&&cornerPlan.goalKick)
  {cornerPlan=CornerPlan{};request.restartPassReason=unconfirmedRelease;return false;} // New legal context: preserve native GK defence, do not sign a pass.
  if(!ownRestart&&cornerPlan.goalKick&&!cornerPlan.issued&&cornerPlan.passer!=self.number)
  {cornerPlan=CornerPlan{};return false;} // No goal-kick contract was addressed to this field player.

  const unsigned budget=cornerPlan.goalKick?goalKickContractBudget:kickoffContractBudget;
  if(now<cornerPlan.started||now-cornerPlan.started>=budget||
     (cornerPlan.goalKick?cornerPlan.strikeClock.exhausted():
       (cornerPlan.readySince&&now-cornerPlan.readySince>=strikeBudget)))
    cornerPlan.retired=true;
  if(cornerPlan.retired)
  {
    request=SkillRequest::Builder::walkTo(Pose2f(0.f,self.baseTranslation));
    stamp(budgetExpired,false);request.restartPassRequired=false;
    return true; // Bounded attempt ended explicitly; no hidden clear/near-pass/shot.
  }
  if(keeper&&!cornerPlan.goalKick)return false;
  if(!keeper&&cornerPlan.goalKick)return prepare(preparing);
  if(!keeper&&!cornerPlan.selected&&self.role!=ActiveRole::toRole(ActiveRole::playBall))
    return prepare(preparing);
  cornerPlan.passer=self.number;
  if(!freshBall)return prepare(waitObservation);

  // Goal-kick origin is committed only after NEW observations are stable and
  // close; no 3-second strike timer runs while walking to a relocated ball.
  const bool anchored=cornerPlan.anchor.update(now,theBallModel.timeWhenLastSeen,cornerPlan.epoch,
    {ball.x(),ball.y()},insideBall&&self.isUpright&&theFieldBall.positionRelative.norm()<=1000.f&&
    theFieldBall.velocityRelative.norm()<=650.f);
  if(cornerPlan.goalKick&&!anchored&&!cornerPlan.issued)return prepare(waitObservation);
  if(cornerPlan.goalKick&&anchored&&!cornerPlan.issued)
  {cornerPlan.origin=ball;cornerPlan.originKnown=true;}
  if(!cornerPlan.goalKick&&(!insideBall||(ball-Vector2f::Zero()).norm()>500.f)&&!cornerPlan.issued)
    return prepare(waitObservation);

  if(cornerPlan.goalKick&&cornerPlan.selected&&!cornerPlan.issued&&now>=cornerPlan.proposalAt&&now-cornerPlan.proposalAt>=1000u)
  {
    const auto proposed=std::find_if(agents.begin(),agents.end(),[&](const Agent& p){return p.number==cornerPlan.receiver;});
    if(proposed!=agents.end()&&freshPeer(*proposed)&&!safeTarget(cornerPlan.target,cornerPlan.receiver))
    {cornerPlan.selected=false;cornerPlan.receiver=-1;cornerPlan.target=Vector2f::Zero();}
  }
  if(!cornerPlan.selected)
  {
    const Agent* choice=nullptr;Vector2f choiceTarget;float best=-1e9f;
    const Agent* potential=nullptr;Vector2f potentialTarget;float potentialScore=-1e9f;
    for(const Agent& peer:agents)
    {
      if(!freshPeer(peer))continue;
      if(cornerPlan.goalKick)
      {
        const auto point=WhiteFleetGoalKickExecution::meeting({ball.x(),ball.y()},{peer.pose.translation.x(),peer.pose.translation.y()});
        if(!point)continue; // Invalid geometry must never become a plausible centre-field target.
        const Vector2f target(point->x,point->y);
        if(!forwardOutlet({ball.x(),ball.y()},{target.x(),target.y()},
             theFieldDimensions.xPosOpponentGoalLine,theFieldDimensions.yPosLeftTouchline))continue;
        const float distance=(peer.pose.translation-ball).norm();
        if(distance>potentialScore){potentialScore=distance;potential=&peer;potentialTarget=target;}
        Vector2f candidate=target;
        bool safe=safeTarget(candidate,peer.number);
        if(!safe)
        {
          // Before issue only, consider the native free-angle result as a NEW
          // proposal. It must pass the original .7/300mm gates again, not bypass
          // the old point's shift limit. No issued receiver/P may change.
          const auto adjusted=thePassTargetEvaluation.getBestPassData(ball,target,peer.number);
          if(adjusted.isValid&&adjusted.isAngleFree&&std::isfinite(adjusted.passRating)&&adjusted.passRating>=.7f&&
             adjusted.bestShiftedTargetOnField.allFinite()&&forwardOutlet({ball.x(),ball.y()},
               {adjusted.bestShiftedTargetOnField.x(),adjusted.bestShiftedTargetOnField.y()},
               theFieldDimensions.xPosOpponentGoalLine,theFieldDimensions.yPosLeftTouchline)&&
             safeTarget(adjusted.bestShiftedTargetOnField,peer.number))
          {candidate=adjusted.bestShiftedTargetOnField;safe=true;}
        }
        if(safe&&(distance>best||(distance==best&&choice&&peer.number<choice->number)))
        {best=distance;choice=&peer;choiceTarget=candidate;}
      }
      else
      {
        for(const int side:{-1,1})
        {
          const auto point=penaltyCorner(theFieldDimensions.xPosOpponentPenaltyArea,theFieldDimensions.yPosLeftPenaltyArea,
            theFieldDimensions.xPosOpponentGoalLine,theFieldDimensions.yPosLeftTouchline,side,theBehaviorParameters.kickoffCornerOutside);
          const Vector2f target(point.x,point.y);
          const float eta=KickSelection::calcTTRP(peer.pose.inverse()*Pose2f(0.f,target),theWalkingEngineOutput.maxSpeed);
          if(!std::isfinite(eta))continue;
          const float preference=-eta;
          if(preference>potentialScore){potentialScore=preference;potential=&peer;potentialTarget=target;}
          if(safeTarget(target,peer.number)&&(safety-eta*.00001f>best))
          {best=safety-eta*.00001f;choice=&peer;choiceTarget=target;}
        }
      }
    }
    if(!choice){choice=potential;choiceTarget=potentialTarget;}
    if(!choice)return prepare(waitPeer);
    cornerPlan.selected=true;cornerPlan.receiver=choice->number;cornerPlan.target=choiceTarget;
    cornerPlan.proposalAt=now;
  }
  const auto peer=std::find_if(agents.begin(),agents.end(),[&](const Agent& p){return p.number==cornerPlan.receiver;});
  if(peer==agents.end()||!freshPeer(*peer))return prepare(waitPeer);
  if(!safeTarget(cornerPlan.target,peer->number))return prepare(waitSafety);
  const Teammate* p=packet(peer->number);
  const float length=(cornerPlan.target-ball).norm();
  const auto type=cornerPlan.goalKick?KickInfo::rlKickStrong:KickInfo::rlKickDynamic;
  const float velocity=KickLengthConverter::requestedBallVelocity(type,length,theKickInfo,
    theBallSpecification,theWalkingEngineOutput.kickVelocityRange);
  const float flight=KickLengthConverter::requestedFlightTime(velocity,length,theBallSpecification)*1000.f;
  const float eta=KickSelection::calcTTRP(p->theRobotPose.inverse()*Pose2f(0.f,cornerPlan.target),theWalkingEngineOutput.maxSpeed)+
    static_cast<float>(theFrameInfo.getTimeSince(p->theFrameInfo.time))+(ownKickoff?1200.f:0.f);
  if(!std::isfinite(flight)||flight<=0.f||flight>=30000.f||!std::isfinite(eta)||eta>flight)
    return prepare(waitArrival);
  if(!insideBall||!self.isUpright)return prepare(waitObservation);
  if(keeper&&!keeperMayPursue({ball.x(),ball.y()},
     {theRobotPose.translation.x(),theRobotPose.translation.y()},theFieldDimensions.xPosOwnPenaltyArea,
     theFieldDimensions.yPosLeftPenaltyArea,theBehaviorParameters.keeperPursuitOutsidePenalty,
     theBehaviorParameters.keeperPursuitBuffer,cornerPlan.issued!=0))return prepare(waitObservation);
  if(cornerPlan.goalKick)
  {
    const Angle direction=Angle::normalize((cornerPlan.target-ball).angle()-theRobotPose.rotation);
    const Vector2f kickPosition=ball+theKickInfo[type].ballOffset.rotated((cornerPlan.target-ball).angle());
    const bool aligned=WhiteFleetGoalKickExecution::geometryReady(theFieldBall.positionRelative.norm(),
      theFieldBall.positionRelative.angle(),direction,(kickPosition-theRobotPose.translation).norm(),theFieldBall.velocityRelative.norm());
    cornerPlan.strikeClock.admit(now,aligned);
    if(aligned&&!cornerPlan.readySince)cornerPlan.readySince=now;
    // Still use the SAME native directed pass to approach/align. Geometric
    // readiness is not an invented kick event and does not replace any gate.
  }
  else if(!cornerPlan.readySince&&theFieldBall.positionRelative.norm()<=1000.f)
    cornerPlan.readySince=now;
  if(!cornerPlan.issued)
  {
    cornerPlan.origin=cornerPlan.goalKick?ball:Vector2f::Zero();
    cornerPlan.issued=now; // Admitted PLAN request; actual release still needs a new native event + observed travel.
  }
  request=SkillRequest::Builder::passTo(cornerPlan.receiver);
  cornerPlan.commandAt=now;
  cornerPlan.commandDirection=Angle::normalize((cornerPlan.target-ball).angle()-theRobotPose.rotation);
  cornerPlan.commandLength=length;
  request.target=Pose2f(0.f,cornerPlan.target);stamp(passAdmitted,false);
  return true;
}

Agent* WhiteFleetStrategyBehaviorControl::updateAgents()
{
  // Add agents that are active now but weren't before.
  for(unsigned int i = 0; i < theGameState.ownTeam.playerStates.size(); ++i)
  {
    const int number = Settings::lowestValidPlayerNumber + i;
    if((number == theGameState.playerNumber ? theGameState.playerState : theGameState.ownTeam.playerStates[i]) != GameState::active)
      continue;
    if(std::any_of(agents.begin(), agents.end(), [&](const Agent& agent) {return agent.number == number;}))
    continue;
    // Fail safe. We assume the teammate send a package which never reached us. Initialize with dummy data
    if(theFrameInfo.getTimeSince(theGameState.ownTeam.timeWhenPlayerStatesStarted[i]) > 5000 || number == theGameState.playerNumber)
    {
      agents.emplace_back();
      Agent& agent = agents.back();
      agent.number = number;
      agent.lastKnownTimestamp = theFrameInfo.time; // This is to avoid that "self" will write things into lastKnown* that were already sent a long time ago.
      agent.lastKnownPose = Vector2f(theFieldDimensions.xPosReturnFromPenalty, number % 2 ? theFieldDimensions.yPosLeftReturnFromPenalty : theFieldDimensions.yPosRightReturnFromPenalty);
    }
  }

  // Remove agents that are not active anymore.
  for(auto it = agents.begin(); it != agents.end();)
  {
    Agent& agent = *it;
    if((agent.number == theGameState.playerNumber ? theGameState.playerState : theGameState.ownTeam.playerStates[agent.number - Settings::lowestValidPlayerNumber]) != GameState::active)
      it = agents.erase(it);
    else
    {
      agent.isGoalkeeper = theGameState.ownTeam.isGoalkeeper(agent.number);
      ++it;
    }
  }

  for(const ReceivedTeamMessage& teamMessage : theReceivedTeamMessages.messages)
  {
    auto it = std::find_if(agents.begin(), agents.end(), [&](const Agent& agent) {return agent.number == teamMessage.number; });
    if(it != agents.end())
      updateAgentByTeamMessage(*it, teamMessage);
    else
    {
      // Add agent that is active now but wasn't before.
      agents.emplace_back();
      Agent& agent = agents.back();
      agent.number = teamMessage.number;
      agent.isGoalkeeper = theGameState.ownTeam.isGoalkeeper(agent.number);
      updateAgentByTeamMessage(agent, teamMessage);
    }
  }

  // SET may seed hypothetical peers with a pose and a ball. This is useful for
  // formation planning, but it is not proof that a peer exists or still sees it.
  // Prune only in PLAY and before taking the self pointer / assigning positions,
  // so search regions, role voting and the active-agent election see one roster.
  if(theBehaviorParameters.requireFreshTeamActivity)
    agents.erase(std::remove_if(agents.begin(), agents.end(), [&](const Agent& agent)
    {
      return !TeamActivityPolicy::retainAgent(true, theGameState.isPlaying(), agent.number == theGameState.playerNumber,
                                            theFrameInfo.time, agent.timestamp, theBehaviorParameters.teamActivityTimeout);
    }), agents.end());

  // The list of agents is now final for this frame, so the self pointer can be set.
  Agent* self = nullptr;
  for(Agent& agent : agents)
    if(agent.number == theGameState.playerNumber)
    {
      self = &agent;
      break;
    }

  if(self)
    updateAgentBySelf(*self);

  if(theGameState.kickOffSetupFromTouchlines)
  {
    for(Agent& agent : agents)
    {
      agent.lastKnownPose = theSetupPoses.getPoseOfRobot(agent.number).position;
    }
  }
  else if(theGameState.isSet() && theExtendedGameState.wasReady())
  {
    for(Agent& agent : agents)
    {
      agent.lastKnownPose = agent.baseTranslation;
      // Pretend that all other agents are where they are supposed to be and see the ball where they are supposed to see it - until the first message in playing arrives.
      if(&agent != self)
      {
        agent.pose = agent.baseTranslation;
        agent.ballPosition = agent.pose.inverse() * (theGameState.isPenaltyKick() ? Vector2f(theGameState.isForOwnTeam() ? theFieldDimensions.xPosOpponentPenaltyMark : theFieldDimensions.xPosOwnPenaltyMark, 0.f) :  Vector2f::Zero());
        agent.ballVelocity = Vector2f::Zero();
        agent.timeWhenBallLastSeen = agent.timestamp;
        agent.timeWhenBallDisappeared = agent.timestamp;
        agent.disagreeOnBall = false;
        agent.timeSinceDisagreeOnBall = theFrameInfo.time;
        agent.isUpright = true;
        agent.timeWhenLastUpright = agent.timestamp;
        if(self)
        {
          agent.proposedTactic = self->proposedTactic;
          agent.proposedSetPlay = self->proposedSetPlay;
          agent.proposedMirror = self->proposedMirror;
        }
      }
    }
  }

  for(Agent& agent : agents)
    updateCurrentPosition(agent);

  return self;
}

void WhiteFleetStrategyBehaviorControl::updateAgentBySelf(Agent& agent)
{
  if(theSentTeamMessage.theFrameInfo.time > agent.lastKnownTimestamp)
  {
    agent.lastKnownTimestamp = theSentTeamMessage.theFrameInfo.time;
    agent.lastKnownPose = theSentTeamMessage.theRobotPose;
    agent.lastKnownTarget = theSentTeamMessage.theBehaviorStatus.walkingTo;
    agent.lastKnownSpeed = theSentTeamMessage.theBehaviorStatus.speed;
  }
  agent.timestamp = theFrameInfo.time;
  agent.pose = theRobotPose;
  agent.ballPosition = theBallModel.estimate.position;
  agent.ballVelocity = theBallModel.estimate.velocity;
  agent.timeWhenBallLastSeen = theBallModel.timeWhenLastSeen;
  agent.timeWhenBallDisappeared = theBallModel.timeWhenDisappeared;
  agent.disagreeOnBall = false;
  agent.isUpright = (theFallDownState.state == FallDownState::upright || theFallDownState.state == FallDownState::staggering || theFallDownState.state == FallDownState::squatting) &&
                    (theMotionInfo.executedPhase != MotionPhase::getUp && theMotionInfo.executedPhase != MotionPhase::fall);
  if(agent.isUpright)
    agent.timeWhenLastUpright = theFrameInfo.time;
  agent.proposedTactic = theStrategyStatus.proposedTactic;
  agent.acceptedTactic = theStrategyStatus.acceptedTactic;
  agent.proposedMirror = theStrategyStatus.proposedMirror;
  agent.acceptedMirror = theStrategyStatus.acceptedMirror;
  agent.proposedSetPlay = theStrategyStatus.proposedSetPlay;
  agent.acceptedSetPlay = theStrategyStatus.acceptedSetPlay;
  agent.setPlayStep = theStrategyStatus.setPlayStep;
  agent.position = theStrategyStatus.position;
  agent.role = theStrategyStatus.role;
}

void WhiteFleetStrategyBehaviorControl::updateAgentByTeamMessage(Agent& agent, const ReceivedTeamMessage& teamMessage)
{
  agent.lastKnownTimestamp = teamMessage.theFrameInfo.time;
  agent.lastKnownPose = teamMessage.theRobotPose;
  agent.lastKnownTarget = teamMessage.theBehaviorStatus.walkingTo;
  agent.lastKnownSpeed = teamMessage.theBehaviorStatus.speed;
  agent.timestamp = teamMessage.theFrameInfo.time;
  agent.pose = teamMessage.theRobotPose;
  agent.ballPosition = teamMessage.theBallModel.estimate.position;
  agent.ballVelocity = teamMessage.theBallModel.estimate.velocity;
  agent.timeWhenBallLastSeen = teamMessage.theBallModel.timeWhenLastSeen;
  agent.timeWhenBallDisappeared = teamMessage.theBallModel.timeWhenDisappeared;
  // Calculate disagreeOnBall.
  {
    // In theory, all those calculations should be made at the time teamMessage.theFrameInfo.time and therefore, there should be a buffer of the last few own RobotPoses and BallModels to compare with.
    // The following is okay as long as the network delay is sufficiently short.
    // Disagreement can come from either delocalized robots (where both would walk to the same physical ball, but at least one of them would play in the wrong direction) or from wrong balls (in which case both must go to their respective ball).
    // Agents who disagree on the ball with me will not be considered in my decision to play the ball.
    // Therefore, it is worse if both players wrongly assume no disagreement (while in fact they would go to two different balls).

    // Also, it could be nice if this decision was made somewhere in the modeling stage because similar calculations could happen in the TeamBallModel.
    if(teamMessage.theFrameInfo.getTimeSince(teamMessage.theBallModel.timeWhenLastSeen) > 1000 ||
       theFrameInfo.getTimeSince(theBallModel.timeWhenLastSeen) > 1000)
    {
      // If one of us hasn't seen the ball recently, this is not disagreement.
      // One could instead check abs(teamMessage.theBallModel.timeWhenLastSeen - theBallModel.timeWhenLastSeen).
      // This is upper-bounded by 1000 here, but if both balls are old, it doesn't really matter anyway since the ball can have moved long ago and neither robot would become striker.
      agent.disagreeOnBall = false;
      agent.timeSinceDisagreeOnBall = teamMessage.theBallModel.timeWhenLastSeen;
    }
    else
    {
      // Compare "now", i.e. propagate the teammate's ball to the current time.
      const Vector2f itsBallInRobot = BallPhysics::propagateBallPosition(teamMessage.theBallModel.estimate.position, teamMessage.theBallModel.estimate.velocity, static_cast<float>(theFrameInfo.getTimeSince(teamMessage.theFrameInfo.time)) / 1000.f, theBallSpecification);
      const Vector2f itsBallOnField = teamMessage.theRobotPose * itsBallInRobot;
      const Vector2f myBallOnField = theRobotPose * theBallModel.estimate.position;

      const float maxDistanceToBall = std::max(itsBallInRobot.norm(), theBallModel.estimate.position.norm());
      const float baseThreshold = mapToRange(maxDistanceToBall, 2500.f, 15000.f, 777.f, 3000.f);

      // As goal keeper, expect everyone else to accept your ball
      // This prevents the goal keeper to become strike in the most random situations
      agent.disagreeOnBall = (itsBallOnField - myBallOnField).squaredNorm() > sqr(baseThreshold + (agent.disagreeOnBall ? 0.f : 222.f));
    }
  }
  if(!agent.disagreeOnBall)
    agent.timeSinceDisagreeOnBall = teamMessage.theBallModel.timeWhenLastSeen;
  agent.isUpright = teamMessage.theRobotStatus.isUpright;
  agent.timeWhenLastUpright = teamMessage.theRobotStatus.timeWhenLastUpright;
  agent.proposedTactic = teamMessage.theStrategyStatus.proposedTactic;
  agent.acceptedTactic = teamMessage.theStrategyStatus.acceptedTactic;
  agent.proposedMirror = teamMessage.theStrategyStatus.proposedMirror;
  agent.acceptedMirror = teamMessage.theStrategyStatus.acceptedMirror;
  agent.proposedSetPlay = teamMessage.theStrategyStatus.proposedSetPlay;
  agent.acceptedSetPlay = teamMessage.theStrategyStatus.acceptedSetPlay;
  agent.setPlayStep = -1;
  agent.position = teamMessage.theStrategyStatus.position;
  agent.role = teamMessage.theStrategyStatus.role;
}

void WhiteFleetStrategyBehaviorControl::updateCurrentPosition(Agent& agent)
{
  agent.currentPosition = Teammate::getEstimatedPosition(agent.lastKnownPose, agent.lastKnownTarget, agent.lastKnownSpeed, theFrameInfo.getTimeSince(agent.lastKnownTimestamp));
}
