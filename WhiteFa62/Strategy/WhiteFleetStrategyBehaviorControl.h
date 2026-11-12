/**
 * @file WhiteFleetStrategyBehaviorControl.h
 *
 * This file declares a module that determines the strategy of the team.
 *
 * @author Arne Hasselbring
 */

#pragma once
#include <string>
#include <vector>

#include "WhiteFleetBehavior.h"
#include "WhiteFleetKickoffThroughV8CandidateAudit.h"
#include "Representations/BehaviorControl/AgentStates.h"
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/FieldInterceptBall.h"
#include "Representations/BehaviorControl/ExpectedGoals.h"
#include "Representations/BehaviorControl/InterceptBallInformation.h"
#include "Representations/BehaviorControl/IndirectKick.h"
#include "Representations/BehaviorControl/PassEvaluation.h"
#include "Representations/BehaviorControl/PassTargetEvaluation.h"
#include "Representations/BehaviorControl/SkillRequest.h"
#include "Representations/BehaviorControl/StrategyStatus.h"
#include "Representations/Communication/ReceivedTeamMessages.h"
#include "Representations/Communication/TeamData.h"
#include "Representations/Communication/SentTeamMessage.h"
#include "Representations/Configuration/BehaviorParameters.h"
#include "Representations/Configuration/BallSpecification.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Configuration/SetupPoses.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameState.h"
#include "Representations/Modeling/BallModel.h"
#include "Representations/Modeling/GlobalOpponentsModel.h"
#include "Representations/Modeling/ObstacleModel.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Modeling/TeamBallModel.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"
#include "Representations/MotionControl/OdometryData.h"
#include "Representations/MotionControl/WalkingEngineOutput.h"
#include "Representations/Sensing/FallDownState.h"
#include "Tools/BehaviorControl/Strategy/Agent.h"
#include "WhiteFleetMatchTacticalRepair.h"
#include "WhiteFleetKickoffThroughV8Session.h"
#include "WhiteFleetC10KickoffPhase.h"
#include "WhiteFleetFleetProgressPolicy.h"
#include "WhiteFleetGoalKickExecution.h"
#include "Framework/Module.h"
#include <vector>

MODULE(WhiteFleetStrategyBehaviorControl,
{,
  REQUIRES(BallModel),
  REQUIRES(BallSearchParticles),
  REQUIRES(BallSpecification),
  REQUIRES(BehaviorParameters),
  REQUIRES(ExtendedGameState),
  REQUIRES(ExpectedGoals),
  REQUIRES(FallDownState),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(FieldInterceptBall),
  REQUIRES(FrameInfo),
  REQUIRES(GameState),
  REQUIRES(GlobalOpponentsModel),
  REQUIRES(IndirectKick),
  REQUIRES(InterceptBallInformation),
  REQUIRES(KickInfo),
  REQUIRES(MotionInfo),
  USES(MotionRequest), // Previous final SBC output only; REQUIRES would introduce a SkillRequest cycle.
  REQUIRES(ObstacleModel),
  REQUIRES(OdometryData),
  REQUIRES(PassEvaluation),
  REQUIRES(PassTargetEvaluation),
  REQUIRES(ReceivedTeamMessages),
  REQUIRES(TeamData),
  REQUIRES(RobotPose),
  REQUIRES(SentTeamMessage),
  REQUIRES(SetupPoses),
  REQUIRES(TeamBallModel),
  REQUIRES(WalkingEngineOutput),
  PROVIDES(SkillRequest),
  REQUIRES(SkillRequest),
  PROVIDES(StrategyStatus),
  PROVIDES(AgentStates),
  LOADS_PARAMETERS(
  {,
    (Strategy::Type) strategy, /**< The strategy to play. */
  }),
});

class WhiteFleetStrategyBehaviorControl : public WhiteFleetStrategyBehaviorControlBase
{
public:
  /** Constructor. */
  WhiteFleetStrategyBehaviorControl();

  /**
   * Creates extended module info (union of this module's info and requirements of other behavior parts).
   * @return The extended module info.
   */
  static std::vector<ModuleBase::Info> getExtModuleInfo();

private:
  /**
   * Updates the skill request.
   * @param skillRequest The provided skill request.
   */
  void update(SkillRequest& skillRequest) override;

  /**
   * Updates the strategy status.
   * @param strategyStatus The provided strategy status.
   */
  void update(StrategyStatus& strategyStatus) override { strategyStatus = theStrategyStatus; }

  /**
   * Updates the agent states.
   * @param agentStates The provided agent states.
   */
  void update(AgentStates& agentStates) override { agentStates.agents = agents; }

  /**
   * Updates the list of agents to represent the most recent data.
   * @return A pointer to the agent that represents this player.
   */
  Agent* updateAgents();

  /**
   * Updates an agent using local representations.
   * @param agent The agent to update.
   */
  void updateAgentBySelf(Agent& agent);

  /**
   * Updates an agent using a team message.
   * @param agent The agent to update.
   * @param teamMessage The team message to incorporate.
   */
  void updateAgentByTeamMessage(Agent& agent, const ReceivedTeamMessage& teamMessage);

  /**
   * Updates the estimated position of the agent.
   * @param agent The agent to update.
   */
  void updateCurrentPosition(Agent& agent);

  WhiteFleetBehavior theBehavior; /**< The instance of the behavior. */
  std::vector<Agent> agents; /**< The list of active agents. */
  StrategyStatus theStrategyStatus; /**< The strategy status which is provided later. */
  bool wasPenalized = false;
  bool recoveryScreen = false;
  unsigned recoveryScreenSince = 0;
  unsigned screenCooldownUntil = 0;
  int counterOutletSide = 0;
  unsigned counterOutletSideSince = 0;
  void applyDefenseCounter(SkillRequest& request, const Agent& self);
  bool applyMatchRestart(SkillRequest& request, const Agent& self);
  bool allowKickoffReceiverContactPosture(const WhiteFleetKickoffThroughV8::Params& params, const Agent& self);
  bool runIsolatedKickoff(SkillRequest& request, Agent& self);
  bool applyKickoffThrough(SkillRequest& request, const Agent& self);
  bool applyBackfieldOutlet(SkillRequest& request, const Agent& self);
  void applyOpponentSidePosition(SkillRequest& request, const Agent& self);
  bool applyMdfzFormation(SkillRequest& request, const Agent& self);
  bool applyMdfzRules(SkillRequest& request, const Agent& self); // MDFZ arena: class rules (mdfzRules.cfg)
  struct MdfzCond {int name=0;bool less=true;float value=0.f;};
  struct MdfzRule {bool ball=true;int action=0;float a=0.f,b=0.f;std::vector<MdfzCond> conds;std::string text;};
  struct {std::vector<MdfzRule> rules;int active=-1;unsigned since=0,lastLog=0;unsigned fired[16]={0};} mdfzRules; // MDFZ arena: class formation (mdfzFormation.cfg)
  struct {bool enabled=false;float followBall=.3f;unsigned lastLog=0,lastStats=0;unsigned n[8]={0};} mdfzFormation; // n: applied,notPlaying,keeper,active,setPlay,skill,passTarget,outlet
  struct OutletPlan {unsigned born=0,issued=0;int receiver=-1;Vector2f target=Vector2f::Zero();} outletPlan;
  WhiteFleetFleetProgressPolicy::Half supportHalf;
  unsigned lastSupportLog=0,lastOutletLog=0;
  WhiteFleetC10Kickoff::Phase isolatedKickoffPhase;
  WhiteFleetKickoffThroughV8::Session kickoffThroughSession;
  WhiteFleetKickoffThroughV8::CandidateAuditLatch c15CandidateAuditLatch;
  WhiteFleetC15RollingRendezvous::MotionMemory c15RaceMotion;
  unsigned kickoffThroughEvent=0,kickoffThroughLastLog=0;
  int kickoffThroughLastAction=-1;
  struct {unsigned event=0,since=0,lastLog=0,logs=0;bool blocked=false;} receiverContactPosture;
  unsigned c3WitnessEvent=0,c3WitnessCount=0; // Local diagnostic only; hard32/event, no wire/state permission.
  struct CornerPlan
  {
    unsigned started=0,epoch=0,inFlightUntil=0,readySince=0,issued=0,lastIntent=0,unconfirmedAt=0;
    int passer=-1,receiver=-1;
    Vector2f target=Vector2f::Zero(),origin=Vector2f::Zero();
    bool selected=false,retired=false,complete=false,goalKick=false,departed=false,originKnown=false;
    WhiteFleetMatchTacticalRepair::RestartAnchor anchor;
    WhiteFleetGoalKickExecution::StrikeClock strikeClock;
    unsigned proposalAt=0,commandAt=0;
    float commandDirection=0.f,commandLength=0.f;
    Vector2f yieldTarget=Vector2f::Zero();
    unsigned yieldSeen=0;
    unsigned lastPhaseDiagnostic=0;
    int lastPhaseKey=-1;
  } cornerPlan;
  unsigned goalKickFallbackLog=0;
  static constexpr unsigned goalKickFallbackMs=8000; /**< Own goal kick: keeper plays the ball itself after this long without a contract strike. */
  static constexpr unsigned goalKickPassMs=3000; /**< Fallback: aimed pass to the front forward for this long, then a long forward kick. */
  static constexpr int goalKickFrontForward=3; /**< Own goal kick: this field player stays in the opponent half as the outlet. */
  static constexpr float goalKickFrontX=1500.f,goalKickFrontY=2000.f; /**< Outlet point (x into the opponent half, |y| on the ball-far side). */
  unsigned lastTeamActivityDiagnostic = 0;
};
