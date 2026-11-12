/**
 * @file SkillRequest.h
 *
 * This file declares the representation of the request from the strategy layer to the skill layer.
 *
 * @author Arne Hasselbring
 */

#pragma once

#include "Math/Pose2f.h"
#include "Streaming/AutoStreamable.h"
#include "Streaming/Enum.h"

STREAMABLE(SkillRequest,
{
  /** When adding a type for playing the ball, add them to
   * - HandlePenaltyKick
   * - HandleStrikerLostBall
   * - SkillBehaviorControl
   * - PlayBall
   */
  ENUM(Type,
  {,
    none,
    stand,
    walk,
    shoot,
    pass,
    dribble,
    block,
    mark,
    observe,
    clear,
    waitShoot,
  });

  struct Builder
  {
    // All coordinates/angles/directions are in field coordinates.
    static SkillRequest empty();
    static SkillRequest stand();
    static SkillRequest walkTo(const Pose2f& target);
    static SkillRequest shoot();
    static SkillRequest passTo(int target);
    static SkillRequest dribbleTo(Angle target);
    static SkillRequest block(const Vector2f& player);
    static SkillRequest mark(const Vector2f& player);
    static SkillRequest observe(const Vector2f& point);
    static SkillRequest clear();
    static SkillRequest waitShoot(int target);
  },

  (Type)(none) skill, /**< The skill that shall run. */
  (Pose2f) target, /**< The target pose, object or direction (in field coordinates). */
  (int)(-1) passTarget, /**< The number of the passed-to player. */
  (bool)(false) closeSetPlay, /**< Scoped close-restart proposal, not ball permission. */
  (bool)(false) closeSetPlayCancelled, /**< Explicit abort; never reinterpret the same close intent as a long pass. */
  (bool)(false) closeSetPlayPaused, /**< Bounded observation/reconfirmation pause; no kick permission. */
  (bool)(false) closeSetPlayFallbackAllowed, /**< Cancellation may use native clear only with valid current ball geometry. */
  (Vector2f)(Vector2f::Zero()) closeSetPlayTarget, /**< Proposed field target; native pass evaluation publishes the final target. */
  (unsigned)(0) closeSetPlayEpoch, /**< Prevent proposals surviving a restart change. */
  (bool)(false) priorityPositioning, /**< Use existing fast walking for distant, clear positioning only. */
  (bool)(false) backlineCover, /**< Qualified local backline walk or safe stand; no ball ownership or kick permission. */
  (bool)(false) defensiveContest, /**< V7 local own-penalty clear intent, never a global ball-owner vote. */
  (bool)(false) defensiveContestSafety, /**< Keep robot obstacles for the scoped GK/defender pair, including native requests. */
  (unsigned)(0) defensiveContestEpoch, /**< Ordinary-PLAY epoch binding; old requests cannot retain extra permission. */
  (int)(-1) defensiveContestDefender, /**< Event-held single field participant; independent of active-role churn. */
  (bool)(false) kickoffThrough, /**< Local V8 contract only; not a wire field or ball permission. */
  (bool)(false) kickoffThroughRetired, /**< Finite receiver tombstone, only suppresses its old same-endpoint receive intent. */
  (int)(0) kickoffThroughAction,
  (int)(-1) kickoffThroughKicker,
  (int)(-1) kickoffThroughReceiver,
  (unsigned)(0) kickoffThroughEvent,
  (Vector2f)(Vector2f::Zero()) kickoffThroughTarget,
  (Vector2f)(Vector2f::Zero()) kickoffThroughBodyTarget,
  (Vector2f)(Vector2f::Zero()) kickoffThroughBall,
  (bool)(false) kickoffThroughBallKnown,
  (float)(0.f) kickoffThroughLength,
  (bool)(false) kickoffThroughPublish,
  (bool)(false) restartPassInFlight, /**< Local plan intent, never a touch/ball measurement. */
  (int)(-1) restartPassReceiver,
  (Vector2f)(Vector2f::Zero()) restartPassTarget,
  (unsigned)(0) restartPassEpoch,

  (bool)(false) restartPassRequired, /**< Local restart contract, not a new rule permission. */
  (bool)(false) restartPassPreparing, /**< Announce/navigation only; never permits a strike. */
  (bool)(false) restartPassGoalKick,
  (int)(-1) restartPassPasser,
  (int)(0) restartPassReason, /**< MatchTacticalRepair::RestartReason, actual same-frame decision. */
  (float)(-1.f) restartPassSafety, /**< Native pass risk, -1 when not evaluated. */
  (unsigned)(0) restartPassReadySince, /**< Fixed first close/ready strike-budget start. */
  (bool)(false) restartPassTargetValid, /**< Explicit local goal-kick proposal; zero field coordinates are not a sentinel. */
  (unsigned)(0) restartPassStrikeElapsed, /**< Local budget diagnostic, not a physical execution/touch claim. */
  (int)(0) restartPassNativeStage, /**< Local native pass gate stage; not in teamMessage.def. */
  (bool)(false) isolatedKickoffFinished,
  (unsigned)(0) isolatedKickoffEvent,
  (unsigned)(0) kickoffFixedRunFrame, /**< Same-Cognition active Session witness only, not a ball observation or wireless field. */
  (unsigned)(0) kickoffFixedRunRelease, /**< Original immutable Session nativePursuitAt, never renewed by retained data. */
  (bool)(false) kickoffThroughHasLaunch, /**< Local actual contract origin; zero is a valid coordinate, never a sentinel. */
  (Vector2f)(Vector2f::Zero()) kickoffThroughLaunch,
  (bool)(false) kickoffThroughAimValid, /**< Local same-update qualification, not new wireless permission. */
  (Vector2f)(Vector2f::Zero()) kickoffThroughAimTargetOdometry,
  (unsigned)(0) kickoffThroughAimFrame,
  (unsigned)(0) kickoffThroughPostureFrame, /**< Same-Cognition receiver-only contact witness; never transmitted. */
  (unsigned)(0) kickoffThroughPostureSince, /**< Fixed first warning in this event, never refreshed by observations. */
  (bool)(false) kickoffThroughGapValid, /**< Local same-frame walking-only witness, never wireless or kicking permission. */
  (unsigned)(0) kickoffThroughGapSeen,
  (unsigned)(0) kickoffThroughGapFrame,
  (Vector2f)(Vector2f::Zero()) kickoffThroughGapBallOdometry,
  (Vector2f)(Vector2f::Zero()) kickoffThroughGapTargetOdometry,
  (bool)(false) backfieldStrongOutlet, /**< Ordinary safe front supply/clear, not another owner or wire field. */
  (bool)(false) mdfzRule, /**< MDFZ arena: this ball action comes from the class's own rule (no duel/outlet override). */


});
