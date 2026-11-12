/**
 * @file Representations/Configuration/BehaviorParameters/BehaviorParameters.h
 *
 * The file declares a struct that contains frequently used parameters of the behavior which can be modified.
 *
 * @author Andreas Stolpmann
 */

#pragma once

#include "Math/Angle.h"

STREAMABLE(BehaviorParameters,
{,
  (Angle) penaltyStrikerAngleToLeftPostOffset,  /**< (the greater the value, the closer to the center the robot will shoot) */
  (Angle) penaltyStrikerAngleToRightPostOffset,  /**< (the greater the value, the closer to the center the robot will shoot) */
  (bool) penaltyStrikerUseObstacles, /**< Whether the penalty striker should use the obstacle model to kick in the more free corner */

  (Angle) defaultLookDownAngle, /**< Use this default head tilt */

  (float) ballCatchMaxWalkDistance,
  (float) genuflectRadius, /**< The range that is covered with the penalty keeper genuflect. */
  (float) illegalAreasSmallDistance, /**< To enter illegal state use this smaller value. */
  (float) illegalAreasLargeDistance, /**< To leave illegal state use this higher value. */

  (int) noCommunicationAfterUnpenalizedTime, /**< Prevent communication when freshly unpenalized. */
  (int) noSkillRequestAfterUnpenalizedTime, /**< Prevent behavior calulcation when freshly unpenalized. Should be bigger than noCommunicationAfterUnpenalizedTime. */
  (int) lookAroundAfterUnpenalizedTime, /**< Look around after unpenalized. Should be bigger than lookAroundAfterUnpenalizedTime. */

  (int) ballSeenThreshold, /**< If the ball is older than this [ms], an agent can not become active (it can stay active, though, if it already was). */
  (int) ballLostThreshold, /**< If the ball is older than this [ms], an agent can not be active (even if it was). */
  (int) ballSearchThresholdNoTeam, /**< The time after the ball was last seen to start searching for it, when the robot has no team and is playing alone. */
  (int) ballSearchThresholdTeam, /**< The time after the ball was last seen to start searching for it, when the robot has team and is not playing alone. */
  (bool)(false) requireFreshTeamActivity, /**< During PLAY, do not reserve roles for never-heard or expired peers. Optional incomplete-roster policy. */
  (unsigned)(10000) teamActivityTimeout, /**< Real peer-message lifetime [ms], clamped to 5..60 seconds. Not a local-ball or GC timeout. */
  (bool)(true) matchTacticalRepair, /**< Requires the existing identity-bound V2.1 profile; no hardware authorization. */
  (float)(500.f) keeperPursuitOutsidePenalty, /**< Absolute outer chase edge beyond the penalty rectangle [mm]. */
  (float)(200.f) keeperPursuitBuffer, /**< Inward start/deceleration buffer, not an extra outward allowance [mm]. */
  (float)(300.f) kickoffCornerOutside, /**< Outside the opponent penalty corner [mm], clipped to the field. */

  (bool)(false) c15Kickoff, /**< Dedicated C15 3->2/right kickoff; still requires the identity-bound profile. */

  (bool)(false) c15EarlyStart, /**< Legal own-half preload only, never early BallFree. */
  (bool)(false) c15CandidateDiagnostics, /**< Local bounded candidate/rollout logs only; no behavior permission. */
  (bool)(false) c15RestoreProvenPool, /**< Same C15 entry: prefer safe deep points, otherwise existing tested C14 pool. */
  (bool)(false) c15DefenceAdaptiveAngle, /**< Bound contour-derived pre-offer rays; endpoint, normal kick mapping and receiver run remain one contract. */
  (bool)(false) c15InsideCornerLane, /**< Low defender: guarded inner route to the real same-flank penalty corner, before offer only. */
  (bool)(false) c15RaceRendezvous, /**< Explicit rolling first-hop, stand-observe native release and calibrated interception race. */
  (bool)(false) c15DirectCenterKickoff, /**< Independent finite centre PASS + locked-point run/receive/shot trial. Default OFF. */
  (bool)(false) c15CenterFastKickoff, /**< C2 finite right/deep trial, native release before running, normal-cap/justHit. */
  (bool)(false) c15ReceiverPrepareHold, /**< C3 receiver only: keep actual Stand within existing safe150mm prepare domain. */
  (bool)(false) c15ReceiverBallApproach, /**< C4 receiver movement-only approach before the unchanged capture/shot gates. */
  (bool)(false) c15ReceiverFollowFromRelease, /**< C4 follow v2: body follows fresh ball from the first legal run; no blind fixed-P fallback. */
  (bool)(false) c15StableKickAim, /**< Qualified short ODOM aim plus native normal precision for this first pass only. */
  (bool)(false) c15ReceiverEarlyNative, /**< Receiver-only native pursuit after legal PLAY and fresh observed ball motion. */
  (bool)(false) c15ReceiverGapWalk, /**< Bounded far-ball walking through a short visual gap; never stale native shooting. */
  (bool)(false) c15FirstPassClearance, /**< Prefer existing corridor reserve among already-admitted first-pass plans only. */
  (bool)(false) c15ForwardPrepare, /**< Qualified own-kickoff READY preparation advance; actual peer and original protection remain authoritative. */
  (bool)(false) c15EarlyChannelReceive, /**< Isolated receiver-only early shoulder channels; OFF keeps the exact parent strategy. */
  (bool)(false) c15ReceiverLocalStep, /**< Only with early channels: future-leg occupancy/whole-route ETA are soft; first native leg remains hard. */
  (bool)(false) c15ReceiverLowerChannel, /**< Isolated canonical-field -Y shoulder commitment; requires early-channel/local-step profile. */
  (bool)(false) c15ReceiverShortestIntercept, /**< Mutually exclusive with lower-channel: native shortest forecast route, not ordinary chase. */
  (bool)(false) c15ReceiverFixedDirect, /**< Isolated fixed FIELD receiving reference; tactical detours disabled, all actual safety retained. */
  (bool)(false) c15ReceiverFixedRunToArrival, /**< Only authenticated fixed RUN ignores unavailable ball; ARRIVED requires new real sight before near play. */
  (bool)(false) c15ReceiverBoundedFastRun, /**< Fixed RUN only: independent pose commitment and short-lived native velocity with unchanged checked endpoint. */
  (bool)(false) c15ReceiverCoordinateRun, /**< Explicit split ordinary-ODO/Preview certificate; no monotonic-distance requirement. */
  (bool)(false) c15ReceiverSimMaxRun, /**< SIM ONLY red2 ablation: no tactical/collision/arm/near-point slowdown during fixed RUN. Never physical. */
  (bool)(false) c15ReceiverSimFlow, /**< SIM ONLY red2: legal earlier prepare and one-way fresh1m native approach. Requires sim max-run. */
  (bool)(false) c15ReceiverPhysicalFlow, /**< New identity-bound physical entry only: guarded native navigation and fresh1m handoff; no SIM ablation. */
  (float)(6940.f) c15FixedReceiveBallX, /**< Receiving BALL reference only [mm], not a ball observation or kick permission. */
  (float)(-1050.f) c15FixedReceiveBallY,
  (float)(6650.f) c15ShortestReceiveMinX, /**< Preferred receiving BALL rectangle [mm], team field coordinates, never a hard waiting region. */
  (float)(7250.f) c15ShortestReceiveMaxX,
  (float)(-1300.f) c15ShortestReceiveMinY,
  (float)(-800.f) c15ShortestReceiveMaxY,
  (bool)(false) backfieldStrongOutlet, /**< Safe forward field-player supply in ordinary own-half play. */
  (bool)(false) opponentSidePosition, /**< Non-ball-role soft preference using fresh visible opponents. */


});
