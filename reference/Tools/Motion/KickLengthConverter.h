/**
 * @file KickLengthConverter.h
 * This file contains functions to convert the kick length into the kick power
 *
 * @author Philip Reichenberg
 */
#include "Representations/Configuration/KickInfo.h"
#include "Representations/Configuration/KickLengthPair.h"
struct BallSpecification;

namespace KickLengthConverter
{
  /**
   * Converts a kick length (in mm) to a kick power (in [0, 1]). Maybe this should be done in motion instead.
   * @param kickType The requested kick type.
   * @param length The requested length.
   * @return The resulting kick power (1 for kicks with constant length).
   */
  float interpolateKickStrength(const KickInfo::KickType kickType, const float length, const KickInfo& theKickInfo, const std::vector<KickLengthPair::Pair>& kickPair);

  /**
   * Converts a kick length (in mm) to a kick power (in [0, 1]). Maybe this should be done in motion instead.
   * @param kickType The requested kick type.
   * @param length The requested length.
   * @return The resulting kick power (1 for kicks with constant length).
   */
  float kickLengthToPower(const KickInfo::KickType kickType, const float length, const KickInfo& theKickInfo, const KickLengthPair& theKickLengthPair);

  /** Read-only prediction of the EXISTING normal/Strong motion input calculation.
   * Does not change the model, rangeFactor, request or actual ball velocity.
   */
  float requestedBallVelocity(KickInfo::KickType type, float length, const KickInfo& info,
    const BallSpecification& ball, const Rangef& actualLimits);
  /** Native rolling-model time [s]. 1mm endpoint tolerance is numerical only. */
  float requestedFlightTime(float speed, float distance, const BallSpecification& ball);
}
