#pragma once

#include <algorithm>
#include <cmath>

/** A gaze-only prediction is NOT a ball observation or a kick confirmation.
 * The caller may resume pursuit only using a fresh real ball and the native
 * motion/obstacle/owner/referee checks. Stored coordinates are odometric.
 */
namespace WhiteFleetPostKickReacquire
{
  struct Point {float x = 0.f, y = 0.f;};
  struct Input
  {
    unsigned now = 0, ballSeen = 0, kick = 0;
    bool allowed = false;
    float x = 0.f, y = 0.f, vx = 0.f, vy = 0.f;
    float odomX = 0.f, odomY = 0.f, odomRotation = 0.f;
    // Optional, gaze-only image evidence. Never copied into the ball model.
    bool logPolicy = false, perceptValid = false;
    bool bodyAllowed = true;
    unsigned perceptTime = 0;
    float perceptX = 0.f, perceptY = 0.f;
  };
  struct Output
  {
    bool gaze = false, stopForVision = false;
    unsigned pursuitTimestamp = 0;
    Point target;
  };

  class State
  {
  public:
    enum Mode {idle, probe, confirmed};
    Mode mode() const {return mode_;}

    void suspend(unsigned observedKick = 0)
    {
      consumedKick_ = std::max(consumedKick_, observedKick);
      mode_ = idle;
      armed_ = false;
      lastNow_ = 0;
      lastPerceptTime_ = departureTime_ = 0;
    }

    Output tick(const Input& in)
    {
      Output out;
      const bool finite = std::isfinite(in.x) && std::isfinite(in.y) &&
        std::isfinite(in.vx) && std::isfinite(in.vy) && std::isfinite(in.odomX) &&
        std::isfinite(in.odomY) && std::isfinite(in.odomRotation);
      if(!in.allowed || !finite || in.now == 0 || in.ballSeen > in.now || in.kick > in.now)
      {
        suspend(in.kick <= in.now ? in.kick : 0);
        return out;
      }
      // A missed behavior frame, clock reset or long scheduling gap cannot
      // revive a previous shooter's gaze or its pursuit permission.
      if(lastNow_ && (in.now <= lastNow_ || in.now - lastNow_ > 250))
        suspend(in.kick);
      lastNow_ = in.now;
      if(armed_ && (in.now < commandTime_ || in.now - commandTime_ > 700))
        armed_ = false;
      if(mode_ != idle && in.now >= expires_)
        mode_ = idle;

      const unsigned age = in.ballSeen ? in.now - in.ballSeen : ~0u;
      const bool fresh = age <= 150;
      const float distance = std::hypot(in.x, in.y);
      const Point ball = toOdom(in, {in.x, in.y});
      const float dx = ball.x - origin_.x, dy = ball.y - origin_.y;
      const float along = dx * std::cos(direction_) + dy * std::sin(direction_);
      const float across = -dx * std::sin(direction_) + dy * std::cos(direction_);
      const float away = distance > 1.f ? (in.vx * in.x + in.vy * in.y) / distance : 0.f;
      const bool newKick = in.kick && in.kick > consumedKick_;
      if(newKick)
        consumedKick_ = in.kick;
      const bool imageFresh = in.logPolicy && in.perceptValid && in.perceptTime &&
        in.perceptTime <= in.now && in.now - in.perceptTime <= 120 &&
        std::isfinite(in.perceptX) && std::isfinite(in.perceptY);
      const float imageDistance = imageFresh ? std::hypot(in.perceptX, in.perceptY) : 0.f;
      const bool imageUsable = imageFresh && imageDistance >= 100.f && imageDistance <= 12000.f;
      bool imageDeparture = false;
      if(imageUsable && in.perceptTime > lastPerceptTime_)
      {
        const Point image = toOdom(in, {in.perceptX, in.perceptY});
        const float ix = image.x - origin_.x, iy = image.y - origin_.y;
        const float imageAlong = ix * std::cos(direction_) + iy * std::sin(direction_);
        const float imageAcross = -ix * std::sin(direction_) + iy * std::cos(direction_);
        if(armed_ && lastPerceptTime_ && in.perceptTime - lastPerceptTime_ <= 150)
        {
          const float progress = (image.x - lastPercept_.x) * std::cos(direction_) +
                                 (image.y - lastPercept_.y) * std::sin(direction_);
          const float dt = (in.perceptTime - lastPerceptTime_) * .001f;
          // Two independent images moving away in the commanded corridor.
          // An airborne ball's ground projection is NOT a measured trajectory:
          // this witness may move the head, never authorize blind translation.
          imageDeparture = imageDistance >= 650.f && imageAlong >= 250.f &&
            std::abs(imageAcross) <= 500.f + .2f * std::max(0.f, imageAlong) &&
            progress >= 80.f && progress / dt <= 15000.f;
        }
        if(imageDeparture)
        {
          const float ray = std::atan2(iy, ix);
          gazePoint_ = {origin_.x + gazeDistance_ * std::cos(ray),
                        origin_.y + gazeDistance_ * std::sin(ray)};
          departureTime_ = in.perceptTime;
        }
        lastPercept_ = image;
        lastPerceptTime_ = in.perceptTime;
      }
      // Genuine witnesses: native contact, or a NEW observed outgoing ball
      // displaced from the recently commanded near-foot ball.
      const bool contact = newKick && armed_ && in.now - in.kick <= 250 &&
        in.kick >= nearSeen_ && (mode_ != confirmed || commandTime_ > eventTime_);
      const bool outgoing = mode_ != confirmed && armed_ && fresh &&
        in.ballSeen > nearSeen_ && distance >= 750.f && along >= 400.f &&
        std::abs(across) <= 500.f && away > 250.f;
      if(contact || outgoing)
      {
        mode_ = confirmed;
        eventTime_ = contact ? in.kick : in.ballSeen;
        expires_ = eventTime_ + 900;
      }
      else if(mode_ == idle && imageDeparture && nearSeen_ != lastImageProbeAnchor_)
      {
        mode_ = probe;
        lastImageProbeAnchor_ = nearSeen_;
        eventTime_ = in.perceptTime;
        expires_ = eventTime_ + 650;
      }
      else if(mode_ == idle && armed_ && in.ballSeen == nearSeen_ &&
              nearSeen_ != lastProbedSeen_ && age >= 100 && age <= 450)
      {
        // RL contact detection itself needs to see the departing ball. Break
        // that dependency with ONE short head probe, not a fabricated contact.
        mode_ = probe;
        lastProbedSeen_ = nearSeen_;
        eventTime_ = nearSeen_;
        expires_ = nearSeen_ + 650;
      }

      if(mode_ == idle || in.now >= expires_)
        return out;
      if(fresh && in.ballSeen > eventTime_ && distance < 650.f &&
         (!in.logPolicy || (imageUsable && in.perceptTime > eventTime_ && imageDistance < 650.f)))
      {
        // A blocked shot / ball still at the feet: align and kick normally.
        mode_ = idle;
        return out;
      }
      out.gaze = true;
      const bool modelImageDisagree = imageUsable && imageDistance >= 650.f && distance < 650.f;
      out.stopForVision = !fresh || (in.logPolicy &&
        (!in.bodyAllowed || modelImageDisagree || (departureTime_ && distance < 650.f) ||
         (mode_ == confirmed && distance < 650.f)));
      // Only a genuine witness is sent to Motion, and never while blind.
      out.pursuitTimestamp = mode_ == confirmed && fresh && !out.stopForVision ? eventTime_ : 0;
      if(fresh && distance >= 650.f)
        out.target = {in.x + .18f * in.vx, in.y + .18f * in.vy};
      else if(in.logPolicy && departureTime_)
        out.target = fromOdom(in, gazePoint_);
      else
        out.target = fromOdom(in, {origin_.x + gazeDistance_ * std::cos(direction_),
                                  origin_.y + gazeDistance_ * std::sin(direction_)});
      const float targetDistance = std::hypot(out.target.x, out.target.y);
      if(!std::isfinite(targetDistance) || targetDistance < 100.f)
      {
        mode_ = idle;
        return {};
      }
      if(targetDistance > 4500.f)
      {
        out.target.x *= 4500.f / targetDistance;
        out.target.y *= 4500.f / targetDistance;
      }
      return out;
    }

    /** Called AFTER an actual native kick/dribble request was produced in this
     * automatic-playing frame. A plan alone cannot authorize body pursuit.
     */
    void noteCommand(const Input& in, float direction, float kickLength)
    {
      // Preserve the pre-kick anchor/deadline while the old near-foot model
      // catches up. Repeated commands are not additional physical kicks.
      if(in.logPolicy && mode_ != idle)
        return;
      if(!in.allowed || in.ballSeen == 0 || in.ballSeen > in.now ||
         in.now - in.ballSeen > 120 || !std::isfinite(direction) ||
         !std::isfinite(kickLength) || !std::isfinite(in.x) || !std::isfinite(in.y) ||
         !std::isfinite(in.odomX) || !std::isfinite(in.odomY) || !std::isfinite(in.odomRotation) ||
         std::hypot(in.x, in.y) > 600.f || in.x < 70.f || std::abs(direction) > 1.1f)
        return;
      armed_ = true;
      commandTime_ = in.now;
      nearSeen_ = in.ballSeen;
      origin_ = toOdom(in, {in.x, in.y});
      direction_ = in.odomRotation + direction;
      // Requested length selects WHERE TO LOOK, never measured speed/range.
      gazeDistance_ = std::clamp(kickLength * .45f, 1500.f, 3500.f);
      if(in.logPolicy)
        departureTime_ = 0;
    }

  private:
    static Point toOdom(const Input& in, Point p)
    {
      const float c = std::cos(in.odomRotation), s = std::sin(in.odomRotation);
      return {in.odomX + c * p.x - s * p.y, in.odomY + s * p.x + c * p.y};
    }
    static Point fromOdom(const Input& in, Point p)
    {
      const float c = std::cos(in.odomRotation), s = std::sin(in.odomRotation);
      const float x = p.x - in.odomX, y = p.y - in.odomY;
      return {c * x + s * y, -s * x + c * y};
    }
    Mode mode_ = idle;
    bool armed_ = false;
    unsigned lastNow_ = 0, commandTime_ = 0, nearSeen_ = 0, consumedKick_ = 0;
    unsigned lastProbedSeen_ = 0, eventTime_ = 0, expires_ = 0;
    Point origin_;
    Point lastPercept_, gazePoint_;
    unsigned lastPerceptTime_ = 0, departureTime_ = 0, lastImageProbeAnchor_ = 0;
    float direction_ = 0.f, gazeDistance_ = 1500.f;
  };
}
