#pragma once
#include <algorithm>
#include <cmath>
#include <array>
#include <vector>

/** Local observation evidence, not a foot/contact or scoring certificate.
 * No GameState/BallModel/RobotPose writes, global pose, GT or wire fields. */
namespace WhiteFleetOwnKickoffRelease
{
  struct Point{float x=0.f,y=0.f;};
  struct Pose{float x=0.f,y=0.f,angle=0.f;};
  inline bool finite(Point p){return std::isfinite(p.x)&&std::isfinite(p.y);}
  inline bool finite(Pose p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.angle);}
  inline float distance(Point a,Point b){return std::hypot(a.x-b.x,a.y-b.y);}
  inline Point transform(Pose o,Point b)
  {return{o.x+std::cos(o.angle)*b.x-std::sin(o.angle)*b.y,o.y+std::sin(o.angle)*b.x+std::cos(o.angle)*b.y};}
  inline bool fresh(unsigned now,unsigned stamp,unsigned age)
  {return stamp>0&&stamp<=now&&now-stamp<=age;}
  struct Input
  {
    struct Peer{unsigned number=0,frame=0,seen=0,kick=0;Point velocity{};};
    unsigned now=0,event=0,seen=0,kick=0;
    bool enabled=false,legal=false,healthy=false;
    Pose odometry{};Point percept{};
    float variance=0.f,tolerance=300.f;
    std::vector<Peer> peers;
  };
  struct State
  {
    unsigned event=0,lastTime=0,lastSeen=0,stationary=0,shifted=0,shiftedSeen=0;
    struct PeerTrack{unsigned frame=0,seen=0,kick=0;float speed=0.f,travel=0.f;unsigned samples=0;};
    std::array<PeerTrack,21> peerTracks{};
    Pose lastOdometry{};Point anchor{},anchorPercept{},previousShift{};
    float anchorVariance=0.f,movement=0.f,required=0.f;
    bool anchored=false,ready=false,usedKick=false;
    const char* reason="not_enabled";
    void clear(unsigned epoch=0)
    {*this=State{};event=epoch;}
    bool update(const Input& i)
    {
      if(!i.enabled||!i.legal||!i.healthy||!i.event||i.event>i.now)
      {clear(i.event);reason=!i.enabled?"not_enabled":!i.legal?"not_legal":"not_healthy";return false;}
      if(event!=i.event)clear(i.event);
      if(!finite(i.odometry)||!std::isfinite(i.tolerance)||i.tolerance<300.f)
      {clear(i.event);reason="nonfinite_evidence";return false;}
      if(lastTime)
      {
        const unsigned elapsed=i.now>=lastTime?i.now-lastTime:0;
        const float turn=std::abs(std::remainder(i.odometry.angle-lastOdometry.angle,6.283185307f));
        if(!elapsed||elapsed>500u||distance({i.odometry.x,i.odometry.y},{lastOdometry.x,lastOdometry.y})>200.f+3.f*static_cast<float>(elapsed)||
           turn>.15f+.003f*static_cast<float>(elapsed))
        {clear(i.event);lastTime=i.now;lastOdometry=i.odometry;reason="odometry_discontinuity";return false;}
      }
      lastTime=i.now;lastOdometry=i.odometry;
      bool peerMoved=false;
      for(const auto& p:i.peers)
      {
        if(p.number==0||p.number>=peerTracks.size()||!fresh(i.now,p.frame,1200u)||p.frame<i.event||
           !fresh(i.now,p.seen,250u)||p.seen<i.event||p.seen>p.frame||!finite(p.velocity))continue;
        const float speed=std::hypot(p.velocity.x,p.velocity.y);
        auto& track=peerTracks[p.number];
        if(p.frame>track.frame&&p.seen>track.seen)
        {
          if(track.seen&&p.seen>track.seen&&p.seen-track.seen<=500u&&speed>=i.tolerance&&track.speed>=i.tolerance)
          {track.travel+=std::min(speed,track.speed)*static_cast<float>(p.seen-track.seen)*.001f;++track.samples;}
          else {track.travel=0.f;track.samples=1u;}
          track.frame=p.frame;track.seen=p.seen;track.speed=speed;track.kick=p.kick;
        }
        // The velocity is the peer's own visual/ODO filter, explicitly sent on
        // the existing wire. No global RobotPose or untransmitted covariance.
        peerMoved|=track.samples>=2u&&track.travel>i.tolerance&&fresh(i.now,p.kick,1200u)&&p.kick>event&&p.kick<=p.frame;
      }
      if(!fresh(i.now,i.seen,250u)||i.seen<i.event||!finite(i.percept)||!std::isfinite(i.variance)||i.variance<0.f)
      {shifted=0;shiftedSeen=0;ready=peerMoved;reason=peerMoved?"fresh_peer_kick_and_visual_motion":"no_fresh_own_percept";return ready;}
      const Point observed=transform(i.odometry,i.percept);
      if(!finite(observed)){clear(i.event);reason="nonfinite_observation";return false;}
      if(!anchored)
      {anchor=observed;anchorPercept=i.percept;anchorVariance=i.variance;anchored=true;lastSeen=i.seen;
       required=i.tolerance+3.f*std::sqrt(2.f*i.variance);
       ready=peerMoved;reason=peerMoved?"fresh_peer_kick_and_visual_motion":"anchor";return ready;}
      if(i.seen>lastSeen)
      {
        lastSeen=i.seen;movement=distance(observed,anchor);
        const float uncertainty=3.f*std::sqrt(anchorVariance+i.variance);
        required=i.tolerance+uncertainty;
        const Point delta{observed.x-anchor.x,observed.y-anchor.y};
        if(movement<=i.tolerance*.5f+uncertainty)
        {stationary=std::min(3u,stationary+1u);shifted=0;shiftedSeen=0;}
        else if(movement>required)
        {
          const float previousLength=std::hypot(previousShift.x,previousShift.y);
          const bool coherent=!shifted||previousLength<=1.f||
              delta.x*previousShift.x+delta.y*previousShift.y>=.8f*movement*previousLength;
          shifted=coherent?std::min(3u,shifted+1u):1u;previousShift=delta;shiftedSeen=i.seen;
        }
        else {shifted=0;shiftedSeen=0;}
      }
      usedKick=fresh(i.now,i.kick,1200u)&&i.kick>event;
      const bool recentShift=fresh(i.now,shiftedSeen,250u);
      const float uncertainty=3.f*std::sqrt(anchorVariance+i.variance);
      const float rawChange=distance(i.percept,anchorPercept);
      const bool paired=usedKick&&shifted>=2u&&recentShift&&rawChange>i.tolerance*.5f;
      // Packet-loss fallback needs an observed stationary reference followed by
      // THREE coherent raw observations and twice the original300mm distance.
      const bool ownOnly=stationary>=2u&&shifted>=3u&&recentShift&&movement>2.f*i.tolerance+uncertainty&&rawChange>i.tolerance;
      ready=paired||ownOnly||peerMoved;
      reason=paired?"fresh_kick_and_own_motion":ownOnly?"own_stationary_then_clear_motion":peerMoved?"fresh_peer_kick_and_visual_motion":
          shifted?"waiting_consistent_motion_or_kick":"waiting_motion";
      return ready;
    }
  };
}
