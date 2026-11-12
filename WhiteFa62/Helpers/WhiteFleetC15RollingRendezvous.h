#pragma once
#include "WhiteFleetKickoffThroughV8Math.h"
#include <array>

/** Same 3->2 contract, explicitly different FIRST-hop geometry: receive a
 * rolling normal-model ball on the same flank, then native continuation.
 * This is a calibrated tactical risk estimate, not a guarantee against an
 * instantaneous omnidirectional maximum-speed adversary. Original short
 * max-speed clearance, native risks, capture radii and referee gates remain.
 * No opponent internal command, identity, simulator coordinate or GT is used.
 */
namespace WhiteFleetC15RollingRendezvous
{
  namespace K=WhiteFleetKickoffThroughV8;
  constexpr float wireTolerance=300.f,temporalMargin=.10f,maxControlHorizon=2.25f;
  constexpr float calibratedSpeed=1500.f; // Old7 cases: body peak<=1149mm/s over120ms; explicit +30% margin.
  constexpr float calibratedNormalLimit=3400.f; // Frozen existing normal model/config, not a changed actuator limit.

  inline bool enabled(const K::Params& p)
  {return p.raceRendezvous&&p.nativeReceive&&p.isolated&&p.playbook&&p.calibrated&&p.adaptiveMeet&&p.side==-1&&K::paramsValid(p);}

  inline std::vector<K::Point> targets(const K::Field& field,const K::Params& p)
  {
    std::vector<K::Point> out;
    if(!enabled(p)||!K::fieldValid(field))return out;
    const auto prep=K::preparePoint(field,p,-1);
    for(float x:{450.f,550.f,650.f})for(float dy:{0.f,-100.f,100.f})out.push_back({x,prep.y+dy});
    return out;
  }
  inline bool domain(K::Point ball,K::Point target,const K::Field& field,const K::Params& p,float tolerance=0.f)
  {
    if(!enabled(p)||!K::fieldValid(field)||!K::finite(ball)||!K::finite(target)||
       !std::isfinite(tolerance)||tolerance<0.f||tolerance>wireTolerance||
       !K::targetSafe(ball,target,field,p,-1))return false;
    const auto prep=K::preparePoint(field,p,-1);
    return target.x>=450.f-tolerance&&target.x<=650.f+tolerance&&
        std::abs(target.y-prep.y)<=100.f+tolerance&&target.x-ball.x>250.f&&target.y<0.f&&target.y-ball.y<-250.f;
  }
  inline bool endpoint(K::Point target,const K::Field& field,const K::Params& p,float tolerance=0.f)
  {return domain({0.f,0.f},target,field,p,tolerance);} // Endpoint membership, not a fabricated current-ball observation.
  inline bool approachingOwnContract(K::Point ball,K::Point launch,K::Point target)
  {
    if(!K::finite(ball)||!K::finite(launch)||!K::finite(target))return false;
    const float dx=target.x-launch.x,dy=target.y-launch.y,length=std::hypot(dx,dy);
    if(length<=250.f)return false;
    const float along=((ball.x-launch.x)*dx+(ball.y-launch.y)*dy)/length;
    const float cross=std::abs((ball.x-launch.x)*dy-(ball.y-launch.y)*dx)/length;
    return along>300.f&&along<=length+1500.f&&cross<=900.f; // Existing movement/tail/cross-track sizes, not a capture certificate.
  }

  /** Bounded position-history qualification in the robot's OWN field estimate.
   * Cold/stale/jumped/unidentified tracks never get the nominal calibration.
   * A measured faster mover raises the estimate, up to the original2200 cap.
   */
  struct MotionMemory
  {
    struct Track {K::Point anchor{},last{};unsigned born=0,stamp=0,anchorTime=0,count=0;float peak=0.f;};
    std::array<Track,8> tracks{};unsigned event=0,lastTime=0;
    void reset(unsigned nextEvent){*this=MotionMemory();event=nextEvent;}
    void observe(unsigned now,unsigned nextEvent,const std::vector<K::Point>& points)
    {
      if(nextEvent!=event||now<lastTime)reset(nextEvent);
      lastTime=now;std::array<bool,8> used{};
      for(auto point:points)
      {
        if(!K::finite(point))continue;
        int best=-1;float dist=500.f;
        for(unsigned i=0;i<tracks.size();++i)if(!used[i]&&tracks[i].stamp&&now>=tracks[i].stamp&&now-tracks[i].stamp<=650)
        {const float d=K::distance(point,tracks[i].last);if(d<dist){dist=d;best=static_cast<int>(i);}}
        if(best<0)
        {
          for(unsigned i=0;i<tracks.size();++i)if(!used[i]&&(!tracks[i].stamp||now<tracks[i].stamp||now-tracks[i].stamp>650)){best=static_cast<int>(i);break;}
          if(best<0)continue;
          tracks[best]={point,point,now,now,now,1,0.f};used[best]=true;continue;
        }
        auto& track=tracks[best];used[best]=true;
        if(now==track.stamp)continue;
        if(now-track.anchorTime>=100)
        {
          const float speed=K::distance(point,track.anchor)/((now-track.anchorTime)*.001f);
          track.peak=std::max(track.peak,speed);track.anchor=point;track.anchorTime=now;
        }
        track.last=point;track.stamp=now;++track.count;
      }
    }
    float speed(unsigned now,K::Point point)const
    {
      for(const auto& track:tracks)
        if(track.stamp==now&&track.count>=3&&now>=track.born&&now-track.born>=250&&
           K::distance(point,track.last)<=1.f&&std::isfinite(track.peak)&&track.peak<=2200.f)
          return std::min(2200.f,std::max(calibratedSpeed,track.peak+250.f));
      return -1.f;
    }
  };

  inline float temporalSpeed(const K::Obstacle& o)
  {
    return std::isfinite(o.calibratedMotionSpeed)&&o.calibratedMotionSpeed>=calibratedSpeed?
        std::min(o.speedBound,o.calibratedMotionSpeed):o.speedBound;
  }

  struct PrefixAssessment
  {
    bool valid=false,hardConflict=false;unsigned mobileChecks=0;
    float minGap=INFINITY,hardClearance=INFINITY;
    const char* reason="race_invalid_prefix";
  };
  /** Independent production time gate. It intentionally knows nothing about
   * the endpoint table or receiver staging: an old dangerous ray can be
   * rejected by TIME itself, not merely by the new rendezvous domain. */
  template<class BallAt>
  PrefixAssessment assessPrefix(const std::vector<K::Obstacle>& obstacles,const K::Params& p,
                                float ballRadius,float controlTime,BallAt ballAt)
  {
    PrefixAssessment out;
    if(!enabled(p)||!std::isfinite(ballRadius)||ballRadius<=0.f||!std::isfinite(controlTime)||
       controlTime<=0.f||controlTime>maxControlHorizon+.001f)return out;
    for(const auto& o:obstacles)
      if(!K::finite(o.center)||!K::finite(o.left)||!K::finite(o.right)||!std::isfinite(o.radius)||o.radius<0.f||
         !std::isfinite(o.speedBound)||o.speedBound<0.f)return out;
    auto previous=ballAt(0.f);if(!K::finite(previous))return out;
    for(float elapsed=.025f;elapsed<=controlTime+.025f;elapsed+=.025f)
    {
      const float t=std::min(elapsed,controlTime);const auto point=ballAt(t);
      if(!K::finite(point))return out;
      for(const auto& o:obstacles)
      {
        const float clearance=K::obstacleDistance(previous,point,o)-o.radius-ballRadius-p.obstaclePadding;
        if(clearance<=0.f){out.hardConflict=true;out.reason="blocked_corridor";return out;}
        if(o.mobile&&!o.teammate)
        {
          ++out.mobileChecks;const float hard=clearance-o.speedBound*p.opponentReleaseAdvantage;
          out.hardClearance=std::min(out.hardClearance,hard);
          if(hard<=0.f){out.hardConflict=true;out.reason="early_defence_swept_corridor";return out;}
          const float speed=temporalSpeed(o);
          if(speed>0.f)
          {
            const float gap=clearance/speed-p.opponentReleaseAdvantage-t;
            out.minGap=std::min(out.minGap,gap);
            if(gap<temporalMargin){out.hardConflict=true;out.reason="race_opponent_can_arrive_first";return out;}
          }
        }
      }
      previous=point;if(t>=controlTime)break;
    }
    out.valid=true;out.reason="race_prefix_ready";return out;
  }
}
