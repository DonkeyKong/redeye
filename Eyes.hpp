#pragma once

#include <cpp/Vector.hpp>
#include <cpp/Servo.hpp>
#include <cpp/LedStripWs2812b.hpp>

#include <pico/stdlib.h>

#include <cmath>
#include <map>

static float LedIdxToX[12];
static float LedIdxToY[12];

enum class EyeBase
{
  Off,
  Neutral,
  Sad,
  Happy,
  //Angry,
  Debug,
};

enum class EyeAnim
{
  None,
  Blink,
  // WinkL,
  // WinkR,
  // Fade,
  // Hypno,
  // Rainbow
};

struct EyeParams
{
  float ipd = 100.0f;
  float offsetYawL = 0.0f;
  float offsetPitchL = 0.0f;
  float offsetYawR = 0.0f;
  float offsetPitchR = 0.0f;
  float speedDegSec = 600.0;
};

class EyeAnimProg
{
public:
  virtual void trigger() { }

  virtual void end() { }

  virtual void update(float secElapsed, float& offsetYL, float& offsetPL, float& offsetYR, float& offsetPR, LEDBuffer& lBuf, LEDBuffer& rBuf)
  {
    offsetYL = 0;
    offsetPL = 0;
    offsetYR = 0;
    offsetPR = 0;
  }

  virtual bool done()
  {
    return false;
  }
};

class BlinkProg : public EyeAnimProg
{
  float t = 0;
  bool opening = false;
  static constexpr float blinkDisplacementDeg = -45.0f;
public:
  void trigger() override
  {
    t = 0;
    opening = false;
  }
  void end() override
  {
    opening = true;
  }
  void update(float secElapsed, float& offsetYL, float& offsetPL, float& offsetYR, float& offsetPR, LEDBuffer& lBuf, LEDBuffer& rBuf) override
  {
    offsetYL = 0;
    offsetYR = 0;
    float speedT = 10.0f * secElapsed;

    if (opening == false)
    {
      t = moveTowards(t, 1.0f, speedT);
    }
    else
    {
      t = moveTowards(t, 0.0f, speedT);
    }

    float lidPos = t * 2.0f - 1.2f;
    for (int i=0; i < 12; ++i)
    {
      if (LedIdxToY[i] < lidPos)
      {
        lBuf[i] = {0,0,0};
        rBuf[i] = {0,0,0};
      }
    }
    
    offsetPL = t * blinkDisplacementDeg;
    offsetPR = t * blinkDisplacementDeg;
  }
  virtual bool done()
  {
    return opening && t == 0.0f;
  }
};

// This class describes a 2-eye pair. It implicitly
// defines a coordinate system from the POV of the eyes
// that is +x right, +y forward, +z up.
// Left eye location:  [ -(ipdMm / 2), 0 , 0 ]
// Right eye location: [  (ipdMm / 2), 0 , 0 ]
// All distance values in mm 
// (though it doesn't matter if you set them all consistently)
class Eyes
{
  Servo& yl_;
  Servo& pl_;
  Servo& yr_;
  Servo& pr_;
  LedStripWs2812b& l_;
  LedStripWs2812b& r_;
  EyeParams& params_;

  // Destination position
  float destYawL_ = 0.0f;
  float destPitchL_ = 0.0f;
  float destYawR_ = 0.0f;
  float destPitchR_ = 0.0f;

  absolute_time_t lastUpdateTime_;
  std::map<EyeBase, LEDBuffer> bases_;
  std::map<EyeAnim, std::unique_ptr<EyeAnimProg>> anims_;
  EyeAnim anim_ = EyeAnim::None;
  EyeBase base_ = EyeBase::Neutral;
  LEDBuffer lBuf_;
  LEDBuffer rBuf_;
  static constexpr int ringLightOffset = 6;

public:
  // Last sent position
  float yawL = 0.0f;
  float pitchL = 0.0f;
  float yawR = 0.0f;
  float pitchR = 0.0f;

  Eyes(Servo& yl, Servo& pl, Servo& yr, Servo& pr, LedStripWs2812b& l, LedStripWs2812b& r, EyeParams& params)
    : yl_{yl}
    , pl_{pl}
    , yr_{yr}
    , pr_{pr}
    , l_{l}
    , r_{r}
    , params_{params}
    , lastUpdateTime_{from_us_since_boot(0)}
  {
    setupAnim();
  }

  // No copy, no move
  Eyes(const Eyes&) = delete;
  Eyes(Eyes&&) = delete;

  void setupAnim()
  {
    RGBColor w{255, 255, 255};
    RGBColor m{127,   0, 127};
    RGBColor r{255,   0,   0};
    RGBColor y{127, 127,   0};
    RGBColor g{  0, 255,   0};
    RGBColor c{  0, 127, 127};
    RGBColor b{  0,   0, 255};
    RGBColor k{  0,   0,   0};

    bases_ = {
      {EyeBase::Neutral, {w,w,w,w,w,w,w,w,w,w,w,w}},
      {EyeBase::Happy,   {k,k,k,k,k,w,w,w,w,w,w,w}},
      {EyeBase::Sad,     {w,w,w,w,w,w,k,k,k,k,k,w}},
      {EyeBase::Off,     {k,k,k,k,k,k,k,k,k,k,k,k}},
      {EyeBase::Debug,   {w,m,r,y,g,c,b,k,m,r,y,g}},
    };

    anims_.emplace(EyeAnim::None, std::make_unique<EyeAnimProg>());
    anims_.emplace(EyeAnim::Blink, std::make_unique<BlinkProg>());

    for (int i=0; i < 12; ++i)
    {
      float theta = (5.0f-i) / 6.0f * M_PI;
      LedIdxToX[i] = std::cos(theta);
      LedIdxToY[i] = std::sin(theta);
    }
  }

  void setBase(EyeBase state)
  {
    base_ = state;
  }

  void setColor(RGBColor color)
  {
    l_.colorBalance(color.toVec3f());
    r_.colorBalance(color.toVec3f());
  }

  void triggerAnim(EyeAnim anim)
  {
    anim_ = anim;
    anims_[anim]->trigger();
  }

  void endAnim()
  {
    anims_[anim_]->end();
  }

  void lookAt(float x, float y, float z)
  {
    float dist = Vec3f{x, y, z}.norm();
    lookDir(std::atan2(y, x) * (180.0f / M_PI), std::asin(z/dist) * (180.0f / M_PI), dist);
  }

  // Yaw and pitch in degrees
  void lookDir(float yaw, float pitch, float dist = 1000.0f)
  {
    float adjust = std::atan2(params_.ipd/2.0f, dist) * (180.0f / M_PI);
    destYawL_ = yaw + adjust + params_.offsetYawL;
    destYawR_ = yaw - adjust + params_.offsetYawR;
    destPitchL_ = pitch + params_.offsetPitchL;
    destPitchR_ = pitch + params_.offsetPitchR;
  }

  void update()
  {
    auto now = get_absolute_time();
    float elapsedSecs = 0;
    if (to_us_since_boot(lastUpdateTime_) > 0)
    {
      elapsedSecs = (float)((double) absolute_time_diff_us(lastUpdateTime_, now) / 1000000.0);
    }
    lastUpdateTime_ = now;

    // Reset the left and right buffers to their base image
    lBuf_ = bases_[base_];
    rBuf_ = bases_[base_];

    // If there is no animation or the animation is done
    // then revert back to the none animation
    if (anims_[anim_]->done())
    {
      anim_ = EyeAnim::None;
    }

    // Update any running animation
    float offsetYL = 0, offsetPL = 0, offsetYR = 0, offsetPR = 0;
    anims_[anim_]->update(elapsedSecs, offsetYL, offsetPL, offsetYR, offsetPR, lBuf_, rBuf_);

    // Move current pos toward dest
    float crawlSpeed = elapsedSecs * params_.speedDegSec; 
    yawL = moveTowards(yawL, destYawL_, crawlSpeed);
    pitchL = moveTowards(pitchL, destPitchL_, crawlSpeed);
    yawR = moveTowards(yawR, destYawR_, crawlSpeed);
    pitchR = moveTowards(pitchR, destPitchR_, crawlSpeed);

    // Send pos to servos
    yl_.posDeg(yawL + offsetYL);
    pl_.posDeg(pitchL + offsetPL);
    yr_.posDeg(yawR + offsetYR);
    pr_.posDeg(pitchR + offsetPR);

    l_.writeColors(lBuf_, 0.1f);
    r_.writeColors(rBuf_, 0.1f);
  }

  void shutdownHardware()
  {
    yl_.release();
    pl_.release();
    yr_.release();
    pr_.release();
    l_.writeColors(bases_[EyeBase::Off], 0.0f);
    r_.writeColors(bases_[EyeBase::Off], 0.0f);
    sleep_ms(100); // Be sure all the writes happened
  };
};