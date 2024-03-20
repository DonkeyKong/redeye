#pragma once

#include <cpp/Vector.hpp>
#include <cpp/Servo.hpp>

#include <pico/stdlib.h>

#include <cmath>

struct EyeParams
{
  float ipd = 100.0f;
  float offsetYawL = 0.0f;
  float offsetPitchL = 0.0f;
  float offsetYawR = 0.0f;
  float offsetPitchR = 0.0f;
  float speedDegSec = 600.0;
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
  EyeParams& params_;

  // Destination position
  float destYawL_ = 0.0f;
  float destPitchL_ = 0.0f;
  float destYawR_ = 0.0f;
  float destPitchR_ = 0.0f;

  absolute_time_t lastUpdateTime_;

public:
  Eyes(Servo& yl, Servo& pl, Servo& yr, Servo& pr, EyeParams& params)
    : yl_{yl}
    , pl_{pl}
    , yr_{yr}
    , pr_{pr}
    , params_{params}
    , lastUpdateTime_{from_us_since_boot(0)}
  { }

  // No copy, no move
  Eyes(const Eyes&) = delete;
  Eyes(Eyes&&) = delete;

  // Last sent position
  float yawL = 0.0f;
  float pitchL = 0.0f;
  float yawR = 0.0f;
  float pitchR = 0.0f;

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
    // Determine the desired crawl speed to get stable motion
    float crawlSpeed;
    auto now = get_absolute_time();
    if (to_us_since_boot(lastUpdateTime_) == 0)
    {
      crawlSpeed = 0;
    }
    else
    {
      crawlSpeed = ((double) absolute_time_diff_us(lastUpdateTime_, now) / 1000000.0) * params_.speedDegSec; 
    }
    lastUpdateTime_ = now;

    // Move current pos toward dest
    yawL = moveTowards(yawL, destYawL_, crawlSpeed);
    pitchL = moveTowards(pitchL, destPitchL_, crawlSpeed);
    yawR = moveTowards(yawR, destYawR_, crawlSpeed);
    pitchR = moveTowards(pitchR, destPitchR_, crawlSpeed);

    // Send pos to servos
    yl_.posDeg(yawL);
    pl_.posDeg(pitchL);
    yr_.posDeg(yawR);
    pr_.posDeg(pitchR);
  }
};