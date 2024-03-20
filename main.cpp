// application headers
#include "Eyes.hpp"

// pi pico C++ headers
#include <cpp/Color.hpp>
#include <cpp/Math.hpp>
#include <cpp/Button.hpp>
#include <cpp/DiscreteOut.hpp>
#include <cpp/Servo.hpp>
#include <cpp/Logging.hpp>
#include <cpp/Nunchuck.hpp>
#include <cpp/LedStripWs2812b.hpp>
#include <cpp/CommandParser.hpp>
#include <cpp/FlashStorage.hpp>

// Pico SDK headers
#include <hardware/watchdog.h>
#include <pico/stdlib.h>
#include <pico/stdio.h>
#include <pico/bootrom.h>

// std headers
#include <iostream>
#include <string>
#include <algorithm>
#include <map>

enum class EyeState
{
  Neutral,
  Happy,
  Sad,
  Blank,
};

void setupAnim(std::map<EyeState, LEDBuffer>& anim)
{
  RGBColor r{255, 0, 0};
  RGBColor k{0, 0, 0};
  anim = {
    {EyeState::Neutral, {r,r,r,r,r,r,r,r,r,r,r,r}},
    {EyeState::Happy,   {k,k,k,k,k,r,r,r,r,r,r,r}},
    {EyeState::Sad,     {r,r,r,r,r,r,k,k,k,k,k,r}},
    {EyeState::Blank,   {k,k,k,k,k,k,k,k,k,k,k,k}},
  };
}

int main()
{
  // Configure stdio
  stdio_init_all();
  #ifdef LOGGING_ENABLED
  sleep_ms(2000);
  #endif

  // Init the servos
  Servo servoLX(0, -90.0f, 90.0f);
  Servo servoLY(3, -90.0f, 90.0f);
  Servo servoRX(2, -90.0f, 90.0f);
  Servo servoRY(1, -90.0f, 90.0f);
  servoRY.invert = true;

  // Eye motion system
  FlashStorage<EyeParams> flashStorage;
  flashStorage.readFromFlash();
  EyeParams& eyeParams = flashStorage.data;
  float eyeVerge = 1000.0f;
  Eyes eyes(servoLX, servoLY, servoRX, servoRY, eyeParams);

  // Setup the lights and eye animation system
  std::map<EyeState, LEDBuffer> anim;
  setupAnim(anim);
  EyeState currentAnim = EyeState::Neutral;
  int ringLightOffset = 6;
  LedStripWs2812b ledL = LedStripWs2812b::create(4);
  LedStripWs2812b ledR = LedStripWs2812b::create(5);

  // Setup the controller input object
  bool enableNunchuck = true;
  Nunchuck nunchuck(i2c1, 14, 15);

  // Create a utility function for turning off the servos and LEDs
  auto shutdownHardware = [&]()
  {
    servoLX.release();
    servoLY.release();
    servoRX.release();
    servoRY.release();
    ledL.writeColors(anim[EyeState::Blank], 0.0f);
    ledR.writeColors(anim[EyeState::Blank], 0.0f);
  };

  // Init the command parser
  CommandParser parser;
  parser.addCommand("reboot", [&]()
  {
    std::cout << "ok" << std::endl;
    shutdownHardware();
    watchdog_reboot(0,0,0);
  }, "", "Reboot the microcontroller");

  parser.addCommand("prog", [&]()
  {
    std::cout << "ok" << std::endl;
    shutdownHardware();
    reset_usb_boot(0,0);
  }, "", "Enter USB programming mode");

  parser.addCommand("en", [&](bool enable)
  {
    enableNunchuck = enable;
  }, "[0,1]", "Enable or disable nunchuck input");

  parser.addCommand("dir", [&](double yaw, double pitch)
  {
    eyes.lookDir(yaw, pitch, eyeVerge);
  }, "[yaw] [pitch]", "Look at the specified yaw and pitch in degrees");

  parser.addCommand("look", [&](double x, double y, double z)
  {
    eyes.lookAt(x, y, z);
  }, "[x] [y] [z]", "Look at the specified location x,y,z");

  parser.addCommand("ipd", [&](double ipd)
  {
    eyeParams.ipd = ipd;
  }, "[dist]", "Set interpupilary distance (eye spacing)");

  parser.addCommand("verge", [&](double verge)
  {
    eyeVerge = verge;
  }, "[dist]", "Set verge distance (distance to focus)");

  parser.addCommand("center", [&](std::string side)
  {
    if (side == "l" || side == "b")
    {
      eyeParams.offsetYawL = eyes.yawL;
      eyeParams.offsetPitchL = eyes.pitchL;
    }
    if (side == "r" || side == "b")
    {
      eyeParams.offsetYawR = eyes.yawR;
      eyeParams.offsetPitchR = eyes.pitchR;
    }
  }, "[l/r/b]", "Set the current pose to be the center for the left, right or both eyes");

  parser.addCommand("speed", [&](float speed)
  {
    eyeParams.speedDegSec = speed;
  }, "", "Set the max eye speed in degrees per sec (default: 600)");

  parser.addCommand("defaults", [&]()
  {
    eyeParams = EyeParams();
  }, "", "Set all params back to defaults");

  parser.addCommand("save", [&]()
  {
    flashStorage.writeToFlash();
  }, "", "Write all the parameters to flash");

  absolute_time_t nextFrameTime = get_absolute_time();
  while (1)
  {
    // Regulate loop speed
    sleep_until(nextFrameTime);
    nextFrameTime = make_timeout_time_ms(16);

    // Nunchuck request and fetch must be separated by a few ms
    // the object enforces this but we might as well try to do work in between...
    nunchuck.requestControllerState();

    // Refresh the ring lights
    ledL.writeColors(anim[currentAnim], 0.2f);
    ledR.writeColors(anim[currentAnim], 0.2f);

    // Give the command parser a chance to read stdin
    parser.processStdIo();

    nunchuck.fetchControllerState();
    if (enableNunchuck && nunchuck.ok())
    {
      if (nunchuck.c())
      {
        // // Pick an emotion with accelerometer
        // if (nunchuck.accelY() < -0.5f) currentAnim = EyeState::Happy;
        // else if (nunchuck.accelY() > 0.5f) currentAnim = EyeState::Sad;
        // else currentAnim = EyeState::Neutral;
        currentAnim = EyeState::Blank;
      }
      else
      {
        currentAnim = EyeState::Neutral;
      }

      if (nunchuck.z())
      {
        eyeVerge = std::clamp((nunchuck.pitch() + 45.0f)* 4.0f, 1.0f, 3000.0f);
      }
      
      eyes.lookDir(nunchuck.stickX() * 90.0f, nunchuck.stickY() * 90.0f, eyeVerge);
      
    }

    eyes.update();
  }
  return 0;
}
