#include <cpp/Color.hpp>
#include <cpp/Math.hpp>
#include <cpp/Button.hpp>
#include <cpp/DiscreteOut.hpp>
#include <cpp/Servo.hpp>
#include <cpp/Logging.hpp>
#include <cpp/Nunchuck.hpp>
#include <cpp/LedStripWs2812b.hpp>
#include <cpp/Settings.hpp>

#include <hardware/watchdog.h>
#include <pico/stdlib.h>
#include <pico/stdio.h>
#include <pico/bootrom.h>

#include <memory>
#include <iostream>
#include <istream>
#include <sstream>
#include <cstring>
#include <string>
#include <cmath>
#include <map>

enum class EyeState
{
  Neutral,
  Happy,
  Sad,
  Blank,
};

Servo servoX(0);
Servo servoY(1);

bool printNunchuck = false;
std::map<EyeState, LEDBuffer> anim;
EyeState currentAnim;

int ringLightOffset = 6;
LedStripWs2812b leds = LedStripWs2812b::create(2);

void setupAnim()
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

void shutdown()
{
  servoX.release();
  servoY.release();
  leds.writeColors(anim[EyeState::Blank], 0.0f);
}

void rebootIntoProgMode()
{
  //multicore_reset_core1();  // Only needed for multicore
  shutdown();
  reset_usb_boot(0,0);
}

template <typename T>
bool setValFromStream(T& val, T min, T max, std::istream& s)
{
  T input;
  s >> input;
  if (s.fail())
  {
    std::cout << "parse error" << std::endl << std::flush;
    return false;
  }
  if (!between(input, min, max))
  {
    std::cout << "value out of range error" << std::endl << std::flush;
    return false;
  }
  val = input;
  return true;
}

bool setValFromStream(bool& val, std::istream& s)
{
  int input;
  s >> input;
  if (s.fail())
  {
    std::cout << "parse error" << std::endl << std::flush;
    return false;
  }
  if (input < 0 || input > 1)
  {
    std::cout << "value out of range error" << std::endl << std::flush;
    return false;
  }
  val = (input == 1);
  return true;
}

bool setValFromStream(char* val, size_t len, std::istream& ss)
{
  // Eat any extra whitespace in the stream
  ss >> std::ws;
  // Get the whole rest of the line as a string
  std::string str;
  std::getline(ss, str);

  // Check if the read was clean and if the string will fit
  if (ss.fail())
  {
    std::cout << "parse error" << std::endl << std::flush;
    return false;
  }
  if (str.size() >= len)
  {
    std::cout << "string param too long" << std::endl << std::flush;
    return false;
  }
  // Copy the string to the dest buffer will null terminator
  memcpy(val, str.data(), str.size());
  val[str.size()] = '\0';
  return true;
}

void processCommand(std::string cmdAndArgs, Settings& settings)
{
  std::stringstream ss(cmdAndArgs);
  std::string cmd;
  ss >> cmd;
  
  if (cmd == "defaults")
  {
    settings.setDefaults();
  }
  else if (cmd == "flash")
  {
    // Write the settings to flash
    if (settings.writeToFlash())
      std::cout << "Wrote settings to flash!" << std::endl << std::flush;
    else
      std::cout << "Skipped writing to flash because contents were already correct." << std::endl << std::flush;
  }
  else if (cmd == "info" || cmd == "about")
  {
    std::cout << "Redeye by Donkey Kong" << std::endl;
    std::cout << std::endl;
    settings.print();
    std::cout << std::endl;
    std::cout << "-- Runtime Data --" << std::endl;
    std::cout << "full settings size: " << sizeof(Settings) << std::endl;
    std::cout << std::flush;
  }
  else if (cmd == "reboot")
  {
    // Reboot the system immediately
    std::cout << "ok" << std::endl << std::flush;
    watchdog_reboot(0,0,0);
  }
  else if (cmd == "release")
  {
    servoX.release();
    servoY.release();
  }
  else if (cmd == "nunchuck")
  {
    printNunchuck = !printNunchuck;
  }
  else if (cmd == "goT")
  {
    double xT, yT;
    if (setValFromStream(xT, 0.0, 1.0, ss) &&
        setValFromStream(yT, 0.0, 1.0, ss))
    {
      servoX.posT(xT);
      servoY.posT(yT);
    }
  }
  else if (cmd == "goDeg")
  {
    double xDeg, yDeg;
    if (setValFromStream(xDeg, 0.0, 180.0, ss) &&
        setValFromStream(yDeg, 0.0, 180.0, ss))
    {
      servoX.posDeg(xDeg);
      servoY.posDeg(yDeg);
    }
  }
  else if (cmd == "prog")
  {
    // Reboot into programming mode
    std::cout << "ok" << std::endl << std::flush;
    rebootIntoProgMode();
  }
  else
  {
    std::cout << "unknown command error" << std::endl << std::flush;
    return;
  }

  if (!ss.fail())
  {
    std::cout << "ok" << std::endl << std::flush;
  }
}

void processStdIo(Settings& settings)
{
  static char inBuf[1024];
  static int pos = 0;
  static std::string lastCmd;

  while (true)
  {
    int inchar = getchar_timeout_us(0);
    if (inchar > 31 && inchar < 127 && pos < 1023)
    {
      inBuf[pos++] = (char)inchar;
      std::cout << (char)inchar << std::flush; // echo to client
    }
    else if (inchar == '\b' && pos > 0) // handle backspaces
    {
      --pos;
      std::cout << "\b \b" << std::flush;
    }
    else if (inchar == '\t' && lastCmd.size() < 1023) // handle tab to insert last command
    {
      while (pos-- > 0) std::cout << "\b \b" << std::flush;
      memcpy(inBuf, lastCmd.data(), lastCmd.size());
      pos = lastCmd.size();
      std::cout << lastCmd << std::flush;
    }
    else if (inchar == '\n')
    {
      inBuf[pos] = '\0';
      std::cout << std::endl << std::flush; // echo to client
      lastCmd = inBuf;
      processCommand(inBuf, settings);
      pos = 0;
    }
    else
    {
      return;
    }
  }
}

int main()
{
  // Configure stdio
  stdio_init_all();

  #ifdef LOGGING_ENABLED
  sleep_ms(2000);
  #endif

  // Init the settings object
  SettingsManager settingsMgr;
  Settings& settings = settingsMgr.getSettings();

  // Setup the eye animation system
  setupAnim();
  currentAnim = EyeState::Neutral;

  Nunchuck nunchuck(i2c1, 14, 15);

  absolute_time_t nextFrameTime = get_absolute_time();

  while (1)
  {
    // Regulate loop speed
    sleep_until(nextFrameTime);
    nextFrameTime = make_timeout_time_ms(16);

    // Nunchuck request and fetch must be separated by a few ms
    // the object enforces this but we might as well try to do work in between...
    nunchuck.requestControllerState();

    // Refresh the ring light
    leds.writeColors(anim[currentAnim], 0.2f);


    nunchuck.fetchControllerState();

    if (nunchuck.ok())
    {
      if (printNunchuck)
      {
        std::cout << "Stick: ( " << nunchuck.stickX() << " , " << nunchuck.stickY() << " )" << std::endl;
        std::cout << "Raw Stick: ( " << (int)nunchuck.rawStickX() << " , " << (int)nunchuck.rawStickY() << " )" << std::endl;
        std::cout << "Accel: ( " << nunchuck.accelX() << " , " << nunchuck.accelY() << " , " << nunchuck.accelZ() <<  " )" << std::endl;
        std::cout << "Raw Accel: ( " << nunchuck.rawAccelX() << " , " << nunchuck.rawAccelY() << " , " << nunchuck.rawAccelZ() <<  " )" << std::endl;
      }
      
      if (nunchuck.c())
      {
        // Pick an emotion with accelerometer
        if (nunchuck.accelY() < -0.2f) currentAnim = EyeState::Happy;
        else if (nunchuck.accelY() > 0.2f) currentAnim = EyeState::Sad;
        else currentAnim = EyeState::Neutral;
      }
      if (nunchuck.z())
      {
        // Control with accelerometer
        servoX.posT(std::clamp((nunchuck.yaw() + 1.0) / 2.0, 0.0, 1.0));
        servoY.posT(std::clamp((nunchuck.pitch() + 1.0) / 2.0, 0.0, 1.0));
      }
      else
      {
        servoX.posT((nunchuck.stickX() + 1.0) / 2.0);
        servoY.posT((nunchuck.stickY() + 1.0) / 2.0);
      }

    }

    // Process input
    processStdIo(settings);
  }
  return 0;
}
