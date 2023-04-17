#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H

#include <string>
#include <thread>
#include <serial/serial.h>

/** Handles communications with the pump microcontroller.
 * When an action is started, the getXXX functions are polled regularly and
 * return true when the action is complete.
 */
class Communications
{
public:
  Communications();
  ~Communications();
  void init();
  bool getGimbal(int* pitch, int* yaw);
  void setGimbal(const int pitch, const int yaw);
  bool getPump(bool* running, float* litres_remaining);
  void setPump(const bool running);

private:
  // Structs
  struct Gimbal
  {
    int requested_pitch;
    int last_pitch;
    bool done_pitch;
    int requested_yaw;
    int last_yaw;
    bool done_yaw;
    Gimbal()
      : requested_pitch(0.0), last_pitch(0.0), done_pitch(false)
      , requested_yaw(0.0), last_yaw(0.0), done_yaw(false)
    {
    }
  };

  struct Pump
  {
    bool running; // true when pump running.  Updated from Arduino.
    bool done;  // true when finished or stopped.
    float litres_remaining;
    Pump() : running(false), done(false), litres_remaining(0.0)
    {
    }
  };

  // Methods
  bool floatEqual(const float& rhs, const float& lhs);
  void write(const std::string& command);
  void updateLitresRemaining(const std::string &reply);
  void startReadLoop();
  void waitForArduino();
  void readLoop();

  // Action variables
  Gimbal gimbal_;
  Pump pump_;
  // Other variables.
  serial::Serial* serial_;
  std::thread* readLoopThread_;
  bool arduino_ready_;
};  // CraneCommunications

#endif
