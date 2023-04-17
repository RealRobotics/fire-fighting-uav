#include "communications.h"

#include <sys/stat.h>
#include <cstdlib>
#include <string>
#include <thread>
#include <ros/ros.h>
#include <serial/serial.h>

#define SERIAL_PORT "/dev/ttyACM0"
#define SERIAL_BAUD_RATE (9600)
#define SERIAL_TIME_OUT_MS (250)
// Rate of looping. 10.0 = 100ms per loop.
#define UPDATE_RATE_PER_S (10.0)
// Check arduino every 250ms.
#define POLL_RATE_PER_S (4.0)
#define SERIAL_MAX_COMMAND_LENGTH (256)
// Status update interval 0.2 = 200ms
#define STATUS_UPDATE_INTERVAL_S (0.2)
// Adjust for size of water bag.
// Could be a ROS parameter?
#define MAX_LITRES (4)

// Both servos have a range of 0 to 180 degrees with centre at 90 degrees.
#define PITCH_MIN_DEGREES (-90)
#define PITCH_MAX_DEGREES (90)
#define PITCH_OFFSET_DEGREES (90)
#define YAW_MIN_DEGREES (-90)
#define YAW_MAX_DEGREES (90)
#define YAW_OFFSET_DEGREES (90)


Communications::Communications() : serial_(0), readLoopThread_(0), arduino_ready_(false)
{
  serial_ = new serial::Serial();
  pump_.litres_remaining = MAX_LITRES;
}

Communications::~Communications()
{
  readLoopThread_->join();
  delete readLoopThread_;
  delete serial_;
}

/** Set up the serial port communications.
 * Configure the serial port, baud rate etc.
 * Set up a callback to handle messages sent from the crane microcontroller.
 */
void Communications::init()
{
  ROS_DEBUG("Called: %s", __FUNCTION__);
  // See if serial port exists.
  struct stat buffer;
  bool port_exists = (stat(SERIAL_PORT, &buffer) == 0);
  if (port_exists)
  {
    try
    {
      // Open the serial port.
      serial::Timeout timeout = serial::Timeout::simpleTimeout(SERIAL_TIME_OUT_MS);
      serial_->setPort(SERIAL_PORT);
      serial_->setBaudrate(SERIAL_BAUD_RATE);
      serial_->setTimeout(timeout);
      serial_->open();
    }
    catch (...)
    {
      // One of:
      // serial::PortNotOpenedException
      // serial::IOException
      // std::invalid_argument
      delete serial_;
      std::exception_ptr p = std::current_exception();
      ROS_ERROR("%s: exception thrown: %s", __FUNCTION__, (p ? p.__cxa_exception_type()->name() : "null"));
      throw;
    }
    // Opened so start readLoop
    if (serial_->isOpen())
    {
      startReadLoop();
    }
    else
    {
      ROS_ERROR("%s: Failed to open serial port on %s", __FUNCTION__, SERIAL_PORT);
    }
  }
  else
  {
    ROS_ERROR("%s: serial port %s was not found.", __FUNCTION__, SERIAL_PORT);
    // FIXME(AJB) Using exit causes a memory leak so it is not ideal.
    // ros::shutdown() doesn't stop the node properly so this is used as it works.
    exit(1);
  }
}

/** Gets the last reported value of the gimbal.
 * @param pitch The pitch value in degrees.
 * @param yaw The yaw value in degrees.
 * @return true if the action has been completed.
 */
bool Communications::getGimbal(int* pitch, int* yaw)
{
  ROS_DEBUG("Called: %s", __FUNCTION__);
  *pitch = gimbal_.last_pitch;
  *yaw = gimbal_.last_yaw;
  // TODO This may not always be correct, but it should be good enough.
  return gimbal_.done_pitch || gimbal_.done_yaw;
}

/** Set the pitch and yaw values for the gimbal.
 * @param pitch The pitch value in degrees.
 * @param yaw The yaw value in degrees.
 */
void Communications::setGimbal(const int pitch, const int yaw)
{
  ROS_DEBUG("Called: %s, pitch %d, yaw %d.", __FUNCTION__, pitch, yaw);
  if (gimbal_.last_pitch != pitch) {
    if (pitch >= PITCH_MIN_DEGREES && pitch <= PITCH_MAX_DEGREES) {
      // Update internal variables.
      gimbal_.requested_pitch = pitch;
      gimbal_.done_pitch = false;
      int server_pitch = pitch + PITCH_OFFSET_DEGREES;
      // Send command.
      std::string command("p");
      command += std::to_string(server_pitch);
      write(command);
    }
    else
    {
      ROS_WARN("pitch %d must be in range %d to %d degrees.", pitch, PITCH_MIN_DEGREES, PITCH_MAX_DEGREES);
    }
  }
  if (gimbal_.last_yaw != yaw) {
    if (yaw >= YAW_MIN_DEGREES && yaw <= YAW_MAX_DEGREES) {
      // Update internal variables.
      gimbal_.requested_yaw = yaw;
      gimbal_.done_yaw = false;
      int server_yaw = yaw + YAW_OFFSET_DEGREES;
      // Send command.
      std::string command("y");
      command += std::to_string(server_yaw);
      write(command);
    }
    else
    {
      ROS_WARN("yaw %d must be in range %d to %d degrees.", yaw, YAW_MIN_DEGREES, YAW_MAX_DEGREES);
    }
  }
}

/** Gets the last reported value of the pump.
 * @param running true when the pump is running.
 * @param litres_remaining Number of litres remaining.
 *                         Only updated when action completed.
 * @return true if the action has been completed.
 */
bool Communications::getPump(bool* running, float* litres_remaining) {
  ROS_DEBUG("Called: %s", __FUNCTION__);
  *running = pump_.running;
  *litres_remaining = pump_.litres_remaining;
  return pump_.done;
}

/** Start or stop the pump.
 * @param running true to start the pump, false to stop.
 */
void Communications::setPump(const bool running)
{
  ROS_DEBUG("Called: %s", __FUNCTION__);
  if (pump_.running != running) {
    std::string command;
    if (running) {
      command = "f";
    } else {
      command = "s";
    }
    write(command);
  }
}

/** Send command to serial port.
 * @param command The command to send.
 */
void Communications::write(const std::string& command)
{
  ROS_DEBUG("Called: %s", __FUNCTION__);
  if (serial_->isOpen())
  {
    ROS_DEBUG("%s. Sending '%s'", __FUNCTION__, command.c_str());
    std::string command_with_new_line = command;
    command_with_new_line += '\n';
    serial_->write(command_with_new_line);
  }
}

/** Start the readLoop thread.
 */
void Communications::startReadLoop()
{
  readLoopThread_ = new std::thread(&Communications::readLoop, this);
  if (readLoopThread_)
  {
    waitForArduino();
  }
  else
  {
    ROS_ERROR("%s: Could not start read loop thread", __FUNCTION__);
  }
}

/** Block until Arduino is ready.  */
void Communications::waitForArduino() {
  ROS_INFO("Waiting for arduino...");
  ros::Rate rate_s(POLL_RATE_PER_S);
  while (ros::ok())
  {
    // Ask for status
    write("q");
    // Give the Arduino a chance to reply.
    rate_s.sleep();
    if (arduino_ready_) {
      ROS_INFO("Arduino ready.");
      break;
    }
  }
}

/** Updates the litres remaining internal value.
 * @param reply The reply to parse.
 */
void Communications::updateLitresRemaining(const std::string &reply) {
  float value = 0.0;
  std::string float_string = reply.substr(1);
  try {
    value = std::stof(float_string);
    pump_.litres_remaining = value;
  }
  catch (const std::invalid_argument& ia) {
    ROS_ERROR("Invalid argument: %s", ia.what());
  }
}

/** Handles all data sent by serial port.
 * The returned string is used to update the member variables.
 */
void Communications::readLoop()
{
  while (ros::ok())
  {
    //  The received messages look something like this.
    // 'c123d' where 'c' is the command byte, the digits are optional and
    // the 'd' indicates done.
    // All messages from the Arduino end with a '\n', the default end of line char.
    std::string reply = serial_->readline(SERIAL_MAX_COMMAND_LENGTH);
    if (!reply.empty())
    {
      // Update internal values from response.
      char action_char = reply[0];
      char done_char = reply[1];
      ROS_DEBUG("%s. Processing reply '%s', action %c, done %c", __FUNCTION__, reply.c_str(), action_char, done_char);
      // Handle action_char.
      switch(action_char) {
        // Deal with 'q' as echo.
        case 'q':
          // Ignore.
          break;
        // Reply to query.
        case 'r':
          arduino_ready_ = true;
          updateLitresRemaining(reply);
          break;
        // Pump stopped.
        case 's':
          pump_.done = true;
          pump_.running = false;
          updateLitresRemaining(reply);
          break;
        // Pump started acknowledgement.
        case 'f':
          pump_.running = true;
          break;
        // Yaw
        case 'y':
          if (reply[1] == 'd') {
            // Done
            gimbal_.last_yaw = gimbal_.requested_yaw;
            gimbal_.done_yaw = true;
          } else {
            // Ack received so show change in value for feedback.
            int change = gimbal_.requested_yaw - gimbal_.last_yaw;
            gimbal_.last_yaw += change / 2;
          }
          break;
        // Pitch
        case 'p':
          if (reply[1] == 'd') {
            gimbal_.last_pitch = gimbal_.requested_pitch;
            gimbal_.done_pitch = true;
          } else {
            // Ack recevied so show change in value for feedback.
            int change = gimbal_.requested_pitch - gimbal_.last_pitch;
            gimbal_.last_pitch += change / 2;
          }
          break;
        default:
          ROS_WARN("%s. Unhandled action '%s'", __FUNCTION__, reply.c_str());
          break;
      }
    }  // else empty reply, timeout occurred, so wait again.
  }  // while
}
