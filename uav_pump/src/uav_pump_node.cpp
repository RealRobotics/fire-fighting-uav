#include <string>
#include <ros/ros.h>
#include <ros/console.h>

#include "gimbal_action.h"
#include "pump_action.h"
#include "communications.h"

int main(int argc, char** argv)
{
  std::string name("pump");
  ros::init(argc, argv, name);
  ros::Time::init();
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ROS_INFO("Starting pump node...");

  // Instantiate communications.
  Communications* comms = new Communications();
  comms->init();
  // Instantiate actions.
  // NOTE: The name strings must match those in the client.
  name = "pump";
  PumpAction pump(name, comms);
  name = "gimbal";
  GimbalAction gimbal(name, comms);

  ROS_INFO("Pump node ready.");
  ros::spin();

  free(comms);
  return 0;
}
