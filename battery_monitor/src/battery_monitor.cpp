#include <atomic>
#include <string>

#include <ros/ros.h>

#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#include "uav_msgs/BatteryStatus.h"
#include "uav_msgs/InputAccepted.h"

ros::Publisher battery_status_pub;
ros::Publisher input_status_pub;
//the limit under which battery monitor will start to publish safety critical msg, -1 is invalid setting, 100 is max value
std::atomic<int8_t> safety_threshold {-1};
//the limit under which battery monitor will start to publish mission critical msg, -1 is invalid setting, 100 is max value
std::atomic<int8_t> mission_threshold {-1};
//max percentage level for the battery, technically drones should never return number bigger than 1 but it doesnt hurt to make this code safe
constexpr int8_t kMaxBatteryLevel {100};
//min percentage level for the battery, technically drones should never return number smaller than 0 but it doesnt hurt to make this code safe
constexpr int8_t kMinBatteryLevel {0};

void BatteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
  uav_msgs::InputAccepted msg2;
  msg2.data = true;
  msg2.input_msg_id = msg->header.seq;
  input_status_pub.publish(msg2);

  uav_msgs::BatteryStatus output_msg;
  output_msg.input_msg_id = msg->header.seq;
  output_msg.status = uav_msgs::BatteryStatus::UNSET;

  if ((safety_threshold != -1) && (mission_threshold != -1)) { //the thresholds must be set for the status to be published
    int8_t current_battery {(int8_t)round(msg->percentage*100)};

    if ((current_battery > mission_threshold) && (current_battery <= kMaxBatteryLevel)){
      output_msg.status = uav_msgs::BatteryStatus::OK;
    } else if ((current_battery <= mission_threshold) && (current_battery > safety_threshold )) {
      output_msg.status = uav_msgs::BatteryStatus::MISSION_CRITICAL;
    } else if ((current_battery <= safety_threshold) && (current_battery >= kMinBatteryLevel)) {
      output_msg.status = uav_msgs::BatteryStatus::SAFETY_CRITICAL;
    } else {
      ROS_ERROR("Invalid battery reading.");
      output_msg.status = uav_msgs::BatteryStatus::BATTERY_READING_OUT_OF_SCOPE;
    }
  }
  battery_status_pub.publish(output_msg);
}

int8_t LoadThreshold(ros::NodeHandle nh, std::string threshold_name) {
  int read_threshold {-1};
  if (nh.getParam(threshold_name, read_threshold)) {
    if ((read_threshold >= 0) && (read_threshold <=100)) {
      ROS_INFO("%s is going to be set to %d percent", threshold_name.c_str(), read_threshold);
    } else {
      ROS_ERROR("Value for %s must be between 0 and 100.", threshold_name.c_str());
      ros::shutdown();
    }
  } else {
    ROS_ERROR("Failed to get param %s", threshold_name.c_str());
    ros::shutdown();
  }
  return (int8_t)read_threshold;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "battery_monitor");
  ros::NodeHandle nh;

  safety_threshold = LoadThreshold(nh, "safety_threshold");
  mission_threshold = LoadThreshold(nh, "mission_threshold");

  if (mission_threshold<safety_threshold) {
    ROS_ERROR("Mission threshold is below Safety threshold! Please fix the settings.");
    ros::shutdown();
  }

  input_status_pub = nh.advertise<uav_msgs::InputAccepted>("input_accepted",1000);
  battery_status_pub = nh.advertise<uav_msgs::BatteryStatus>("battery_status", 1000);

  ros::Subscriber battery_sub = nh.subscribe("/fcs/battery_state", 10, BatteryCallback);

  ROS_INFO("Battery monitor is running");

  ros::spin();

  return 0;
}