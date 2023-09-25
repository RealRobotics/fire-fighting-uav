#ifndef DUMMY_FCS_INTERFACE_H
#define DUMMY_FCS_INTERFACE_H

#include <mutex> 

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <actionlib/server/simple_action_server.h>

#include "uav_msgs/FlyToWPAction.h"
#include "uav_msgs/SpecialMovementAction.h"

class DummyFCS_Interface
{
public:
  DummyFCS_Interface(ros::NodeHandle node_handle);

  /** Connects to the drone autopilot and prepares for any other commands.
   * @return true if the drone is ready to be used, false otherwise
   */
  bool start();

private:
  
  bool droneTaskControl_();

  bool specialMovement_(const uav_msgs::SpecialMovementGoalConstPtr &goal);
  bool setWaypoint_(const uav_msgs::FlyToWPGoalConstPtr &goal);

  bool uploadNavSatFix_(const sensor_msgs::NavSatFix& nav_sat_fix);

  void gpsPositionCallback_(const sensor_msgs::NavSatFix::ConstPtr& message);
  void batteryStateCallback_(const sensor_msgs::BatteryState::ConstPtr& message);

  ros::NodeHandle node_handle_;

  ros::Subscriber gps_position_subscriber_;
  ros::Subscriber battery_state_subscriber_;

  ros::Publisher battery_state_publisher_; 

  sensor_msgs::NavSatFix gps_position_;
  
  std::mutex position_mutex_;
  std::mutex home_mutex_;
  
  actionlib::SimpleActionServer<uav_msgs::SpecialMovementAction> special_mv_server_;
  actionlib::SimpleActionServer<uav_msgs::FlyToWPAction> fly_server_;
  std::string fly_action_name_ {"fcs/fly_to_wp"};
  std::string special_mv_action_name_ {"fcs/special_movement"};
  uav_msgs::FlyToWPFeedback fly_feedback_;
  uav_msgs::FlyToWPResult fly_result_;
  uav_msgs::SpecialMovementFeedback special_mv_feedback_;
  uav_msgs::SpecialMovementResult special_mv_result_;

  sensor_msgs::NavSatFix home_location_;
  bool home_position_initialised_ {false};

};

#endif //DUMMY_FCS_INTERFACE_H
