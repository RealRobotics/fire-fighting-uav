#ifndef FCS_INTERFACE_H
#define FCS_INTERFACE_H

#include <mutex> 

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <actionlib/server/simple_action_server.h>

#include <dji_sdk/MissionWaypointTask.h>

#include "uav_msgs/FlyToWPAction.h"
#include "uav_msgs/SpecialMovementAction.h"

class FCS_Interface
{
public:
  FCS_Interface(ros::NodeHandle node_handle);

  /** Connects to the drone autopilot and prepares for any other commands.
   * @return true if the drone is ready to be used, false otherwise
   */
  bool start();

private:
  enum DroneTask
  {
    kTaskTakeOff,
    kTaskLand,
    kTaskGoHome
  };

  enum WaypointAction
  {
    kActionStart,
    kActionStop,
    kActionPause,
    kActionResume
  };

  bool activate_();
  bool obtainControlAuthority_();
  /** This method arms the drone and takes off.
   * Blocks until done.
   * @return true on success.
   */
  bool getReady_();
  bool droneTaskControl_(DroneTask task);

  bool specialMovement_(const uav_msgs::SpecialMovementGoalConstPtr &goal);
  bool setWaypoint_(const uav_msgs::FlyToWPGoalConstPtr &goal);

  void convertToWaypoint_(const sensor_msgs::NavSatFix& nav_sat_fix, dji_sdk::MissionWaypoint* waypoint);
  void setWaypointInitDefaults_(dji_sdk::MissionWaypointTask* waypointTask);
  bool uploadNavSatFix_(const sensor_msgs::NavSatFix& nav_sat_fix);
  bool waypointMissionAction_(WaypointAction action);
  bool droneWithinRadius_(double radius, sensor_msgs::NavSatFix goal);

  void gpsPositionCallback_(const sensor_msgs::NavSatFix::ConstPtr& message);
  void batteryStateCallback_(const sensor_msgs::BatteryState::ConstPtr& message);

  ros::NodeHandle node_handle_;
  ros::ServiceClient control_authority_client_;
  ros::ServiceClient drone_activation_client_;
  ros::ServiceClient drone_task_client_;
  ros::ServiceClient waypoint_action_client_;
  ros::ServiceClient waypoint_upload_client_;

  ros::Subscriber gps_position_subscriber_;
  ros::Subscriber battery_state_subscriber_;

  ros::Publisher battery_state_publisher_; 

  bool loaded_ {false};
  sensor_msgs::NavSatFix gps_position_;
  double altitude_ {0.0};
  
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

  const double take_off_height_ {1.2};
  const double height_precision_ {0.05};
  sensor_msgs::NavSatFix home_location_;
  bool home_position_initialised_ {false};

};

#endif //FCS_INTERFACE_H
