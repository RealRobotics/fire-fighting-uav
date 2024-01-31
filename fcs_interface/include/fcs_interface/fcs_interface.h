#ifndef FCS_INTERFACE_H
#define FCS_INTERFACE_H

#include <mutex> 

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <actionlib/server/simple_action_server.h>

#include <dji_sdk/MissionWaypointTask.h>
#include <djiosdk/dji_vehicle.hpp>

#include "uav_msgs/FlyToWPAction.h"
#include "uav_msgs/SpecialMovementAction.h"
#include "uav_msgs/RelativePositionAction.h"

#include "std_msgs/Float32.h"

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Pose.h>

#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>


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

  void relative_position_ctrl_(double &xCmd, double &yCmd, double &zCmd, double &yawCmd); // added for relative postion
  void setTarget_relative_position_(const uav_msgs::RelativePositionGoalConstPtr &goal)  ; // added for relative postion
  bool set_reference_relative_position_(); // added for relative postion

  void convertToWaypoint_(const sensor_msgs::NavSatFix& nav_sat_fix, dji_sdk::MissionWaypoint & waypoint);
  void setWaypointInitDefaults_(dji_sdk::MissionWaypointTask & waypointTask);
  bool uploadNavSatFix_(const sensor_msgs::NavSatFix& nav_sat_fix);
  bool waypointMissionAction_(WaypointAction action);
  bool droneWithinRadius_(double radius, sensor_msgs::NavSatFix goal);

  void gpsPositionCallback_(const sensor_msgs::NavSatFix::ConstPtr& message);
  void batteryStateCallback_(const sensor_msgs::BatteryState::ConstPtr& message);
  void altitudeCallback_(const std_msgs::Float32::ConstPtr& message);
  void attitudeCallback_(const geometry_msgs::QuaternionStamped::ConstPtr& msg); 
  
  void relative_position_Callback_(const geometry_msgs::PointStamped::ConstPtr& msg); //added for relative postion
  void display_mode_Callback_(const std_msgs::UInt8::ConstPtr& msg);  //added for relative postion
  void flight_status_Callback_(const std_msgs::UInt8::ConstPtr& msg); //added for relative postion
  void gps_health_Callback_(const std_msgs::UInt8::ConstPtr& msg); //added for relative postion

  sensor_msgs::NavSatFix generate_mid_point_(const sensor_msgs::NavSatFix& nav_sat_fix);
  sensor_msgs::NavSatFix generate_d_point_(const sensor_msgs::NavSatFix& nav_sat_fix, const u_int8_t point_number);// Nabil


  ros::NodeHandle node_handle_;
  ros::ServiceClient control_authority_client_;
  ros::ServiceClient drone_activation_client_;
  ros::ServiceClient drone_task_client_;
  ros::ServiceClient waypoint_action_client_;
  ros::ServiceClient waypoint_upload_client_;
  ros::ServiceClient set_local_pos_reference; // added for or relative postion 

  ros::Subscriber gps_position_subscriber_;
  ros::Subscriber battery_state_subscriber_;
  ros::Subscriber altitude_subscriber_;

  ros::Subscriber flight_status_subscriber_;  // added for relative postion 
  ros::Subscriber display_mode_subscriber_;   // added for relative postion
  ros::Subscriber local_position_subscriber_;  // added for relative postion
  ros::Subscriber gps_health_subscriber_;    // added for relative postion   
  ros::Subscriber height_above_takeoff_subscriber_;   // added for relative postion

  ros::Publisher battery_state_publisher_; 
  ros::Publisher ctrlPosYawPub; // added for relative postion

  geometry_msgs::PointStamped relative_position; // added for relative position
  geometry_msgs::QuaternionStamped current_attitude; // add for the attitude

  bool loaded_ {false};
  bool referenceSet_ = false;
  bool at_target_result = false;
  sensor_msgs::NavSatFix gps_position_;
  double altitude_ {0.0};
  float target_offset_x{0.0};
  float target_offset_y{0.0};
  float target_offset_z{0.0};
  float target_yaw{0.0};
  uint8_t flight_status = {255};
  uint8_t display_mode  = {255};
  uint8_t current_gps_health = {0};
  
  std::mutex position_mutex_;
  std::mutex home_mutex_;
  std::mutex altitude_mutex_;
  
  actionlib::SimpleActionServer<uav_msgs::SpecialMovementAction> special_mv_server_;
  actionlib::SimpleActionServer<uav_msgs::FlyToWPAction> fly_server_;
  actionlib::SimpleActionServer<uav_msgs::RelativePositionAction> relative_position_server_;


  std::string fly_action_name_ {"fcs/fly_to_wp"};
  std::string special_mv_action_name_ {"fcs/special_movement"};
   std::string relative_position_action_name_ {"fcs/relative_position"};

  uav_msgs::FlyToWPFeedback fly_feedback_;
  uav_msgs::SpecialMovementFeedback special_mv_feedback_;
  uav_msgs::RelativePositionFeedback relative_position_feedback_;

  uav_msgs::FlyToWPResult fly_result_;
  uav_msgs::SpecialMovementResult special_mv_result_;
  uav_msgs::RelativePositionResult relative_position_result_;
  

  const double take_off_height_ {1.2};
  const double height_precision_ {0.05};
  sensor_msgs::NavSatFix home_location_;
  bool home_position_initialised_ {false};

};

#endif //FCS_INTERFACE_H
