#ifndef FCS_INTERFACE_H
#define FCS_INTERFACE_H

#include <mutex> 

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <actionlib/server/simple_action_server.h>

#include <mbzirc_msgs/LeedsCraneStatus.h>
#include <dji_sdk/MissionWaypointTask.h>

#include "uav_msgs/PrepareDrone.h"
#include "uav_msgs/FlyToWPAction.h"
#include "uav_msgs/SpecialMovementAction.h"

class FCS_Interface
{
public:
  FCS_Interface(ros::NodeHandle node_handle);

  /** Connects to the drone autopilot and prepares for any other commands.
   * @return true on success.
   */
  bool start();

  /** Disarms the drone.
   */
  void stop();

  /** After this call, the drone will be ready to move.
   * Blocks until done.
   * @return true on success.
   */
  bool getReady(uav_msgs::PrepareDrone::Request  &req, 
  uav_msgs::PrepareDrone::Response &res);

  /** Causes the drone to hold its current position and altitude.
   * Blocks until holding position (normally returns very quickly).
   * @return true on success.
   */
  bool holdPosition();

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
  void convertToWaypoint_(const sensor_msgs::NavSatFix& nav_sat_fix, dji_sdk::MissionWaypoint* waypoint);
  void craneStatusCallback(const mbzirc_msgs::LeedsCraneStatus::ConstPtr& message);
  bool droneTaskControl_(DroneTask task);
  void gpsPositionCallback_(const sensor_msgs::NavSatFix::ConstPtr& message);
  bool obtainControlAuthority_();
  void setWaypointInitDefaults_(dji_sdk::MissionWaypointTask* waypointTask);
  bool uploadNavSatFix_(const sensor_msgs::NavSatFix& nav_sat_fix);
  bool waypointMissionAction_(WaypointAction action);
  bool droneWithinRadius_(double radius, sensor_msgs::NavSatFix goal);
  bool setWaypoint_(const uav_msgs::FlyToWPGoalConstPtr &goal);
  bool specialMovement_(const uav_msgs::SpecialMovementGoalConstPtr &goal);


  bool loaded_ {false};
  ros::NodeHandle node_handle_;
  ros::ServiceClient control_authority_client_;
  ros::ServiceClient drone_activation_client_;
  ros::ServiceClient drone_task_client_;
  ros::ServiceClient set_local_pos_reference_client_;
  ros::ServiceClient waypoint_action_client_;
  ros::ServiceClient waypoint_upload_client_;
  ros::Subscriber gps_position_subscriber_;

  sensor_msgs::NavSatFix gps_position_;
  double altitude_ {0.0};
  
  std::mutex position_mutex_;
  std::mutex altitude_mutex_;
  
  ros::Subscriber crane_status_subscriber_; //TODO check if this is needed here

  ros::ServiceServer prepare_service_;
  actionlib::SimpleActionServer<uav_msgs::SpecialMovementAction> special_mv_server_;
  actionlib::SimpleActionServer<uav_msgs::FlyToWPAction> fly_server_;
  std::string fly_action_name_ {"fcs/fly_to_wp"};
  std::string special_mv_action_name_ {"fcs/special_movement"};
  uav_msgs::FlyToWPFeedback fly_feedback_;
  uav_msgs::FlyToWPResult fly_result_;
  uav_msgs::SpecialMovementFeedback special_mv_feedback_;
  uav_msgs::SpecialMovementResult special_mv_result_;
};

#endif //FCS_INTERFACE_H
