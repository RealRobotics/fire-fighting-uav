#ifndef FCS_INTERFACE_H
#define FCS_INTERFACE_H

#include <mutex> 

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <actionlib/server/simple_action_server.h>

#include <mbzirc_msgs/LeedsCraneStatus.h>
#include <dji_sdk/MissionWaypointTask.h>

#include "uav_msgs/TakeOff.h"
#include "uav_msgs/Land.h"
#include "uav_msgs/ReturnHome.h"
#include "uav_msgs/PrepareDrone.h"
#include "uav_msgs/FlyToWPAction.h"

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

  /** This is a callback for a takeoff service. It causes the drone to takeoff and fly to the given altitude.
   * Blocks until done.
   * @param req Includes the altitude to fly to in metres.
   * @param res The response if the take off succeeded or failed
   * @return true on success.
   */
  bool takeOff(uav_msgs::TakeOff::Request  &req, 
  uav_msgs::TakeOff::Response &res);

  /** Causes the drone to land at the current position.
   * Blocks until done.
   * @return true on success.
   */
  bool land(uav_msgs::Land::Request  &req, 
  uav_msgs::Land::Response &res);

  /** Causes the drone to go to its home location.
   * Blocks until done.
   * @return true on success.
   */
  bool returnHome(uav_msgs::ReturnHome::Request  &req, 
  uav_msgs::ReturnHome::Response &res);

  /** Sends the given waypoint to the autopilot.
   * Blocks until the waypoint is reached.
   * @param nav_sat_fix The waypoint to set.
   * @return true on success.
   */
  bool setWaypoint(const uav_msgs::FlyToWPGoalConstPtr &goal);

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

  bool loaded_ {false};
  ros::NodeHandle node_handle_;
  ros::ServiceClient control_authority_client_;
  ros::ServiceClient drone_activation_client_;
  ros::ServiceClient drone_task_client_;
  ros::ServiceClient set_local_pos_reference_client_;
  ros::ServiceClient waypoint_action_client_;
  ros::ServiceClient waypoint_upload_client_;
  sensor_msgs::NavSatFix gps_position_;
  ros::Subscriber gps_position_subscriber_;
  
  std::mutex position_mutex_;
  
  ros::Subscriber crane_status_subscriber_; //TODO check if this is needed here

  ros::ServiceServer prepare_service_;
  ros::ServiceServer takeoff_service_;
  ros::ServiceServer land_service_;
  ros::ServiceServer home_service_;
  actionlib::SimpleActionServer<uav_msgs::FlyToWPAction> fly_server_;
  std::string fly_action_name_ {"fcs/fly_to_wp"};
  uav_msgs::FlyToWPFeedback fly_feedback_;
  uav_msgs::FlyToWPResult fly_result_;
};

#endif //FCS_INTERFACE_H
