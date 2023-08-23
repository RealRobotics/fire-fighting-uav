#include "fcs_interface/fcs_interface.h"

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <dji_sdk/Activation.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/MissionWaypointTask.h>
#include <dji_sdk/MissionWpAction.h>
#include <dji_sdk/MissionWpUpload.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <djiosdk/dji_vehicle.hpp>
#include <mbzirc_msgs/LeedsCraneStatus.h>

#define UNLADEN_VELOCITY_M_PER_S (2.0)
#define LADEN_VELOCITY_M_PER_S (1.0)
#define VELOCITY_RANGE_M_PER_S (5.0)

FCS_Interface::FCS_Interface(ros::NodeHandle node_handle)
: node_handle_(node_handle) {
}

/** Start the OSDK client.
 * @return true if successful, false otherwise.
 */
bool FCS_Interface::start()
{
  // Set up clients on DJI OSDK node.
  control_authority_client_ = node_handle_.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
  drone_activation_client_ = node_handle_.serviceClient<dji_sdk::Activation>("dji_sdk/activation");
  drone_task_client_ = node_handle_.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  waypoint_action_client_ = node_handle_.serviceClient<dji_sdk::MissionWpAction>("dji_sdk/mission_waypoint_action");
  waypoint_upload_client_ = node_handle_.serviceClient<dji_sdk::MissionWpUpload>("dji_sdk/mission_waypoint_upload");
  set_local_pos_reference_client_ = node_handle_.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");
  // Subscribe to GPS location.
  gps_position_subscriber_ = node_handle_.subscribe<sensor_msgs::NavSatFix>("dji_sdk/gps_position", 10,
                                                                            &FCS_Interface::gpsPositionCallback_, this);
  // Subscribe to crane status.
  //TODO Lenka - think if FCS should have direct access to crane, commented out for now
  //crane_status_subscriber_ = node_handle_.subscribe("leeds_crane/status", 100, &OsdkClient::craneStatusCallback, this);

  prepare_service_ = node_handle_.advertiseService("fcs/prepare_drone",  &FCS_Interface::getReady, this);
  takeoff_service_ = node_handle_.advertiseService("fcs/take_off",  &FCS_Interface::takeOff, this);
  land_service_ = node_handle_.advertiseService("fcs/land",  &FCS_Interface::land, this);
  home_service_ = node_handle_.advertiseService("fcs/return_home",  &FCS_Interface::returnHome, this);

  ROS_INFO("Started FCS_Interface.");
   //TODO LEnka - think if this method can fail and if yes, when and return false
  return true;
}

void FCS_Interface::stop() {
  //TODO Lenka implement
  ROS_INFO("Stopped FCS_Interface.");
}

bool FCS_Interface::getReady(uav_msgs::PrepareDrone::Request  &req, 
  uav_msgs::PrepareDrone::Response &res) {
  ROS_INFO("FCS_Interface getting ready...");
  bool result = activate_();
  if (result) {
    result = obtainControlAuthority_();
    if (result) {
      ROS_INFO("FCS_Interface is ready for action!");
    }
  }
  res.result = result;
  return result;
}

bool FCS_Interface::takeOff(uav_msgs::TakeOff::Request  &req, 
  uav_msgs::TakeOff::Response &res)
{
  ROS_INFO("Taking off to %f metres.", req.altitude);
  // Get off the ground.
  bool result = droneTaskControl_(kTaskTakeOff);
  if (result)
  {
    // Now that we have taken off, create nav_sat_fix using the current GPS
    // location plus the requested altitude and upload that.
    //TODO Lenka - when is the gps_position_ initialised?
    sensor_msgs::NavSatFix nav_sat_fix;
    nav_sat_fix.altitude = gps_position_.altitude + req.altitude;
    nav_sat_fix.latitude = gps_position_.latitude;
    nav_sat_fix.longitude = gps_position_.longitude;
    // Upload waypoint.
    result = uploadNavSatFix_(nav_sat_fix);
  }
  else
  {
    ROS_WARN("Take off failed");
  }
  res.result = result;
  return result;
}

bool FCS_Interface::land(uav_msgs::Land::Request  &req, 
  uav_msgs::Land::Response &res)
{
  ROS_INFO("Landing...");
  bool result = droneTaskControl_(kTaskLand);
  res.result = result;
  return result;
}

bool FCS_Interface::returnHome(uav_msgs::ReturnHome::Request  &req, 
  uav_msgs::ReturnHome::Response &res) {
  ROS_INFO("Returning home...");
  bool result = droneTaskControl_(kTaskGoHome);
  res.result = result;
  return result;
}

bool FCS_Interface::setWaypoint(const sensor_msgs::NavSatFix& nav_sat_fix)
{
  ROS_INFO("Sending waypoint: %f, %f, %f", nav_sat_fix.latitude, nav_sat_fix.longitude, nav_sat_fix.altitude);
  bool result = uploadNavSatFix_(nav_sat_fix);
  return result;
}

bool FCS_Interface::holdPosition()
{
  ROS_INFO("Starting to hold position...");
  bool result = waypointMissionAction_(kActionStop);
  ROS_INFO("Holding position.");
  return result;
}

bool FCS_Interface::activate_()
{
  dji_sdk::Activation activation;
  drone_activation_client_.call(activation);
  auto response = activation.response;
  bool result = response.result;
  if (result)
  {
    ROS_INFO("Activated drone successfully.");
  }
  else
  {
    ROS_WARN("activate failed.");
    ROS_WARN("ack.info: set = %i id = %i", response.cmd_set, response.cmd_id);
    ROS_WARN("ack.data: %i", response.ack_data);
  }
  return result;
}

void FCS_Interface::convertToWaypoint_(const sensor_msgs::NavSatFix& nav_sat_fix, dji_sdk::MissionWaypoint* waypoint)
{
  // Convert the nav_sat_fix to a mission waypoint.
  // From demo_mission::uploadWaypoints()
  waypoint->latitude = nav_sat_fix.latitude;
  waypoint->longitude = nav_sat_fix.longitude;
  waypoint->altitude = nav_sat_fix.altitude;
  waypoint->damping_distance = 0;
  waypoint->target_yaw = 0;
  waypoint->target_gimbal_pitch = 0;
  waypoint->turn_mode = 0;
  waypoint->has_action = 0;
  ROS_INFO("Waypoint created at: %f \t%f \t%f\n ", waypoint->latitude, waypoint->longitude, waypoint->altitude);
}

/*void OsdkClient::craneStatusCallback(const mbzirc_msgs::LeedsCraneStatus::ConstPtr& message)
{
  loaded_ = message->gripperOn;
}*/

bool FCS_Interface::droneTaskControl_(DroneTask task)
{
  bool result = false;
  dji_sdk::DroneTaskControl droneTaskControl;
  switch(task) {
    case kTaskTakeOff:
      droneTaskControl.request.task = dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF;
      break;
    case kTaskLand:
      droneTaskControl.request.task = dji_sdk::DroneTaskControl::Request::TASK_LAND;
      break;
    case kTaskGoHome:
      droneTaskControl.request.task = dji_sdk::DroneTaskControl::Request::TASK_GOHOME;
      break;
  }

  drone_task_client_.call(droneTaskControl);
  result = droneTaskControl.response.result;
  if (!result)
  {
    ROS_WARN("droneTaskControl failed.");
    ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set, droneTaskControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
  }
  return result;
}

void FCS_Interface::gpsPositionCallback_(const sensor_msgs::NavSatFix::ConstPtr& message)
{
  gps_position_ = *message;
}

bool FCS_Interface::obtainControlAuthority_()
{
  bool result = false;
  // Obtain Control Authority
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable = 1;
  control_authority_client_.call(authority);
  if (authority.response.result)
  {
    ROS_INFO("Obtained SDK control authority successfully");
    result = true;
  }
  else
  {
    ROS_WARN("Failed to obtain SDK control authority");
  }
  return result;
}

void FCS_Interface::setWaypointInitDefaults_(dji_sdk::MissionWaypointTask* waypointTask)
{
  waypointTask->velocity_range = 10;
  waypointTask->idle_velocity = 5;
  waypointTask->action_on_finish = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
  waypointTask->mission_exec_times = 1;
  waypointTask->yaw_mode = dji_sdk::MissionWaypointTask::YAW_MODE_AUTO;
  waypointTask->trace_mode = dji_sdk::MissionWaypointTask::TRACE_POINT;
  waypointTask->action_on_rc_lost = dji_sdk::MissionWaypointTask::ACTION_AUTO;
  waypointTask->gimbal_pitch_mode = dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;
}

bool FCS_Interface::uploadNavSatFix_(const sensor_msgs::NavSatFix& nav_sat_fix)
{
  bool result = false;
  // Waypoint Mission : Initialization
  dji_sdk::MissionWaypointTask waypointTask;
  setWaypointInitDefaults_(&waypointTask);
  dji_sdk::MissionWaypoint waypoint;
  convertToWaypoint_(nav_sat_fix, &waypoint);
  waypointTask.mission_waypoint.push_back(waypoint);
  // Initialise the waypoint mission.
  dji_sdk::MissionWpUpload missionWaypointUpload;
  missionWaypointUpload.request.waypoint_task = waypointTask;
  waypoint_upload_client_.call(missionWaypointUpload);
  if (!missionWaypointUpload.response.result)
  {
    ROS_WARN("Failed sending mission upload command");
    ROS_WARN("ack.info: set = %i id = %i", missionWaypointUpload.response.cmd_set,
             missionWaypointUpload.response.cmd_id);
    ROS_WARN("ack.data: %i", missionWaypointUpload.response.ack_data);
  }
  else
  {
    ROS_INFO("Waypoint upload command sent successfully");
    // Start the mission.
    result = waypointMissionAction_(kActionStart);
    if (result)
    {
      ROS_INFO("Mission start command sent successfully");
    }
    else
    {
      ROS_WARN("Failed sending mission start command");
    }
  }

  return result;
}

bool FCS_Interface::waypointMissionAction_(WaypointAction action)
{
  bool result = false;
  dji_sdk::MissionWpAction wp_action;
  switch (action)
  {
    case kActionStart:
      wp_action.request.action = dji_sdk::MissionWpAction::Request::ACTION_START;
      break;
    case kActionStop:
      wp_action.request.action = dji_sdk::MissionWpAction::Request::ACTION_STOP;
      break;
    case kActionPause:
      wp_action.request.action = dji_sdk::MissionWpAction::Request::ACTION_PAUSE;
      break;
    case kActionResume:
      wp_action.request.action = dji_sdk::MissionWpAction::Request::ACTION_RESUME;
      break;
    default:
      ROS_WARN("Waypoint action not handled, %d", action);
      break;
  }
  waypoint_action_client_.call(wp_action);
  if (!wp_action.response.result)
  {
    ROS_WARN("waypointMissionAction failed.");
    ROS_WARN("ack.info: set = %i id = %i", wp_action.response.cmd_set, wp_action.response.cmd_id);
    ROS_WARN("ack.data: %i", wp_action.response.ack_data);
  }
  return result;
}
