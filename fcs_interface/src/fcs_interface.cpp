#include "fcs_interface/fcs_interface.h"
#include "uav_msgs/SpecialMovement.h"
#include "uav_msgs/BatteryPercentage.h"
#include "uav_msgs/RelativePositionAction.h"
#include <tf/transform_datatypes.h>

#include <chrono>
#include <cmath>
#include <future> 
#include <vector> 

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

#define UNLADEN_VELOCITY_M_PER_S (2.0)
#define LADEN_VELOCITY_M_PER_S (1.0)
#define VELOCITY_RANGE_M_PER_S (5.0)

// FCS_Interface::FCS_Interface(ros::NodeHandle node_handle)
// : node_handle_(node_handle), fly_server_(node_handle, "fcs_interface/fly_to_wp", boost::bind(&FCS_Interface::setWaypoint_, this, _1), false),
//   special_mv_server_(node_handle, "fcs_interface/special_movement", boost::bind(&FCS_Interface::specialMovement_, this, _1), false) {
// }

FCS_Interface::FCS_Interface(ros::NodeHandle node_handle)
  : node_handle_(node_handle),
    fly_server_(node_handle, "fcs_interface/fly_to_wp", boost::bind(&FCS_Interface::setWaypoint_, this, _1), false),
    special_mv_server_(node_handle, "fcs_interface/special_movement", boost::bind(&FCS_Interface::specialMovement_, this, _1), false),
    relative_position_server_(node_handle, "fcs_interface/relative_position", boost::bind(&FCS_Interface::setTarget_relative_position_, this, _1), false) {
}

bool FCS_Interface::start() {
  // Set up clients on DJI OSDK node.
  ROS_INFO("Starting the FCS_Interface.");
  control_authority_client_ = node_handle_.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
  drone_activation_client_ = node_handle_.serviceClient<dji_sdk::Activation>("dji_sdk/activation");
  drone_task_client_ = node_handle_.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
  waypoint_action_client_ = node_handle_.serviceClient<dji_sdk::MissionWpAction>("dji_sdk/mission_waypoint_action");
  waypoint_upload_client_ = node_handle_.serviceClient<dji_sdk::MissionWpUpload>("dji_sdk/mission_waypoint_upload");
  set_local_pos_reference    = node_handle_.serviceClient<dji_sdk::SetLocalPosRef> ("dji_sdk/set_local_pos_ref");
  // Subscribe to DJI OSDK topics
  gps_position_subscriber_ = node_handle_.subscribe<sensor_msgs::NavSatFix>("dji_sdk/gps_position", 10,
                                                                            &FCS_Interface::gpsPositionCallback_, this);
  battery_state_subscriber_ = node_handle_.subscribe<sensor_msgs::BatteryState>("dji_sdk/battery_state", 10,
                                                                            &FCS_Interface::batteryStateCallback_, this);

  local_position_subscriber_ = node_handle_.subscribe("dji_sdk/local_position", 10, &FCS_Interface::relative_position_Callback_, this);
  gps_health_subscriber_      = node_handle_.subscribe("dji_sdk/gps_health", 10, &FCS_Interface::gps_health_Callback_,this);
  altitude_subscriber_ = node_handle_.subscribe<std_msgs::Float32>("dji_sdk/height_above_takeoff", 10,
                                                                            &FCS_Interface::altitudeCallback_, this);

  //set up publishing topics
  battery_state_publisher_ = node_handle_.advertise<uav_msgs::BatteryPercentage>("fcs_interface/battery_state", 10);

  ctrlPosYawPub = node_handle_.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_ENUposition_yaw", 10);

  //start the action servers
  fly_server_.start();
  special_mv_server_.start();
  relative_position_server_.start();
  
  //finally activate the drone, get control and wait for the home_location to be initialised (i.e. obtain first gps location)
  bool result = getReady_();
  if (result) {
   while(ros::ok()) {
      home_mutex_.lock();
      if (home_position_initialised_) {
        home_mutex_.unlock();
        break;
      }
      home_mutex_.unlock();
      ros::spinOnce();
      ros::Duration(0.1).sleep();
      ROS_INFO("Waiting to obtain first gps reading");
    }
    ROS_INFO("Started FCS_Interface.");
  }
  return result;
}

bool FCS_Interface::activate_() {
  dji_sdk::Activation activation;
  drone_activation_client_.call(activation);
  auto response = activation.response;
  bool result = response.result;
  if (result) {
    ROS_INFO("Activated drone successfully.");
  } else {
    ROS_WARN("activate failed.");
    ROS_WARN("ack.info: set = %i id = %i", response.cmd_set, response.cmd_id);
    ROS_WARN("ack.data: %i", response.ack_data);
  }
  return result;
}

bool FCS_Interface::obtainControlAuthority_() {
  bool result = false;
  // Obtain Control Authority
  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable = 1;
  control_authority_client_.call(authority);
  if (authority.response.result) {
    ROS_INFO("Obtained SDK control authority successfully");
    result = true;
  } else {
    ROS_WARN("Failed to obtain SDK control authority");
  }
  return result;
}

bool FCS_Interface::getReady_() {
  ROS_INFO("FCS_Interface getting ready...");
  bool result = activate_();
  if (result) {
    result = obtainControlAuthority_();
  }
  return result;
}

bool FCS_Interface::droneTaskControl_(DroneTask task) {
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
  if (!result) {
    ROS_WARN("droneTaskControl failed.");
    ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set, droneTaskControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
  }
  return result;
}

bool FCS_Interface::specialMovement_(const uav_msgs::SpecialMovementGoalConstPtr &goal) {
  bool result {false};
  double desired_height;

  if(goal->movement.data == uav_msgs::SpecialMovement::TAKE_OFF) {
    ROS_INFO("Taking off...");
    home_mutex_.lock();
    desired_height = take_off_height_;
    home_mutex_.unlock();
    result = droneTaskControl_(kTaskTakeOff);
  } else if (goal->movement.data == uav_msgs::SpecialMovement::LAND) {
    ROS_INFO("Landing...");
    home_mutex_.lock();
    desired_height = 0;
    home_mutex_.unlock();
    result = droneTaskControl_(kTaskLand);
  } else if (goal->movement.data == uav_msgs::SpecialMovement::GO_HOME) {
    ROS_INFO("Returning home...");
    result = droneTaskControl_(kTaskGoHome);
    desired_height = 0;
  }

  bool preempted {false};
  altitude_mutex_.lock();
  double height_error = std::abs(desired_height - altitude_);
  altitude_mutex_.unlock();

  ros::Duration feedback_period(0.1);

  while( (height_error > height_precision_) && ros::ok()) {

    if(position_mutex_.try_lock()) {
      special_mv_feedback_.current_location = gps_position_;
      position_mutex_.unlock();
      altitude_mutex_.lock();
      special_mv_feedback_.current_location.altitude = altitude_;
      altitude_mutex_.unlock();
      special_mv_server_.publishFeedback(special_mv_feedback_);
    }

    if(special_mv_server_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", special_mv_action_name_.c_str());
      special_mv_server_.setPreempted();
      preempted = true;
      break;
    }

    altitude_mutex_.lock();
    height_error = std::abs(desired_height - altitude_);
    altitude_mutex_.unlock();

    feedback_period.sleep();
    ros::spinOnce(); //TODO do i needd the spinning here?
  }

  if(!preempted) {
    if(!result) {
      ROS_WARN("Request for special movement has failed");
      special_mv_result_.done = false;
      special_mv_server_.setAborted(special_mv_result_); //consider sending a text msg as well to differentiate the reasons
      return false;
    } else {
      special_mv_result_.done = true;
      ROS_INFO("%s: Succeeded", special_mv_action_name_.c_str());
      special_mv_server_.setSucceeded(special_mv_result_);
      return true;
    }
  } else {
    special_mv_result_.done = false;
    ROS_WARN("%s: Preempted", fly_action_name_.c_str());
    return false;
  }
}

bool FCS_Interface::setWaypoint_(const uav_msgs::FlyToWPGoalConstPtr &goal)
{
  sensor_msgs::NavSatFix nav_sat_fix = goal->goal.location;
  ROS_INFO("Sending waypoint: %f, %f, %f", nav_sat_fix.latitude, nav_sat_fix.longitude, nav_sat_fix.altitude);
  bool result = uploadNavSatFix_(nav_sat_fix);

  if(!result) {
    ROS_WARN("Request to fly to WP has failed");
    fly_result_.in_location = false;
    fly_server_.setAborted(fly_result_); //consider sending a text msg as well to differentiate the reasons
  } else {
    bool preempted {false};
    ros::Duration feedback_period(0.1);
    while(!droneWithinRadius_(goal->goal.loc_precision, nav_sat_fix) && ros::ok()) 
    {

      if(position_mutex_.try_lock()) {
        fly_feedback_.current_location = gps_position_;
        position_mutex_.unlock();
        altitude_mutex_.lock();
        fly_feedback_.current_location.altitude = altitude_;
        altitude_mutex_.unlock();
        fly_server_.publishFeedback(fly_feedback_);
      }

      if(fly_server_.isPreemptRequested() || !ros::ok()) {
        ROS_INFO("%s: Preempted", fly_action_name_.c_str());
        fly_server_.setPreempted();
        preempted = true;
        break;
      }

      feedback_period.sleep();
      ros::spinOnce(); //TODO do i needd the spinning here?
    }

    if(!preempted) 
    {
      fly_result_.in_location = true;
      ROS_INFO("%s: Succeeded", fly_action_name_.c_str());
      // Stop the mission.
      if (waypointMissionAction_(kActionStop))
      {
        ROS_INFO("Mission STOP command sent successfully");
      } 
      else 
      {
        ROS_WARN("Failed sending mission STOP command sending Stop command again");
        if (waypointMissionAction_(kActionStop))
        {
          ROS_INFO("Mission STOP command sent successfully");
        } 
        else 
        {
          ROS_WARN("Failed sending mission STOP command sending again");
        
        }
      }
      fly_server_.setSucceeded(fly_result_);
     } 
    else 
    {
      fly_result_.in_location = false;
      ROS_WARN("%s: Preempted", fly_action_name_.c_str());
    }
  }
  return result;
}

//TODO return the wp rather than using reference
void FCS_Interface::convertToWaypoint_(const sensor_msgs::NavSatFix& nav_sat_fix, dji_sdk::MissionWaypoint & waypoint) {
  // Convert the nav_sat_fix to a mission waypoint.
  // From demo_mission::uploadWaypoints()
  waypoint.latitude = nav_sat_fix.latitude;
  waypoint.longitude = nav_sat_fix.longitude;
  waypoint.altitude = nav_sat_fix.altitude;
  waypoint.damping_distance = 0;
  waypoint.target_yaw = 0;
  waypoint.target_gimbal_pitch = 0;
  waypoint.turn_mode = 0;
  waypoint.has_action = 0;
  ROS_INFO("Waypoint created at: %f \t%f \t%f\n ", waypoint.latitude, waypoint.longitude, waypoint.altitude);
}

void FCS_Interface::setWaypointInitDefaults_(dji_sdk::MissionWaypointTask & waypointTask) {
  waypointTask.velocity_range = 10;
  waypointTask.idle_velocity = 5;
  waypointTask.action_on_finish = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
  waypointTask.mission_exec_times = 1;
  waypointTask.yaw_mode = dji_sdk::MissionWaypointTask::YAW_MODE_AUTO;
  waypointTask.trace_mode = dji_sdk::MissionWaypointTask::TRACE_POINT;
  waypointTask.action_on_rc_lost = dji_sdk::MissionWaypointTask::ACTION_AUTO;
  waypointTask.gimbal_pitch_mode = dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;
}

sensor_msgs::NavSatFix FCS_Interface::generate_mid_point_(const sensor_msgs::NavSatFix& nav_sat_fix) {
  sensor_msgs::NavSatFix mid_wp;
  position_mutex_.lock();
  double latitude_diff = (nav_sat_fix.latitude - gps_position_.latitude)/2.0;
  double longitude_diff = (nav_sat_fix.longitude - gps_position_.longitude)/2.0;
  altitude_mutex_.lock();
  double altitude_diff = (nav_sat_fix.altitude - altitude_)/2.0;
  altitude_mutex_.unlock();
  position_mutex_.unlock();

  mid_wp.latitude = nav_sat_fix.latitude - latitude_diff;
  mid_wp.longitude = nav_sat_fix.longitude - longitude_diff;
  mid_wp.altitude = nav_sat_fix.altitude - altitude_diff;

  return mid_wp;
}


sensor_msgs::NavSatFix FCS_Interface::generate_d_point_(const sensor_msgs::NavSatFix& nav_sat_fix, const u_int8_t point_number)
{
  sensor_msgs::NavSatFix adjusted_wp;
  position_mutex_.lock();
  double latitude_diff = (nav_sat_fix.latitude - gps_position_.latitude) / 2.0;
  double longitude_diff = (nav_sat_fix.longitude - gps_position_.longitude) / 2.0;
  altitude_mutex_.lock();
  double altitude_diff = (nav_sat_fix.altitude - altitude_) / 2.0;
  altitude_mutex_.unlock();
  position_mutex_.unlock();

  adjusted_wp.latitude = nav_sat_fix.latitude - latitude_diff;
  adjusted_wp.longitude = nav_sat_fix.longitude - longitude_diff;
  adjusted_wp.altitude = nav_sat_fix.altitude - altitude_diff;

  // Adjust latitude and longitude based on point_number
  double adjustment = 0.0;
  if (point_number == 1) 
  {
    adjustment = 6.0 / (111.32 * 1000.0);  // Add approximately 6 meters in latitude
  } else if (point_number == 2) 
  {
    adjustment = -6.0 / (111.32 * 1000.0); // Add approximately -6 meters in latitude
  }
  adjusted_wp.latitude -= adjustment;
  adjusted_wp.longitude -= adjustment / std::cos(adjusted_wp.latitude * M_PI / 180.0);  // Adjust longitude based on latitude

  return adjusted_wp;
}

bool FCS_Interface::uploadNavSatFix_(const sensor_msgs::NavSatFix& nav_sat_fix) {
  bool result = false;
  // Waypoint Mission : Initialization
  dji_sdk::MissionWaypointTask waypointTask;
  setWaypointInitDefaults_(waypointTask);

  dji_sdk::MissionWaypoint waypoint;
  position_mutex_.lock();
  altitude_mutex_.lock();
  gps_position_.altitude = altitude_;
  altitude_mutex_.unlock();
  // convertToWaypoint_(gps_position_, waypoint);
  position_mutex_.unlock();
  // waypointTask.mission_waypoint.push_back(waypoint);

    convertToWaypoint_(nav_sat_fix, waypoint);
    waypointTask.mission_waypoint.push_back(waypoint);

   convertToWaypoint_(generate_d_point_(nav_sat_fix,1), waypoint);
   waypointTask.mission_waypoint.push_back(waypoint);

   convertToWaypoint_(generate_d_point_(nav_sat_fix,2), waypoint);
   waypointTask.mission_waypoint.push_back(waypoint);

  // Initialise the waypoint mission.
  dji_sdk::MissionWpUpload missionWaypointUpload;
  missionWaypointUpload.request.waypoint_task = waypointTask;
  waypoint_upload_client_.call(missionWaypointUpload);
  if (!missionWaypointUpload.response.result) {
    ROS_WARN("Failed sending mission upload command");
    ROS_WARN("ack.info: set = %i id = %i", missionWaypointUpload.response.cmd_set,
             missionWaypointUpload.response.cmd_id);
    ROS_WARN("ack.data: %i", missionWaypointUpload.response.ack_data);
  } else {
    ROS_INFO("Waypoint upload command sent successfully");
    // Start the mission.
    result = waypointMissionAction_(kActionStart);
    if (result) {
      ROS_INFO("Mission start command sent successfully");
    } else {
      ROS_WARN("Failed sending mission start command");
    }
  }

  return result;
}

bool FCS_Interface::waypointMissionAction_(WaypointAction action) {
  bool result = false;
  dji_sdk::MissionWpAction wp_action;
  switch (action) {
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
  result = waypoint_action_client_.call(wp_action);
  if (!wp_action.response.result) {
    ROS_WARN("waypointMissionAction failed.");
    ROS_WARN("ack.info: set = %i id = %i", wp_action.response.cmd_set, wp_action.response.cmd_id);
    ROS_WARN("ack.data: %i", wp_action.response.ack_data);
  }
  return result;
}

bool FCS_Interface::droneWithinRadius_(double radius, sensor_msgs::NavSatFix goal) {
  //https://en.wikipedia.org/wiki/Haversine_formula

  double R = 6378.137; // Radius of earth in KM
  position_mutex_.lock();
  double lat1 = gps_position_.latitude;
  double lon1 = gps_position_.longitude;
  position_mutex_.unlock();

  double lat2 = goal.latitude;
  double lon2 = goal.longitude;

  double dLat = lat2 * M_PI / 180 - lat1 * M_PI / 180;
  double dLon = lon2 * M_PI / 180 - lon1 * M_PI / 180;
  
  double a = sin(dLat/2) * sin(dLat/2) +
    cos(lat1 * M_PI / 180) * cos(lat2 * M_PI / 180) *
    sin(dLon/2) * sin(dLon/2);

  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  double d = R * c * 1000; //meters

  ROS_INFO("the current distance is %f", d);

  if (d <= radius) {
    return true;
  } else {
    return false;
  }
}

//TODO use gps health to trust the data only if health is good
void FCS_Interface::gpsPositionCallback_(const sensor_msgs::NavSatFix::ConstPtr& message) {
  static int num_runs = 0;
  if (num_runs == 0) {
    home_mutex_.lock();
    home_location_ = *message;
    home_position_initialised_ = true;
    home_mutex_.unlock();
    num_runs++;
  } 

  position_mutex_.lock();
  gps_position_ = *message;
  position_mutex_.unlock();
}

void FCS_Interface::batteryStateCallback_(const sensor_msgs::BatteryState::ConstPtr& message) {
  static int num_runs = 0;
  uav_msgs::BatteryPercentage msg;
  msg.input_msg_id  = num_runs;
  int perc = int(message->percentage);
  if (perc < 0) {
    perc = 0;
  }
  if (perc > 100) {
    perc = 100;
  }
  msg.percentage = perc;
  msg.stamp = ros::Time::now();
  battery_state_publisher_.publish(msg);
  num_runs++;
}

void FCS_Interface::altitudeCallback_(const std_msgs::Float32::ConstPtr& message) {
  altitude_mutex_.lock();
  altitude_ = message->data;
  altitude_mutex_.unlock();
}

void FCS_Interface::setTarget_relative_position_(const uav_msgs::RelativePositionGoalConstPtr &goal) // added for relative postion
{
    uav_msgs::RelativePosition relative_position_goal = goal->goal;
  
    target_offset_x = relative_position_goal.xv_relative;
    target_offset_y = relative_position_goal.yv_relative;
    target_offset_z= relative_position_goal.zv_relative;
    target_yaw = relative_position_goal.yawv_relative;

    // current_attitude is a geometry_msgs::Quaternion message
    tf::Quaternion q(
    current_attitude.quaternion.x,
    current_attitude.quaternion.y,
    current_attitude.quaternion.z,
    current_attitude.quaternion.w
  );

    tf::Matrix3x3 m(q);

    double roll, pitch, initial_yaw,yawC=0;
    m.getRPY(roll, pitch, initial_yaw);

    if (current_gps_health > 3) 
    {
      if (set_reference_relative_position_())
      {
        relative_position_ctrl_(relative_position.point.x, relative_position.point.y, relative_position.point.z, yawC);
        double error_x = target_offset_x;
        double error_y = target_offset_y;
        double error_yaw = target_yaw ;
        double yaw_offset_InertialToBody = initial_yaw - yawC;
        double yawC = yaw_offset_InertialToBody - error_yaw;
        double xC =  cos(yaw_offset_InertialToBody) * error_x - sin(yaw_offset_InertialToBody) * error_y;
        double yC =  sin(yaw_offset_InertialToBody) * error_x + cos(yaw_offset_InertialToBody) * error_y;
        double xCom= xC + relative_position.point.x;
        double yCom= yC + relative_position.point.y;
        relative_position_ctrl_(xCom, yCom , relative_position.point.z, yawC);
      }
      else 
      {
          ROS_INFO("Local Reference not Set");
          relative_position_server_.setAborted();
      }
    }
    else
    {
      ROS_INFO("Cannot execute Local Position Control");
      ROS_INFO("Not enough GPS Satellites");
      relative_position_server_.setAborted();
    }
}

/*!
 * This function calculates the difference between camera target and current  position
 * and sends the commands to the Position and Yaw control topic.
 *
 */
void FCS_Interface::relative_position_ctrl_(double &xCmd, double &yCmd, double &zCmd, double &yawCmd)
{
  const float pos_threshold = 0.1;
  bool at_target = false;

  while (!at_target && ros::ok())
  {
    if (current_gps_health > 3)
    {
      xCmd = target_offset_x - relative_position.point.x;
      yCmd = target_offset_y - relative_position.point.y;
      zCmd = target_offset_z;

      sensor_msgs::Joy controlPosYaw;
      controlPosYaw.axes.push_back(xCmd);
      controlPosYaw.axes.push_back(yCmd);
      controlPosYaw.axes.push_back(zCmd);
      controlPosYaw.axes.push_back(target_yaw);
      ctrlPosYawPub.publish(controlPosYaw);
    }
    else
    {
      ROS_INFO("Cannot execute Relative Position Control");
      ROS_INFO("Not enough GPS Satellites");
      relative_position_result_.at_target = false;

       break;  // exit the loop if GPS health is not sufficient
    }

    bool ck1 = (std::abs(xCmd) > pos_threshold);
    bool ck2 = (std::abs(yCmd) > pos_threshold);
    bool ck3 = (relative_position.point.z < (target_offset_z - pos_threshold));
    bool ck4 = (relative_position.point.z > (target_offset_z + pos_threshold));

    at_target = !(ck1 || ck2 || ck3 || ck4);
   
    ros::Duration(0.02).sleep();
    ros::spinOnce();
  }

  if(relative_position_result_.at_target = at_target)
  {
  relative_position_server_.setSucceeded(relative_position_result_);
  }
  else
  {
    relative_position_server_.setAborted(relative_position_result_);
  }
}


void FCS_Interface::gps_health_Callback_(const std_msgs::UInt8::ConstPtr& msg) {
  current_gps_health = msg->data;
  // ROS_INFO("Current GPS Health: %d", static_cast<int>(current_gps_health));
}

void FCS_Interface::flight_status_Callback_(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void FCS_Interface::display_mode_Callback_(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}

/**
 * Sets the local position reference if not already set and returns true if successful, false otherwise.
 * Updates the internal flag 'referenceSet_' to reflect the state of the local position reference.
 */
bool FCS_Interface::set_reference_relative_position_()
{
    if (!referenceSet_)
    {
        dji_sdk::SetLocalPosRef localPosReferenceSetter;
        set_local_pos_reference.call(localPosReferenceSetter);

        // Update the flag to indicate that the reference has been set
        referenceSet_ = static_cast<bool>(localPosReferenceSetter.response.result);

        // Return the result of the service call
        return referenceSet_;
    }

    // No need for an explicit else block, it will return referenceSet_ by default
    return referenceSet_;
}

/*!
 * This function is called when local position data is available.
 */
void FCS_Interface::relative_position_Callback_(const geometry_msgs::PointStamped::ConstPtr& msg) 
{
  relative_position = *msg;
}
void FCS_Interface::attitudeCallback_(const geometry_msgs::QuaternionStamped::ConstPtr& msg) {
  current_attitude = *msg;
}