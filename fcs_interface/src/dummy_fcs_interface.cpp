#include "fcs_interface/dummy_fcs_interface.h"

#include <future>  
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "uav_msgs/GpsLocationWithPrecision.h"

#include "uav_msgs/SpecialMovement.h"
#include "uav_msgs/BatteryPercentage.h"
#include "uav_msgs/WayPtSearch.h"

#include <chrono>
#include <cmath>
#include <future> 
#include <vector>
#include <json/json.h>
#include <fstream>
#include <ros/package.h>

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

int current_waypoint_index = 0; 

std::string jsonFilePath;

DummyFCS_Interface::DummyFCS_Interface(ros::NodeHandle node_handle)
: node_handle_(node_handle), 
fly_server_(node_handle, "fcs_interface/fly_to_wp", boost::bind(&DummyFCS_Interface::setWaypoints_, this, _1), false),
special_mv_server_(node_handle, "fcs_interface/special_movement", boost::bind(&DummyFCS_Interface::specialMovement_, this, _1), false) {
}

bool DummyFCS_Interface::start() 
{
  ROS_INFO("Starting the DummyFCS_Interface.");

  // // Subscribe to DJI OSDK topics
  // gps_position_subscriber_ = node_handle_.subscribe<sensor_msgs::NavSatFix>("dji_sdk/gps_position", 10,
  //                                                                           &DummyFCS_Interface::gpsPositionCallback_, this);
  // battery_state_subscriber_ = node_handle_.subscribe<sensor_msgs::BatteryState>("dji_sdk/battery_state", 10,
  //                                                                           &DummyFCS_Interface::batteryStateCallback_, this);

  // //set up publishing topics
  // battery_state_publisher_ = node_handle_.advertise<uav_msgs::BatteryPercentage>("fcs_interface/battery_state", 10);

  //start the action servers
  fly_server_.start();
  special_mv_server_.start();
  
  // //wait for the home_location to be initialised (i.e. obtain first gps location)
  // while(ros::ok()) {
  //   home_mutex_.lock();
  //   if (home_position_initialised_) 
  //   {
  //     home_mutex_.unlock();
  //     break;
  //   }
  //   home_mutex_.unlock();
    ros::Duration(5.0).sleep();
    ros::spinOnce();
  //  ROS_INFO("Waiting to obtain first gps reading");
  //}
  
  ROS_INFO("Started dummy FCS_Interface.");
  
  return true;
}

bool DummyFCS_Interface::droneTaskControl_(DroneTask task) 
{
  // bool result = false;
  // dji_sdk::DroneTaskControl droneTaskControl;

  // // Debugging: Print the task being performed
  // ROS_INFO("Performing drone task: %d", task);

  // switch(task) 
  // {
  //   case kTaskTakeOff:
  //     droneTaskControl.request.task = dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF;
  //     break;
  //   case kTaskLand:
  //     droneTaskControl.request.task = dji_sdk::DroneTaskControl::Request::TASK_LAND;
  //     break;
  //   case kTaskGoHome:
  //     droneTaskControl.request.task = dji_sdk::DroneTaskControl::Request::TASK_GOHOME;
  //     break;
  //   default:
  //     ROS_WARN("Unknown task requested: %d", task);
  //     return false;
  // }

  // // Debugging: Print the specific task being requested
  // ROS_INFO("Sending drone task request: %d", droneTaskControl.request.task);
  // // Adding a delay of 5 seconds after successful execution
  // ROS_INFO("Waiting for 1 seconds after drone task...");
  // ros::Duration(1.0).sleep();
  // // Make the service call
  // if (drone_task_client_.call(droneTaskControl)) 
  // {
  //   // Debugging: Print success message for the service call
  //   ROS_INFO("Service call successful");
  // } else {
  //   // Debugging: Print failure message for the service call
  //   ROS_ERROR("Failed to call drone task service");
  //   return false; // Exit early if the service call fails
  // }

  // // Debugging: Print the result from the response
  // if (droneTaskControl.response.result) 
  // {
  //   ROS_INFO("droneTaskControl successful.");
  // } else 
  // {
  //   ROS_WARN("droneTaskControl failed.");
  //   ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set, droneTaskControl.response.cmd_id);
  //   ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
  // }

  // return droneTaskControl.response.result;
  return true;
}

bool DummyFCS_Interface::specialMovement_(const uav_msgs::SpecialMovementGoalConstPtr &goal) 
{
  bool result {false};
  double desired_height;

 // Log the content of the received goal
  ROS_INFO("Received goal: movement = %d", goal->movement.data);

  // if(goal->movement.data == uav_msgs::SpecialMovement::TAKE_OFF) 
  // {
  //   ROS_INFO("Taking off...");
  //   // home_mutex_.lock();
  //   // desired_height = take_off_height_;
  //   // home_mutex_.unlock();
  //   // //result = droneTaskControl_(kTaskTakeOff);
  //   result = true;
  // } else if (goal->movement.data == uav_msgs::SpecialMovement::LAND) {
  //   ROS_INFO("Landing...");
  //   // home_mutex_.lock();
  //   // desired_height = 0;
  //   // home_mutex_.unlock();
  //   result = true;
  //   //result = droneTaskControl_(kTaskLand);
  // } else if (goal->movement.data == uav_msgs::SpecialMovement::GO_HOME) 
  // {
  //   ROS_INFO("Returning home...");
  //   //result = droneTaskControl_(kTaskGoHome);
  //   result = true;
  // }

  bool preempted {false};
  //altitude_mutex_.lock();
  //double height_error = std::abs(desired_height - altitude_);
  //altitude_mutex_.unlock();

  ros::Duration feedback_period(1);

  // while( ros::ok()) 
  // {

  //   // if(position_mutex_.try_lock()) 
  //   // {
  //   //   special_mv_feedback_.current_location = gps_position_;
  //   //   position_mutex_.unlock();
  //   //   special_mv_server_.publishFeedback(special_mv_feedback_);
  //   // }

  //   if(special_mv_server_.isPreemptRequested() || !ros::ok()) 
  //   {
  //     ROS_INFO("%s: Preempted", special_mv_action_name_.c_str());
  //     special_mv_server_.setPreempted();
  //     preempted = true;
  //     break;
  //   }

  //   // altitude_mutex_.lock();
  //   // height_error = std::abs(desired_height - altitude_);
  //   // altitude_mutex_.unlock();
  //   ROS_INFO("Waiting for preemtion to break the loop...");
  //   feedback_period.sleep();
  //   ros::spinOnce(); //TODO do i needd the spinning here?
  // }

  // if(!preempted) 
  // {
  //   if(!result)
  //    {
  //     ROS_WARN("Request for special movement has failed");
  //     special_mv_result_.done = false;
  //     special_mv_server_.setAborted(special_mv_result_); //consider sending a text msg as well to differentiate the reasons
  //     return false;
  //   } 
  //   else 
  //   {
  //     special_mv_result_.done = true;
  //     ROS_INFO("%s: Succeeded", special_mv_action_name_.c_str());
  //     special_mv_server_.setSucceeded(special_mv_result_);
  //     return true;
  //   }
  // } 
  // else 
  // {
  //   ROS_WARN("%s: Preempted", fly_action_name_.c_str());
  //   return false;
  // }
  ROS_INFO("%s: Succeeded", special_mv_action_name_.c_str());
  special_mv_server_.setSucceeded(special_mv_result_);
}

bool DummyFCS_Interface::setWaypoints_(const uav_msgs::SearchWPGoalConstPtr &goal)
{
  
  bool set_wpgoal=false;
  bool result= false;

  // Access the fields from the goal message
    //uav_msgs::WayPtSearch swaypoints;
    uav_msgs::WayPtSearch swaypoints = goal->goal;
    std_msgs::Header header = swaypoints.header;
    uint8_t status = swaypoints.status;
    std::string fcs_pckg_path = ros::package::getPath("fcs_interface");
  
  switch (status) 
  {
    case uav_msgs::WayPtSearch::WSP0:
    {
        // Handle WSP0 status
       ROS_DEBUG("Received goal with status WSP0 which mean skipping waypoint mission");
      set_wpgoal= true;
      break;
    }
    case uav_msgs::WayPtSearch::WSP1:
    {
      // Handle WSP1 status
      ROS_DEBUG("Received goal with status WSP1");
      jsonFilePath=fcs_pckg_path + "/config/swp.json";
      //jsonFilePath = "/home/shival_dubey/catkin_ws/src/fire-fighting-uav/fcs_interface/config/swp.json";
      set_wpgoal= true;
      break;
    }
    case uav_msgs::WayPtSearch::WSP2:
    {
      ROS_DEBUG("Received goal with status WSP2");
      set_wpgoal=true;
      // Handle WSP2 status
      break;
    }
    case uav_msgs::WayPtSearch::WSP3:
    {
      ROS_DEBUG("Received goal with status WSP1");
      set_wpgoal= true;
      // Handle WSP3 status
      break;
    } 
    default:
    {
      // Unknown status, handle accordingly
      ROS_WARN("Received goal with unknown status: %d", status);
      set_wpgoal=false;
    }
  }
  if (set_wpgoal)
  {
    result=DummyFCS_Interface::runSearchMission(1);
    if(!result) 
    {
      ROS_WARN("Run Search Mission failed");
      fly_result_.in_location = false;
      fly_server_.setAborted(fly_result_); 
    } 
    else 
    {
      bool preempted {false};
      ros::Duration feedback_period(1);

      while(ros::ok()) 
      {

        // if(position_mutex_.try_lock()) 
        // {
        //   fly_feedback_.current_location = gps_position_;
        //   position_mutex_.unlock();
        //   fly_server_.publishFeedback(fly_feedback_);
        // }
        ROS_INFO("IN WHILE LOOP RUNNIG..");
        if(fly_server_.isPreemptRequested() || !ros::ok()) 
        {
    
          ROS_INFO("%s: Preempted", fly_action_name_.c_str());
          // fly_server_.setPreempted();
          fly_result_.in_location = false;
          fly_server_.setAborted(fly_result_);
          preempted = true;
          break;
        }

        feedback_period.sleep();
        ros::spinOnce(); //TODO do i needd the spinning here?
      }

      // if(!preempted) 
      // {
      //   fly_result_.in_location = true;
      //   ROS_INFO("%s: Succeeded", fly_action_name_.c_str());
      //   fly_server_.setSucceeded(fly_result_);
      // }
      // else 
      // {
      //   fly_result_.in_location = false;
      //   ROS_INFO("%s: Preempted", fly_action_name_.c_str());
      // }
  }
  }
 return set_wpgoal;
}

bool DummyFCS_Interface::runSearchMission(int responseTimeout)
{
  bool result=true;
  current_waypoint_index = 0; 
  ros::spinOnce();
  // Search Mission : Initialization Step 1 for loading defualt waypoint 
  //dji_sdk::MissionWaypointTask waypointTask;
  //setWaypointInitDefaults_(waypointTask);(waypointTask);

  // Search Mission : Step 2 Uploading mission from JSON file
  //ROS_INFO("Uploading Waypoints from JSON..\n");
  //uploadWaypointsJSON(jsonFilePath, responseTimeout, waypointTask);
  
  // Step 3 Initialise the waypoint mission.
  //dji_sdk::MissionWpUpload missionWaypointUpload;
  //missionWaypointUpload.request.waypoint_task = waypointTask;
  //waypoint_upload_client_.call(missionWaypointUpload);
  // result=true;
  // if (!result) 
  // {
  //   ROS_WARN("Failed sending mission upload command");
  //   ROS_WARN("ack.info: set = %i id = %i", missionWaypointUpload.response.cmd_set,
  //            missionWaypointUpload.response.cmd_id);
  //   ROS_WARN("ack.data: %i", missionWaypointUpload.response.ack_data);
  // } 
  // else 
  // // {
  //   ROS_INFO("Waypoint upload command sent successfully");
  //   // Start the mission.
  //   //result = waypointMissionAction_(kActionStart);
  //   if (result) 
  //   {
       ROS_INFO("Mission start command sent successfully");
  //   } 
  //   else 
  //   {
  //     ROS_WARN("Failed sending mission start command");
  //   }

  //}

   return result;
}
// void DummyFCS_Interface::setWaypointInitDefaults_(dji_sdk::MissionWaypointTask & waypointTask) 
// {
//   waypointTask.velocity_range = 10;
//   waypointTask.idle_velocity = 5;
//   waypointTask.action_on_finish = dji_sdk::MissionWaypointTask::FINISH_NO_ACTION;
//   waypointTask.mission_exec_times = 1;
//   waypointTask.yaw_mode = dji_sdk::MissionWaypointTask::YAW_MODE_AUTO;
//   waypointTask.trace_mode = dji_sdk::MissionWaypointTask::TRACE_POINT;
//   waypointTask.action_on_rc_lost = dji_sdk::MissionWaypointTask::ACTION_AUTO;
//   waypointTask.gimbal_pitch_mode = dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;
// }

// Function to upload waypoints to a mission task
void DummyFCS_Interface::uploadWaypointsJSON(const std::string& jsonFilePath, int responseTimeout, dji_sdk::MissionWaypointTask& waypointTask) 
{
    // // Read waypoints from JSON file
    // std::vector<WayPointSettings> waypoints = readWaypointsFromJSON(jsonFilePath);

    // // Create a waypoint object to store waypoint information
    // dji_sdk::MissionWaypoint waypoint;

    // // Iterate through the list of waypoints
    // for (size_t i = 0; i < waypoints.size(); ++i) 
    // {
    //     // Display information about the created waypoint (latitude, longitude, altitude)
    //     ROS_INFO("Waypoint %zu created at (LLA): %f \t%f \t%f", i + 1, waypoints[i].latitude,
    //              waypoints[i].longitude, waypoints[i].altitude);

    //     // Set waypoint parameters and push back the waypoint to the mission task
    //     waypoint.latitude = waypoints[i].latitude;
    //     waypoint.longitude = waypoints[i].longitude;
    //     waypoint.altitude = waypoints[i].altitude;
    //     waypoint.damping_distance = 0;
    //     waypoint.target_yaw = 0;
    //     waypoint.target_gimbal_pitch = 0;
    //     waypoint.turn_mode = 0;
    //     waypoint.has_action = 0;

    //     waypointTask.mission_waypoint.push_back(waypoint);

    //     // Display detailed information about the uploaded waypoint
    //     ROS_INFO("Uploaded Waypoint %zu:", i + 1);
    //     ROS_INFO("  Latitude: %f", waypoint.latitude);
    //     ROS_INFO("  Longitude: %f", waypoint.longitude);
    //     ROS_INFO("  Altitude: %f", waypoint.altitude);
    //     ROS_INFO("  Damping Distance: %f", waypoint.damping_distance);
    //     ROS_INFO("  Target Yaw: %f", waypoint.target_yaw);
    //     ROS_INFO("  Target Gimbal Pitch: %f", waypoint.target_gimbal_pitch);
    //     ROS_INFO("  Turn Mode: %d", waypoint.turn_mode);
    //     ROS_INFO("  Has Action: %d", waypoint.has_action);
    // }

}
bool DummyFCS_Interface::waypointMissionAction_(WaypointAction action) {
  bool result = false;
  // dji_sdk::MissionWpAction wp_action;
  switch (action) {
    case kActionStart:
      // wp_action.request.action = dji_sdk::MissionWpAction::Request::ACTION_START;
      ROS_INFO("ACTION_START.");
      break;
    case kActionStop:
      // wp_action.request.action = dji_sdk::MissionWpAction::Request::ACTION_STOP;
      ROS_INFO("ACTION_STOP.");
      break;
    case kActionPause:
      // wp_action.request.action = dji_sdk::MissionWpAction::Request::ACTION_PAUSE;
      ROS_INFO("ACTION_PAUSE.");
      break;
    case kActionResume:
      // wp_action.request.action = dji_sdk::MissionWpAction::Request::ACTION_RESUME;
      ROS_INFO("ACTION_RESUME.");
      break;
    default:
      ROS_WARN("Waypoint action not handled, %d", action);
      break;
  }
  result = true;
  if (!true) 
  {
    ROS_WARN("waypointMissionAction failed.");
    // ROS_WARN("ack.info: set = %i id = %i", wp_action.response.cmd_set, wp_action.response.cmd_id);
    // ROS_WARN("ack.data: %i", wp_action.response.ack_data);
  }
  return result;
}
bool DummyFCS_Interface::droneWithinRadiusFromJSON(const std::string& jsonFilePath) 
{
    bool result=false;
    // double R = 6378.137; // Radius of earth in KM

    // position_mutex_.lock();
    // double lat1 = gps_position_.latitude;
    // double lon1 = gps_position_.longitude;
    // position_mutex_.unlock();

    // Read waypoints from JSON file
    // std::vector<WayPointSettings> waypoints = readWaypointsFromJSON(jsonFilePath);

    // if (waypoints.empty()) 
    // {
    //     ROS_WARN("Waypoints list is empty in the JSON file");
    //     return false;
    // }

    //int current_waypoint_index = 0;  // Local variable to keep track of the current waypoint

    double precision_radius = 2.0; // Set your precision radius here (in meters)

    // ROS_INFO("Number of waypoints in the JSON file: %lu", waypoints.size());
    //ros::Duration feedback_period(1);
    // while (current_waypoint_index < waypoints.size()&& ros::ok())
    // {
        // double lat2 = waypoints[current_waypoint_index].latitude;
        // double lon2 = waypoints[current_waypoint_index].longitude;

        // ROS_INFO("Current Latitude: %f", lat1);
        // ROS_INFO("Waypoint Latitude: %f", lat2);
        // ROS_INFO("Current Longitude: %f", lon1);
        // ROS_INFO("Waypoint Longitude: %f", lon2);
      
       

        // double dLat = (lat2 - lat1) * M_PI / 180;
        // double dLon = (lon2 - lon1) * M_PI / 180;

        // ROS_INFO("diff Lat: %f", dLat);
        // ROS_INFO("diff Long: %f", dLon);

        // double a = sin(dLat / 2) * sin(dLat / 2) +
        //            cos(lat1 * M_PI / 180) * cos(lat2 * M_PI / 180) *
        //            sin(dLon / 2) * sin(dLon / 2);

        // double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        // double d = R * c * 1000; // meters

        // ROS_INFO("Distance to waypoint %d (%f, %f) is %f", current_waypoint_index + 1, lat2, lon2, d);

        // if (d <= precision_radius) 
        // {
        //     ROS_INFO("Drone is within precision radius of waypoint %d", current_waypoint_index + 1);
        //     // Move on to the next waypoint
        //     current_waypoint_index++;
        // }
        // else
        // {
        //     ROS_DEBUG("Drone is not within precision radius of waypoint %d", current_waypoint_index + 1);
        // }
        //feedback_period.sleep();
        ros::spinOnce(); 
   //}

    // If the loop completes, it means the drone has checked all waypoints
    if (current_waypoint_index == 7)
    {
      result= true;
      ROS_INFO("Drone has reached all waypoints");
    }
    else 
    {
      result= false;
    }
    return result;
}

void DummyFCS_Interface::batteryStateCallback_(const sensor_msgs::BatteryState::ConstPtr& message) {
    static int num_runs = 0;
  uav_msgs::BatteryPercentage msg;
  msg.input_msg_id  = num_runs;
  msg.percentage = int(message->percentage);
  battery_state_publisher_.publish(msg);
  num_runs++;
}