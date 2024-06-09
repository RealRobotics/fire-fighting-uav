#include "fcs_interface/dummy_fcs_interface.h"
#include "uav_msgs/SpecialMovement.h"
#include "uav_msgs/BatteryPercentage.h"
#include <uav_msgs/BatterySim.h>

#include <chrono>
#include <cmath>
#include <future>  

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "uav_msgs/GpsLocationWithPrecision.h"

#define UNLADEN_VELOCITY_M_PER_S (2.0)
#define LADEN_VELOCITY_M_PER_S (1.0)
#define VELOCITY_RANGE_M_PER_S (5.0)

DummyFCS_Interface::DummyFCS_Interface(ros::NodeHandle node_handle)
: node_handle_(node_handle), fly_server_(node_handle, "fcs_interface/fly_to_wp", boost::bind(&DummyFCS_Interface::setWaypoint_, this, _1), false),
  special_mv_server_(node_handle, "fcs_interface/special_movement", boost::bind(&DummyFCS_Interface::specialMovement_, this, _1), false) {
}

bool DummyFCS_Interface::start() {
  ROS_INFO("Starting the DummyFCS_Interface.");

  // Subscribe to DJI OSDK topics
  gps_position_subscriber_ = node_handle_.subscribe<sensor_msgs::NavSatFix>("dji_sdk/gps_position", 10,
                                                                            &DummyFCS_Interface::gpsPositionCallback_, this);
  // battery_state_subscriber_ = node_handle_.subscribe<sensor_msgs::BatteryState>("dji_sdk/battery_state", 10,
  //                                                                           &DummyFCS_Interface::batteryStateCallback_, this);

  //set up publishing topics
  battery_state_publisher_ = node_handle_.advertise<uav_msgs::BatteryPercentage>("fcs_interface/battery_state", 10);

  //start the action servers
  fly_server_.start();
  special_mv_server_.start();
  
  //wait for the home_location to be initialised (i.e. obtain first gps location)
  // while(ros::ok()) {
  //   home_mutex_.lock();
  //   if (home_position_initialised_) {
  //     home_mutex_.unlock();
  //     break;
  //   }
  //   home_mutex_.unlock();
  //   ros::Duration(0.1).sleep();
  //   ros::spinOnce();
  //   ROS_INFO("Waiting to obtain first gps reading");
  // }
  ROS_INFO("Started dummy FCS_Interface.");
  
  return true;
}

bool DummyFCS_Interface::droneTaskControl_() {
  ros::Duration(10).sleep();  
  return true;
}

bool DummyFCS_Interface::specialMovement_(const uav_msgs::SpecialMovementGoalConstPtr &goal) {
  bool result {false};
  double desired_height;
  bool preempted {false};

  if(goal->movement.data == uav_msgs::SpecialMovement::TAKE_OFF) {
    ROS_INFO("Taking off...");
    result = true;
  } else if (goal->movement.data == uav_msgs::SpecialMovement::LAND) {
    ROS_INFO("Landing...");
    result = true;
  } else if (goal->movement.data == uav_msgs::SpecialMovement::GO_HOME) {
    ROS_INFO("Returning home...");
    result = true;
    desired_height = 0;
  }
  std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
  while (ros::ok()) {
    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
    double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time).count();
        // Break the loop if elapsed time exceeds desired interval
    if (elapsed_seconds > 7.0) {
        break;
    }
    if(special_mv_server_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", special_mv_action_name_.c_str());
      special_mv_server_.setPreempted();
      preempted = true;
      break;
      }
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


bool DummyFCS_Interface::setWaypoint_(const uav_msgs::FlyToWPGoalConstPtr &goal)
{
  sensor_msgs::NavSatFix nav_sat_fix = goal->goal.location;
  ROS_INFO("Sending waypoint: %f, %f, %f", nav_sat_fix.latitude, nav_sat_fix.longitude, nav_sat_fix.altitude);
  std::future<bool> result = std::async(&DummyFCS_Interface::uploadNavSatFix_,this, nav_sat_fix);

  bool preempted {false};
  std::chrono::milliseconds span (100);
  while((result.wait_for(span)==std::future_status::timeout) && ros::ok()) {
    if(position_mutex_.try_lock()) {
      fly_feedback_.current_location = gps_position_;
      position_mutex_.unlock();
      fly_server_.publishFeedback(fly_feedback_);
    }

    if(fly_server_.isPreemptRequested() || !ros::ok()) {
      ROS_INFO("%s: Preempted", fly_action_name_.c_str());
      fly_server_.setPreempted();
      preempted = true;
      break;
    }
  }

  if(!preempted) {
    if(!result.get()) {
      ROS_WARN("Request to fly to WP has failed");
      fly_result_.in_location = false;
      fly_server_.setAborted(fly_result_); //consider sending a text msg as well to differentiate the reasons
    } else {
      fly_result_.in_location = true;
      ROS_INFO("%s: Succeeded", fly_action_name_.c_str());
      fly_server_.setSucceeded(fly_result_);
    }
  } else {
    ROS_WARN("%s: Preempted", fly_action_name_.c_str());
  }

  return true;
}

bool DummyFCS_Interface::uploadNavSatFix_(const sensor_msgs::NavSatFix& nav_sat_fix) {
  ros::Duration(10).sleep();
  return true;
}

//TODO use gps health to trust the data only if health is good
void DummyFCS_Interface::gpsPositionCallback_(const sensor_msgs::NavSatFix::ConstPtr& message) {
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


void DummyFCS_Interface::batteryStateCallback_(const uav_msgs::BatterySim::ConstPtr& message) {
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