// Copyright 2023 University of Leeds.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "mission_controller/mission_controller.h"

#include <fstream>
#include <future>

#include "uav_msgs/SpecialMovement.h"
#include "uav_msgs/BatteryStatus.h"
#include "uav_msgs/GimbalStatus.h"
#include "uav_msgs/VisualNavigation.h"
#include "uav_msgs/WaterStatus.h"
#include "uav_msgs/FireTarget.h"
#include "uav_msgs/PumpStatus.h"



#include "mission_controller/fly_to_wp_bt_action.h"
#include "mission_controller/special_movement_bt_action.h"
#include "mission_controller/is_battery_required_status.h"
#include "mission_controller/is_mission_enabled.h"
#include "mission_controller/repeat_over_vector_until_failure.h"
#include "mission_controller/check_fire_status.h"
#include "mission_controller/check_gimbal_status.h"
#include "mission_controller/check_visual_status.h"
#include "mission_controller/check_water_status.h"
#include "mission_controller/spray_bt_action.h"
#include "mission_controller/enable_water_monitor_srv.h"

/** This is a constructor
 * @param nh - ros node handle
 * @param tree_file - path to .xml file with BT specification. This BT will be run in the mission controller
 * @param waypoints_file - path to .json file which specifies wp over which drone shall be moving
 */
MissionController::MissionController(ros::NodeHandle nh, std::string tree_file, std::string waypoints_file) 
 : nh_(nh) {
  registerNodes_();
  createTree_(tree_file, waypoints_file);
  enable_service_ = nh.advertiseService("mission_controller/enable_mission", &MissionController::enableMission_,this);
  disable_service_ = nh.advertiseService("mission_controller/disable_mission", &MissionController::disableMission_,this);
}

/**
 * This method contains loop which ticks the BT. The loop ends either:
 * when the exececution of BT finishes (i.e. it succeeded or failed)
 * or when ros dies
*/
void MissionController::run() {
  BT::NodeStatus status = BT::NodeStatus::IDLE;

  ROS_INFO("Mission controller ready to run");

  while( ros::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING))
  {
    ros::spinOnce();
    status = tree_.tickRoot();
    std::cout << "Current tree status: " << status << std::endl;
    ros::Duration sleep_time(1);
    sleep_time.sleep();
  }
}

/**
 * Each type of custom BT node must be "registered" that the BT builder can know what type matches what class
*/
void MissionController::registerNodes_() {
  BT::RegisterRosAction<FlyToWpBTAction>(factory_, "FlyToWpBTAction", nh_);
  BT::RegisterRosAction<SpecialMovementBTAction>(factory_, "SpecialMovementBTAction", nh_);
  BT::RegisterRosAction<SprayBTAction>(factory_,"SprayBTAction", nh_);
  BT::RegisterRosService<EnableWaterMonitorAction>(factory_, "EnableWaterMonitor", nh_);
  IsBatteryRequiredStatus::Register(factory_, "IsBatteryRequiredStatus", nh_);
  IsMissionEnabled::Register(factory_, "IsMissionEnabled", nh_);
  RepeatOverVectorUntilFailure::Register(factory_, "RepeatOverVectorUntilFailure");
  CheckFireStatus::Register(factory_, "CheckFireStatus", nh_);
  CheckGimbalStatus::Register(factory_, "CheckGimbalStatus", nh_);
  CheckVisualStatus::Register(factory_, "CheckVisualStatus", nh_);
  CheckWaterStatus::Register(factory_, "CheckWaterStatus", nh_);

}

/**
 * This method does 4 things:
 * a) creates the BT tree from file
 * b) loads waypoints to BT storage (= blackboard)
 * c) loads constants for battery limits to BT storage, firestatus, gimbalstatus,visualstatus,waterstatus.
 * d) loads constants for special movements to BT storage
 * @param tree_file path to .xml file containing BT specification
 * @param waypoints_file path to .json file containing the search pattern
*/
void MissionController::createTree_(std::string tree_file, std::string waypoints_file) {
  tree_ = factory_.createTreeFromFile(tree_file);
  blackboard_ = tree_.rootBlackboard();
  blackboard_->set("mission_enabled", false);
  loadWaypoints_(waypoints_file);
  loadBatteryLimits_();
  loadSpecialMovementsCmds_();  
  loadfirestatus_();
  loadgimbalstatus_();
  loadvisualstatus_();
  loadwaterstatus_();
  loadPumpWaterMonitorcmds_();
}

/**
 * This method parses json file and loads the waypoints into BT storage (= blackboard)
 * @param path - path to .json file containing the search pattern
*/
void MissionController::loadWaypoints_(std::string path) {
  std::ifstream search_pattern_file; 
  search_pattern_file.open(path);
  Json::Value root;
  if (search_pattern_file.is_open()) {
    ROS_INFO("Loading the search pattern.");
    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    JSONCPP_STRING errs;
    if (!parseFromStream(builder, search_pattern_file, &root, &errs)) {
      std::cout << errs << std::endl;
      return;
    } else {
      int num_waypoints = root["num_waypoints"].asInt();
      blackboard_->set("num_waypoints", num_waypoints);
      ROS_INFO("There are %d waypoints to load.", num_waypoints);
      std::vector<uav_msgs::GpsLocationWithPrecision> waypoints;
      for (uint i = 0; i < num_waypoints; ++i) {
        std::string wp = "wp" + std::to_string(i);
        uav_msgs::GpsLocationWithPrecision gps_wp;
        gps_wp.location.latitude = root[wp]["latitude"].asDouble();
        gps_wp.location.longitude = root[wp]["longitude"].asDouble();
        gps_wp.location.altitude = root[wp]["altitude"].asDouble();
        gps_wp.loc_precision = root[wp]["precision"].asDouble();
        //blackboard_->set(wp, gps_wp);
        waypoints.push_back(gps_wp);
      }
      blackboard_->set("waypoint_list", waypoints);
    }
  }
}

/**
 * The three constants for battery status are loaded to BT storage
 * This is needed as those constants are referred in .xml file;
 * without this loading the BT construction wouldn't understand those limits
*/
void MissionController::loadBatteryLimits_() {
  blackboard_->set("battery_ok", uint8_t(uav_msgs::BatteryStatus::OK));
  blackboard_->set("battery_mission_critical", uint8_t(uav_msgs::BatteryStatus::MISSION_CRITICAL));
  blackboard_->set("battery_safety_critical", uint8_t(uav_msgs::BatteryStatus::SAFETY_CRITICAL));
}

/**
 * The constants for special movements are loaded here. The reasoning is same as in the above method.
*/
void MissionController::loadSpecialMovementsCmds_() {
  blackboard_->set("take_off", uint8_t(uav_msgs::SpecialMovement::TAKE_OFF));
  blackboard_->set("land", uint8_t(uav_msgs::SpecialMovement::LAND));
  blackboard_->set("go_home", uint8_t(uav_msgs::SpecialMovement::GO_HOME));
}

/**
 * The constants for fire status are loaded here. The reasoning is same as in the above method.
*/
void MissionController::loadfirestatus_()
{
  blackboard_->set("no_fire", uint8_t(uav_msgs::FireTarget::NO_FIRE));
  blackboard_->set("detected", uint8_t(uav_msgs::FireTarget::DETECTED));
  blackboard_->set("tracked", uint8_t(uav_msgs::FireTarget::TRACKED));
}

/**
 * The constants for Gimbal status are loaded here. The reasoning is same as in the above method.
*/
void MissionController::loadgimbalstatus_()
{
  blackboard_->set("idle", uint8_t(uav_msgs::GimbalStatus::IDLE));
  blackboard_->set("tracking", uint8_t(uav_msgs::GimbalStatus::TRACKING));
  blackboard_->set("out_of_boundry", uint8_t(uav_msgs::GimbalStatus::OUT_OF_BOUNDARY));
}

/**
 * The constants for visual status are loaded here. The reasoning is same as in the above method.
*/
void MissionController::loadvisualstatus_()
{
  blackboard_->set("offline", uint8_t(uav_msgs::VisualNavigation::OFFLINE));
  blackboard_->set("notcentred", uint8_t(uav_msgs::VisualNavigation::NOTCENTRED));
  blackboard_->set("centred", uint8_t(uav_msgs::VisualNavigation::CENTRED));
}

/**
 * The constants for water status are loaded here. The reasoning is same as in the above method.
*/
void MissionController::loadwaterstatus_()
{
  blackboard_->set("water_ok", uint8_t(uav_msgs::WaterStatus::WaterOK));
  blackboard_->set("water_low", uint8_t(uav_msgs::WaterStatus::WaterLow));
}

/**
 * The constants for goal action server and service water monitor cmd are loaded here. The reasoning is same as in the above method.
*/
void MissionController::loadPumpWaterMonitorcmds_()
{
  blackboard_->set("pump_on", uint8_t(uav_msgs::PumpStatus::ON));
  blackboard_->set("pump_off", uint8_t(uav_msgs::PumpStatus::OFF));
}

/**
 * This method gets called when the enable service is requested. 
 * It set the value in the BT storage space (=blackboard), which is parsed by the appropriate condition node
 * @param req - reference to service request
 * @return always true
*/
bool MissionController::enableMission_(uav_msgs::EnableMission::Request  &req,
  uav_msgs::EnableMission::Response &res) {
  blackboard_->set("mission_enabled", true);
  res.result = true;
  return true;
}

/**
 * This method gets called when the disable service is requested. 
 * It set the value in the BT storage space (=blackboard), which is parsed by the appropriate condition node
 * @param req - reference to service request
 * @return always true
*/
bool MissionController::disableMission_(uav_msgs::DisableMission::Request  &req,
  uav_msgs::DisableMission::Response &res) {
  blackboard_->set("mission_enabled", false);
  res.result = true;
  return true;
}