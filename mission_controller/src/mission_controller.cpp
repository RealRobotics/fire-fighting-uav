#include "mission_controller/mission_controller.h"

#include <fstream>
#include <future>

#include "uav_msgs/SpecialMovement.h"
#include "uav_msgs/BatteryStatus.h"

#include "mission_controller/fly_to_wp_bt_action.h"
#include "mission_controller/special_movement_bt_action.h"
#include "mission_controller/is_battery_required_status.h"
#include "mission_controller/is_mission_enabled.h"
#include "mission_controller/repeat_over_vector_until_failure.h"

MissionController::MissionController(ros::NodeHandle nh, std::string tree_file, std::string waypoints_file) 
 : nh_(nh) {
  registerNodes_();
  createTree_(tree_file, waypoints_file);
  enable_service_ = nh.advertiseService("mission_controller/enable_mission", &MissionController::enableMission_,this);
  disable_service_ = nh.advertiseService("mission_controller/disable_mission", &MissionController::disableMission_,this);
}

void MissionController::registerNodes_() {
  BT::RegisterRosAction<FlyToWpBTAction>(factory_, "FlyToWpBTAction", nh_);
  BT::RegisterRosAction<SpecialMovementBTAction>(factory_, "SpecialMovementBTAction", nh_);
  IsBatteryRequiredStatus::Register(factory_, "IsBatteryRequiredStatus", nh_);
  IsMissionEnabled::Register(factory_, "IsMissionEnabled", nh_);
  RepeatOverVectorUntilFailure::Register(factory_, "RepeatOverVectorUntilFailure");
}

void MissionController::createTree_(std::string tree_file, std::string waypoints_file) {
  tree_ = factory_.createTreeFromFile(tree_file);
  blackboard_ = tree_.rootBlackboard();
  blackboard_->set("mission_enabled", false);
  loadWaypoints_(waypoints_file);
  loadBatteryLimits_();
  loadSpecialMovementsCmds_();  
}

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
      std::vector<sensor_msgs::NavSatFix> waypoints;
      for (uint i = 0; i < num_waypoints; ++i) {
        std::string wp = "wp" + std::to_string(i);
        sensor_msgs::NavSatFix gps_wp;
        gps_wp.latitude = root[wp]["latitude"].asDouble();
        gps_wp.longitude = root[wp]["longitude"].asDouble();
        blackboard_->set(wp, gps_wp);
        waypoints.push_back(gps_wp);
      }
      blackboard_->set("waypoint_list", waypoints);
    }
  }
}

void MissionController::loadBatteryLimits_() {
  blackboard_->set("battery_ok", uint8_t(uav_msgs::BatteryStatus::OK));
  blackboard_->set("battery_mission_critical", uint8_t(uav_msgs::BatteryStatus::MISSION_CRITICAL));
  blackboard_->set("battery_safety_critical", uint8_t(uav_msgs::BatteryStatus::SAFETY_CRITICAL));
}

void MissionController::loadSpecialMovementsCmds_() {
  blackboard_->set("take_off", uint8_t(uav_msgs::SpecialMovement::TAKE_OFF));
  blackboard_->set("land", uint8_t(uav_msgs::SpecialMovement::LAND));
  blackboard_->set("go_home", uint8_t(uav_msgs::SpecialMovement::GO_HOME));
}

void MissionController::run() {
  BT::NodeStatus status = BT::NodeStatus::IDLE;

  ROS_INFO("Mission controller ready to run");

  while( ros::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING))
  {
    ros::spinOnce();
    status = tree_.tickRoot();
    std::cout << status << std::endl;
    ros::Duration sleep_time(1);
    sleep_time.sleep();
  }
}

bool MissionController::enableMission_(uav_msgs::EnableMission::Request  &req,
  uav_msgs::EnableMission::Response &res) {
  blackboard_->set("mission_enabled", true);
  res.result = true;
  return true;
}

bool MissionController::disableMission_(uav_msgs::DisableMission::Request  &req,
  uav_msgs::DisableMission::Response &res) {
  blackboard_->set("mission_enabled", false);
  res.result = true;
  return true;
}