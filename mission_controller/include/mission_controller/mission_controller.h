#ifndef MISSION_CONTROLLER_H
#define MISSION_CONTROLLER_H

#include <string>
#include <jsoncpp/json/json.h>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "ros/ros.h"
#include "uav_msgs/EnableMission.h"
#include "uav_msgs/DisableMission.h"

class MissionController {
public:
  MissionController(ros::NodeHandle nh, std::string tree_file, std::string waypoints_file);

private:
  void registerNodes_();
  void createTree_(std::string tree_file, std::string waypoints_file);
  void saveWpToBlackboard_(Json::Value wp, std::string wp_name);
  void loadWaypoints_(std::string path);
  void loadBatteryLimits_();
  void loadSpecialMovementsCmds_();
  bool enableMission_(uav_msgs::EnableMission::Request  &req,
    uav_msgs::EnableMission::Response &res);
  bool disableMission_(uav_msgs::DisableMission::Request  &req,
    uav_msgs::DisableMission::Response &res);
  void run();

  ros::NodeHandle nh_;
  ros::ServiceServer enable_service_;
  ros::ServiceServer disable_service_;

  BT::Blackboard::Ptr blackboard_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
};

#endif //MISSION_CONTROLLER_H