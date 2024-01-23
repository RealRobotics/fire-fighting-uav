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
  void run();

private:
  void registerNodes_();
  void createTree_(std::string tree_file, std::string waypoints_file);
  void loadWaypoints_(std::string path);
  void loadBatteryLimits_();
  void loadSpecialMovementsCmds_();
  void loadfirestatus_();
  void loadgimbalstatus_();
  void loadwaterstatus_();
  void loadvisualstatus_();
  void loadPumpWaterMonitorcmds_();
  bool enableMission_(uav_msgs::EnableMission::Request  &req,
    uav_msgs::EnableMission::Response &res);
  bool disableMission_(uav_msgs::DisableMission::Request  &req,
    uav_msgs::DisableMission::Response &res);

  ros::NodeHandle nh_;
  ros::ServiceServer enable_service_;
  ros::ServiceServer disable_service_;

  BT::Blackboard::Ptr blackboard_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
};

#endif //MISSION_CONTROLLER_H