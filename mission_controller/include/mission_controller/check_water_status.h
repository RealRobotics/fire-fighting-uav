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

#ifndef CHECK_WATER_STATUS_H
#define CHECK_WATER_STATUS_H

#include <mutex>

#include "behaviortree_cpp_v3/condition_node.h"
#include <behaviortree_cpp_v3/bt_factory.h>
#include "ros/ros.h"
#include "uav_msgs/WaterStatus.h"

class CheckWaterStatus : public BT::ConditionNode {

public:
  CheckWaterStatus(const std::string & instance_name, const BT::NodeConfiguration & conf,
              ros::NodeHandle& nh);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

  static void Register(BT::BehaviorTreeFactory& factory,
                         const std::string& registration_ID,
                         ros::NodeHandle& node_handle);

private:
  void WaterStatusCallback_(const uav_msgs::WaterStatus::ConstPtr& msg);
  ros::NodeHandle nh_;
  ros::Subscriber water_status_sub_;
  std::mutex status_mtx_;
  uint8_t current_status_ {uav_msgs::WaterStatus::WaterOK};
  int required_status_ {uav_msgs::WaterStatus::WaterOK};

};

#endif //CHECK_WATER_STATUS