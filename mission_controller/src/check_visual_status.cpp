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

#include "mission_controller/check_visual_status.h"
#include <memory>


/**
 * BehaviourCpp library doesn't provide wrapper for ros topic under ROS1.
 * So this is topic/type specific  implementation inspired by their ROS2 support.
 * The purpose of this class is to act as a configurable condition node and 
 * to answer "true/false" for quaestion "Check VisualNavigation Status equal to <value>" where
 * the value is configurable via port.
*/

/**
 * Constructor
 * @param instance_name - name of the BT node
 * @param conf - BT::node configuration
 * @param nh - ros nodehandle
*/
 CheckVisualStatus:: CheckVisualStatus(const std::string & instance_name, const BT::NodeConfiguration & conf,
              ros::NodeHandle& nh)
  : BT::ConditionNode(instance_name, conf), nh_ (nh) {
    visual_status_sub_ = nh_.subscribe<uav_msgs::VisualNavigation>("dft_node//dft_node_topic", 10,
                                                                            &CheckVisualStatus::VisualStatusCallback_, this);
    }

/**
 * This method does the main work and is called automatically
 * It loads the required status from the port and check it againts its saved last status.
 * This saved status is updated by the topic callback below.
*/
BT::NodeStatus CheckVisualStatus::tick() {
  getInput<int>("required_status", required_status_);
  status_mtx_.lock(); //it is good practise to use mutexes when accessing variable which can be modified from different threads
  //as callbacks can be threads, I uses mutexes here to make this code robust
  if (current_status_ == uav_msgs::VisualNavigation::OFFLINE) {
    status_mtx_.unlock();
    ROS_INFO("CheckVisualNavigationStatus: RUNNING OFFLINE");
    return BT::NodeStatus::RUNNING;
  }
  if (current_status_ == required_status_) {
    status_mtx_.unlock();
    ROS_INFO("CheckVisualNavigationStatus: SUCCESS");
    return BT::NodeStatus::SUCCESS;
  } else {
    status_mtx_.unlock();
    ROS_INFO("CheckVisualNavigationStatus: FAILURE current status is not required status");
    return BT::NodeStatus::FAILURE;
  }
}

/**
 * Standard topic callback
*/
void CheckVisualStatus::VisualStatusCallback_(const uav_msgs::VisualNavigation::ConstPtr& message) {
  status_mtx_.lock();
  current_status_ = message->status;
  status_mtx_.unlock();
}

/** 
 * List of ports this BT node provides
*/
BT::PortsList CheckVisualStatus:: providedPorts() {
  return {BT::InputPort<uint8_t>("required_status")};
}

/**
 * This is some BT black magic :D And it is inspired from ROS2 BehaviourCpp wrapper
 * The method register is important to tell the BT builder (the factory), about this class
 *  */
void CheckVisualStatus::Register(BT::BehaviorTreeFactory& factory,
                         const std::string& registration_ID,
                         ros::NodeHandle& node_handle)
{
  BT::NodeBuilder builder = [&node_handle](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<CheckVisualStatus>(name, config,node_handle);
  };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<CheckVisualStatus>();
  manifest.ports = CheckVisualStatus::providedPorts();
  manifest.registration_ID = registration_ID;
  factory.registerBuilder( manifest, builder );
}