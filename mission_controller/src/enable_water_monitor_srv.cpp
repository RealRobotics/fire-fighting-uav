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

#include "mission_controller/enable_water_monitor_srv.h"
#include "uav_msgs/EnableWaterMonitor.h"
#include <memory>


EnableWaterMonitorAction::EnableWaterMonitorAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration & conf): 
BT::RosServiceNode<uav_msgs::EnableWaterMonitor>(handle, name, conf){}

BT::PortsList EnableWaterMonitorAction::providedPorts() {
  return {BT::InputPort<uint8_t>("Service Input for Pump Status")};  
}

  void EnableWaterMonitorAction::sendRequest(RequestType& request)
  {
    getInput("Service Input for Pump Status", request.status);

    expected_result_ = true;
    ROS_INFO("Sending request to Water Monitor");
  }

 BT::NodeStatus EnableWaterMonitorAction::onResponse(const ResponseType& rep)
  {
    ROS_INFO(" Response received from Water Monitor");
    if( rep.result == expected_result_)
    {
      return BT::NodeStatus::SUCCESS;
    }
    else{
      return BT::NodeStatus::FAILURE;
    }
  }
 BT::NodeStatus EnableWaterMonitorAction::onFailedRequest(RosServiceNode::FailureCause failure)
  {
    return BT::NodeStatus::FAILURE;
  }
