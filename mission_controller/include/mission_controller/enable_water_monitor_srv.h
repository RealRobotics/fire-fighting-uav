#include <behaviortree_ros/bt_service_node.h>
#include <ros/ros.h>
#include "uav_msgs/EnableWaterMonitor.h"


#ifndef SPRAY_BT_SERVICE_H
#define SPRAY_BT_SERVICE_H

class EnableWaterMonitorAction: public BT::RosServiceNode<uav_msgs::EnableWaterMonitor>
{

public:
  EnableWaterMonitorAction( ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();
  void sendRequest(RequestType& request);
  BT::NodeStatus onResponse(const ResponseType& rep) override;
  virtual BT::NodeStatus onFailedRequest(RosServiceNode::FailureCause failure) override;

private:
  bool expected_result_;
};

#endif //SPRAY_BT_SERVICE_H
