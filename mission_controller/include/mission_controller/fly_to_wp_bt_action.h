#ifndef FLY_TO_WP_ACTION_H
#define FLY_TO_WP_ACTION_H

#include "behaviortree_ros/bt_action_node.h"
#include "uav_msgs/FlyToWPAction.h"

class FlyToWpAction : public BT::RosActionNode<uav_msgs::FlyToWPAction>{
 public:
  FlyToWpAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration & conf);
  static BT::PortsList providedPorts();
  bool sendGoal(GoalType& goal) override;
  void halt() override;
  BT::NodeStatus onResult( const ResultType& res) override;
  virtual BT::NodeStatus onFailedRequest(FailureCause failure) override;

 private:
};

#endif //FLY_TO_WP_ACTION_H