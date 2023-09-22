#ifndef SPECIAL_MOVEMENT_BT_ACTION_H
#define FSPECIAL_MOVEMENT_BT_ACTION_H

#include "behaviortree_ros/bt_action_node.h"
#include "uav_msgs/SpecialMovementAction.h"

class SpecialMovementBTAction : public BT::RosActionNode<uav_msgs::SpecialMovementAction>{
 public:
  SpecialMovementBTAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration & conf);
  static BT::PortsList providedPorts();
  bool sendGoal(GoalType& goal) override;
  void halt() override;
  BT::NodeStatus onResult( const ResultType& res) override;
  virtual BT::NodeStatus onFailedRequest(FailureCause failure) override;

 private:
};

#endif //SPECIAL_MOVEMENT_BT_ACTION_H