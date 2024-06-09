#ifndef SPECIAL_MOVEMENT_BT_ACTION_H
#define FSPECIAL_MOVEMENT_BT_ACTION_H

#include "behaviortree_ros/bt_action_node.h"
#include "uav_msgs/SpecialMovementAction.h"
#include "uav_msgs/BatteryStatus.h"
#include <atomic>

class SpecialMovementBTAction : public BT::RosActionNode<uav_msgs::SpecialMovementAction>{
 public:
  SpecialMovementBTAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration & conf);
  static BT::PortsList providedPorts();
  bool sendGoal(GoalType& goal) override;
  void halt() override;
  BT::NodeStatus onResult( const ResultType& res) override;
  virtual BT::NodeStatus onFailedRequest(FailureCause failure) override;

 private:
 void batteryStatusCallback_(const uav_msgs::BatteryStatus::ConstPtr& message);
 ros::NodeHandle nh_;
 ros::Subscriber battery_status_sub_;
 std::mutex status_mtx_;
 uint8_t current_status_ {uav_msgs::BatteryStatus::SAFETY_CRITICAL};
 uint8_t required_status_ {uav_msgs::BatteryStatus::SAFETY_CRITICAL};
 std::atomic<bool> statementExecuted{false};
};

#endif //SPECIAL_MOVEMENT_BT_ACTION_H