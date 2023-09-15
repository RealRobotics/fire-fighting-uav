#ifndef IS_BATTERY_OK_H
#define IS_BATTERY_OK_H

#include <mutex>

#include "behaviortree_cpp_v3/condition_node.h"
#include <behaviortree_cpp_v3/bt_factory.h>
#include "ros/ros.h"
#include "uav_msgs/BatteryStatus.h"

class IsBatteryOk : public BT::ConditionNode {

public:
  IsBatteryOk(const std::string & instance_name, const BT::NodeConfiguration & conf,
              ros::NodeHandle& nh);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

  static void Register(BT::BehaviorTreeFactory& factory,
                         const std::string& registration_ID,
                         ros::NodeHandle& node_handle);

private:
  void batteryStatusCallback_(const uav_msgs::BatteryStatus::ConstPtr& message);
  ros::NodeHandle nh_;
  ros::Subscriber battery_status_sub_;
  std::mutex status_mtx_;
  uint8_t current_status_ {uav_msgs::BatteryStatus::UNSET};
  int required_status_ {uav_msgs::BatteryStatus::UNSET};

};

#endif //IS_BATTERY_OK_H