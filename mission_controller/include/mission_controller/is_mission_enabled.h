#ifndef IS_MISSION_ENABLED_H
#define IS_MISSION_ENABLED_H

#include "behaviortree_cpp_v3/condition_node.h"
#include <behaviortree_cpp_v3/bt_factory.h>
#include "ros/ros.h"

class IsMissionEnabled : public BT::ConditionNode {

public:
  IsMissionEnabled(const std::string & instance_name, const BT::NodeConfiguration & conf, ros::NodeHandle& nh);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

  static void Register(BT::BehaviorTreeFactory& factory,
                         const std::string& registration_ID,
                         ros::NodeHandle& nh);

private:
  ros::NodeHandle nh_;

};

#endif //IS_MISSION_ENABLED_H