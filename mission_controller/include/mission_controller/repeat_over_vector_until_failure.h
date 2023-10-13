#ifndef REPEAT_OVER_VECTOR_UNTIL_FAILURE_H
#define REPEAT_OVER_VECTOR_UNTIL_FAILURE_H

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "uav_msgs/GpsLocationWithPrecision.h"

class RepeatOverVectorUntilFailure : public BT::DecoratorNode
{
public:
  
  RepeatOverVectorUntilFailure(const std::string& name, std::vector<uav_msgs::GpsLocationWithPrecision> vector);

  RepeatOverVectorUntilFailure(const std::string& name, const BT::NodeConfiguration& config);

  virtual ~RepeatOverVectorUntilFailure() override = default;

  static BT::PortsList providedPorts();

  static void Register(BT::BehaviorTreeFactory& factory,
                         const std::string& registration_ID);

private:
  std::vector<uav_msgs::GpsLocationWithPrecision> vector_;
  int repeat_count_;
  bool all_skipped_ = true;

  bool read_parameter_from_ports_;

  virtual BT::NodeStatus tick() override;

  void halt() override;
};

#endif //REPEAT_OVER_VECTOR_UNTIL_FAILURE_H