#ifndef REPEAT_OVER_VECTOR_H
#define REPEAT_OVER_VECTOR_H

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "sensor_msgs/NavSatFix.h"

class RepeatOverVector : public BT::DecoratorNode
{
public:
  
  RepeatOverVector(const std::string& name, std::vector<sensor_msgs::NavSatFix> vector);

  RepeatOverVector(const std::string& name, const BT::NodeConfiguration& config);

  virtual ~RepeatOverVector() override = default;

  static BT::PortsList providedPorts();

  static void Register(BT::BehaviorTreeFactory& factory,
                         const std::string& registration_ID);

private:
  std::vector<sensor_msgs::NavSatFix> vector_;
  int repeat_count_;
  bool all_skipped_ = true;

  bool read_parameter_from_ports_;

  virtual BT::NodeStatus tick() override;

  void halt() override;
};

#endif //REPEAT_OVER_VECTOR_H