#include "mission_controller/repeat_over_vector.h"

RepeatOverVector::RepeatOverVector(const std::string& name, std::vector<sensor_msgs::NavSatFix> vector) :
  DecoratorNode(name, {}),
  vector_(vector),
  repeat_count_(0),
  read_parameter_from_ports_(false)
{
  setRegistrationID("RepeatOverVector");
}

RepeatOverVector::RepeatOverVector(const std::string& name, const BT::NodeConfiguration& config) :
  DecoratorNode(name, config),
  repeat_count_(0),
  read_parameter_from_ports_(true)
{}

BT::PortsList RepeatOverVector::providedPorts()
{
  return {BT::InputPort<std::vector<sensor_msgs::NavSatFix>>("vector", "Repeat a successful child up for each element of the vector."),
          BT::OutputPort<sensor_msgs::NavSatFix>("next_waypoint")};
}

BT::NodeStatus RepeatOverVector::tick()
{
  if (read_parameter_from_ports_)
  {
    if (!getInput("vector", vector_))
    {
      throw BT::RuntimeError("Missing parameter [vector] in RepeatOverVector node");
    }
  }

  int num_cycles = vector_.size();

  bool do_loop = repeat_count_ < num_cycles;

  setStatus(BT::NodeStatus::RUNNING);

  while (do_loop)
  {
    setOutput("next_waypoint", vector_.at(repeat_count_));
    BT::NodeStatus child_status = child_node_->executeTick();

    switch (child_status)
    {
      case BT::NodeStatus::SUCCESS: {
        repeat_count_++;
        do_loop = repeat_count_ < num_cycles;

        resetChild();
      }
      break;

      case BT::NodeStatus::FAILURE: {
        repeat_count_ = 0;
        resetChild();
        return (BT::NodeStatus::FAILURE);
      }

      case BT::NodeStatus::RUNNING: {
        return BT::NodeStatus::RUNNING;
      }

      case BT::NodeStatus::IDLE: {
        throw BT::LogicError("[", name(), "]: A children should not return IDLE");
      }
    }
  }

  repeat_count_ = 0;
  return BT::NodeStatus::SUCCESS;
}

void RepeatOverVector::halt()
{
  repeat_count_ = 0;
  DecoratorNode::halt();
}

void RepeatOverVector::Register(BT::BehaviorTreeFactory& factory,
                        const std::string& registration_ID)
{
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    return std::make_unique<RepeatOverVector>(name, config);
  };

  BT::TreeNodeManifest manifest;
  manifest.type = BT::getType<RepeatOverVector>();
  manifest.ports = RepeatOverVector::providedPorts();
  manifest.registration_ID = registration_ID;
  factory.registerBuilder( manifest, builder );
}