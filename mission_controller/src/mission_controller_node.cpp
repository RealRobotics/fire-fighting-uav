#include "mission_controller/fly_to_wp_bt_action.h"
#include "mission_controller/special_movement_bt_action.h"
#include "mission_controller/is_battery_required_status.h"

#include "ros/ros.h"

//TODO It would be great to save value to blackboard, such as BatteryStatus::OK and not have the number here
//Ha, it is just a string, so I can connect multiple strings together!
//but even better, load the string form a file -> and then, I would need the value from blackboard
static const char* xml_text = R"(
 <root>
     <BehaviorTree>
        <Sequence>
          <SpecialMovementBTAction type_of_action="1"/>
          <IsBatteryRequiredStatus required_status="3"/>
          <SpecialMovementBTAction type_of_action="2"/>
        </Sequence>
     </BehaviorTree>
 </root>
 )";


int main(int argc, char **argv) {
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;

  BT::BehaviorTreeFactory factory;

  //BT::RegisterRosAction<FlyToWpBTAction>(factory, "FlyToWp", nh);
  BT::RegisterRosAction<SpecialMovementBTAction>(factory, "SpecialMovement", nh);
  IsBatteryRequiredStatus::Register(factory, "IsBatteryRequiredStatus", nh);

  auto tree = factory.createTreeFromText(xml_text);

  BT::NodeStatus status = BT::NodeStatus::IDLE;

  ROS_INFO("Mission controller on");

  while( ros::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING))
  {
    ros::spinOnce();
    status = tree.tickRoot();
    std::cout << status << std::endl;
    ros::Duration sleep_time(1);
    sleep_time.sleep();
  }

  return 0;
}