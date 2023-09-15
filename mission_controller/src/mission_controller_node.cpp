#include "mission_controller/fly_to_wp_bt_action.h"
#include "mission_controller/is_battery_ok.h"

#include "ros/ros.h"

//TODO It would be great to save value to blackboard, such as BatteryStatus::OK and not have the number here
static const char* xml_text = R"(
 <root >
     <BehaviorTree>
        <KeepRunningUntilFailure>
          <Inverter>
            <IsBatteryOk required_status="2"/>
          </Inverter>
        </KeepRunningUntilFailure>
     </BehaviorTree>
 </root>
 )";


int main(int argc, char **argv) {
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;

  BT::BehaviorTreeFactory factory;

  BT::RegisterRosAction<FlyToWpAction>(factory, "FlyToWp", nh);
  IsBatteryOk::Register(factory, "IsBatteryOk", nh);

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