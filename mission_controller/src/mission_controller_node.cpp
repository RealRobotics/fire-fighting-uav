#include "mission_controller/fly_to_wp_bt_action.h"

#include "ros/ros.h"

static const char* xml_text = R"(
 <root >
     <BehaviorTree>
        <Sequence>
          <FlyToWp server_name="FlyToWp" waypoint="5" />
        </Sequence>
     </BehaviorTree>
 </root>
 )";


int main(int argc, char **argv) {
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;

  BT::BehaviorTreeFactory factory;

  BT::RegisterRosAction<FlyToWpAction>(factory, "FlyToWp", nh);

  auto tree = factory.createTreeFromText(xml_text);

  BT::NodeStatus status = BT::NodeStatus::IDLE;

  while( ros::ok() && (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING))
  {
    ros::spinOnce();
    status = tree.tickRoot();
    std::cout << status << std::endl;
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }

  return 0;
}