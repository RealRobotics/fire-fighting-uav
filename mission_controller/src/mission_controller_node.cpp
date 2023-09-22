

#include "ros/ros.h"
#include "mission_controller/mission_controller.h"

static const char* xml_text = R"(
 <root>
     <BehaviorTree>
        <KeepRunningUntilFailure>
          <Inverter>
            <IsBatteryRequiredStatus required_status="{battery_mission_critical}"/>
          </Inverter>
        </KeepRunningUntilFailure>
     </BehaviorTree>
 </root>
 )";

int main(int argc, char **argv) {
  if(argc < 2) {
    ROS_WARN("You must provide path to a file with a search pattern");
    return 0;
  }
  ros::init(argc, argv, "mission_controller");
  ros::NodeHandle nh;  
  MissionController mission_controller = MissionController(nh, xml_text, std::string(argv[1]));  

  while(ros::ok()) {
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  return 0;
}