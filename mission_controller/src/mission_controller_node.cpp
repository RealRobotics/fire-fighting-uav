

#include "ros/ros.h"
#include "mission_controller/mission_controller.h"

static const char* xml_text = R"(
 <root>
     <BehaviorTree>
      <FlyToWpBTAction server_name="FlyToWp" waypoint="0.7"/>
     </BehaviorTree>
 </root>
 )";
//<SpecialMovementBTAction type_of_action="{take_off}"/>
/*
 <Inverter>
            <KeepRunningUntilFailure>
              <Inverter>
                <IsMissionEnabled mission_enabled="{mission_enabled}"/>
              </Inverter>
            </KeepRunningUntilFailure>
          </Inverter>
          <IsBatteryRequiredStatus required_status="{battery_ok}"/>
          */
int main(int argc, char **argv) {
  if(argc < 2) {
    ROS_WARN("You must provide path to a file with a search pattern");
    return 0;
  }
  ros::init(argc, argv, "mission_controller");
  ros::NodeHandle nh;  
  MissionController mission_controller = MissionController(nh, xml_text, std::string(argv[1])); 
  mission_controller.run(); 

  ros::spin();
  return 0;
}