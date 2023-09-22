#include <fstream>

#include <jsoncpp/json/json.h>
#include "mission_controller/fly_to_wp_bt_action.h"
#include "mission_controller/special_movement_bt_action.h"
#include "mission_controller/is_battery_required_status.h"
#include "uav_msgs/SpecialMovement.h"

#include "ros/ros.h"

//TODO It would be great to save value to blackboard, such as BatteryStatus::OK and not have the number here
//Ha, it is just a string, so I can connect multiple strings together!
//but even better, load the string form a file -> and then, I would need the value from blackboard
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

void saveWpToBlackboard(Json::Value wp, std::string wp_name, BT::Blackboard::Ptr blackboard) {
  sensor_msgs::NavSatFix gps_wp;
  gps_wp.latitude = wp["latitude"].asDouble();
  gps_wp.longitude = wp["longitude"].asDouble();

  blackboard->set(wp_name, gps_wp);
}

void loadWaypoints(char ** argv, BT::Blackboard::Ptr blackboard) {
  std::ifstream search_pattern_file; 
  search_pattern_file.open(argv[1]);
  Json::Value root;
  if (search_pattern_file.is_open()) {
    Json::CharReaderBuilder builder;
    builder["collectComments"] = false;
    JSONCPP_STRING errs;
    if (!parseFromStream(builder, search_pattern_file, &root, &errs)) {
      std::cout << errs << std::endl;
      return;
    } else {
      int num_waypoints = root["num_waypoints"].asInt();
      for (uint i = 0; i << num_waypoints; ++i) {
        std::string wp = "wp" + std::to_string(i);
        saveWpToBlackboard(root[wp], wp, blackboard);
      }
    }
  }
}

void loadBatteryLimits(BT::Blackboard::Ptr blackboard) {
  blackboard->set("battery_ok", uint8_t(uav_msgs::BatteryStatus::OK));
  blackboard->set("battery_mission_critical", uint8_t(uav_msgs::BatteryStatus::MISSION_CRITICAL));
  blackboard->set("battery_safety_critical", uint8_t(uav_msgs::BatteryStatus::SAFETY_CRITICAL));
}

void loadSpecialMovementsCmds(BT::Blackboard::Ptr blackboard) {
  blackboard->set("take_off", uav_msgs::SpecialMovement::TAKE_OFF);
  blackboard->set("land", uav_msgs::SpecialMovement::LAND);
  blackboard->set("go_home", uav_msgs::SpecialMovement::GO_HOME);
}

int main(int argc, char **argv) {
  if(argc < 2) {
    ROS_WARN("You must provide path to a file with a search pattern");
    return 0;
  }
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;  

  BT::BehaviorTreeFactory factory;

  //BT::RegisterRosAction<FlyToWpBTAction>(factory, "FlyToWp", nh);
  BT::RegisterRosAction<SpecialMovementBTAction>(factory, "SpecialMovement", nh);
  IsBatteryRequiredStatus::Register(factory, "IsBatteryRequiredStatus", nh);

  auto tree = factory.createTreeFromText(xml_text);
  loadWaypoints(argv, tree.rootBlackboard());
  loadBatteryLimits(tree.rootBlackboard());
  loadSpecialMovementsCmds(tree.rootBlackboard());  

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