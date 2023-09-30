

#include "ros/ros.h"
#include "mission_controller/mission_controller.h"

int main(int argc, char **argv) {
  if(argc < 3) {
    ROS_WARN("You must provide two paths to files: first file containts a search pattern, the second containts the BT tree");
    return 0;
  }
  std::string search_pattern_file = std::string(argv[1]);
  std::string bt_tree_file = std::string(argv[2]);

  if (search_pattern_file.find("json") == std::string::npos) {
    ROS_WARN("the first file must be json containing the search pattern!");
    return 0;
  }

  if (bt_tree_file.find("xml") == std::string::npos) {
    ROS_WARN("the second file must be xml file containing the behaviour tree specification!");
    return 0;
  }

  ros::init(argc, argv, "mission_controller");
  ros::NodeHandle nh;  
  MissionController mission_controller = MissionController(nh, bt_tree_file, search_pattern_file); 
  mission_controller.run(); 

  ros::spin();
  return 0;
}