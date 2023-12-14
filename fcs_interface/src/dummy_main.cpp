#include "fcs_interface/dummy_fcs_interface.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "dummy_fcs_interface");
  ros::NodeHandle nh;
  DummyFCS_Interface interface {nh};
  interface.start();
  ros::spin();
}