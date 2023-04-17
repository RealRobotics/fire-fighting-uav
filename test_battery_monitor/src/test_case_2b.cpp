#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <string>
#include "std_msgs/String.h"

//Variable Declarations//
ros::Timer timer_publisher;
static float battery_percentage = 100.0;
std::string Verdict="Fail";

float Get_Battery_percentage()                                                         //Battery Info In
{
  static float battery_mission_threshold = 40.0;
  battery_percentage = battery_mission_threshold;

  return battery_percentage;
}

void battery_status(const std_msgs::String::ConstPtr& msg) {                              //Battery Status Callback function
      ROS_INFO ("%s: '%s'\n", __func__, msg->data.c_str());
            
        if (std::string(msg->data.c_str()).compare("MissionCritical") == 0){ 
        Verdict="Pass";
        ROS_INFO ("Verdict: %s\n", Verdict.c_str());
        }
        else{
        Verdict="Fail";
        ROS_INFO ("Verdict: %s\n", Verdict.c_str());
        }
      }

/*void input_accepted(const std_msgs::Bool::ConstPtr& msg){                                 //Input Accepted Callback function
      ROS_INFO ("%s: %d\n", __func__, msg->data);
     
      if (msg->data){
      Verdict="Inconclusive";
      ROS_INFO ("Verdict: %s\n", Verdict.c_str());
      }
      else{
      Verdict="Fail";
      ROS_INFO ("Verdict: %s\n", Verdict.c_str());
      }
}

void BatteryStatusCheck(const std_msgs::String::ConstPtr& msg) {                          //Battery Status Callback function
      ROS_INFO ("waiting for another battery_status");
      
      ros::Duration(30.0).sleep();
      
        if (std::string(msg->data.c_str()).compare("Ok") == 0){ 
        Verdict="Fail";
        ROS_INFO ("Verdict: %s\n", Verdict.c_str());
        }
        else{
        Verdict="Pass";
        ROS_INFO ("Verdict: %s\n", Verdict.c_str());
        }
      }*/


void batteryInfoCheck (const sensor_msgs::BatteryState::ConstPtr& msg) {
  ROS_INFO ("waiting for another batteryInfo");
  ros::Duration(30.0).sleep();
      
        if (msg->percentage == 0){ 
        Verdict="Pass";
        ROS_INFO ("Verdict: %s\n", Verdict.c_str());
        }
        else{
        Verdict="Fail";
        ROS_INFO ("Verdict: %s\n", Verdict.c_str());
        }
}  

int main(int argc, char **argv)
{
  ROS_INFO("Test2a_Running");

  sensor_msgs::BatteryState msg;
  int battery_percentage = Get_Battery_percentage();
  msg.percentage = battery_percentage;
  ROS_INFO ("batteryInfo: %f", msg.percentage);

  ros::init(argc, argv, "test_case_2a");
  ros::NodeHandle n;
  
	
  //Publishers//
  ros::Publisher battery_pub = n.advertise<sensor_msgs::BatteryState>("test", 1000);
  ros::Rate loop_rate(1);

  //Subscribers//
  ros::Subscriber battery_status_sub = n.subscribe("battery_status", 10, battery_status);
  ros::Subscriber batteryInfo_sub = n.subscribe("batteryInfo_check", 10, batteryInfoCheck);
  //ros::Subscriber input_sub = n.subscribe("input_accepted", 10, input_accepted);
  //ros::Subscriber battery_status_check_sub = n.subscribe("battery_status_check", 10, BatteryStatusCheck);

  
  //Batter Info loop function//
  int count=3;

  while (ros::ok() && count>0){

      battery_pub.publish(msg);
      ros::spinOnce(); 
      loop_rate.sleep();
      count--;
  }
  
  return 0;
}