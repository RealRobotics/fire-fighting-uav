#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <string>
#include "std_msgs/String.h"

//******Global Variable Declarations*********//
static int Init_Wait = 3;                                                             //Publisher initial delay for 3 seconds
std::string Verdict="UNSET";
sensor_msgs::BatteryState msg; 
float Battery_Percentage = float(0); 																				          //This is a float value for the battery percentages
static int Battery_Safety_Threshold = 30;                                             //Initial battery mission threshold setting
static int Battery_Mission_Threshold = 40;                                            //Initial battery safety threshold setting
ros::Publisher battery_pub;                                                           //ros battery percentage publisher
//******************************************//

//***********Battery Information Input function****************//
float Get_Test_Battery_percentage()                                                         
{
  Battery_Percentage = Battery_Mission_Threshold+1;
  return Battery_Percentage;                                                          // no need for a return
}
//*************************************************************//

//**************Battery Status Callback function***************//
void battery_status(const std_msgs::String::ConstPtr& msg)                            //Battery Status Callback function
{        
  if (std::string(msg->data.c_str()).compare("MissionCritical") == 0) 
  { 
  Verdict="Fail";
  ROS_INFO ("Final Verdict: %s\n", Verdict.c_str());
  ros::shutdown();
  }
  else 
  {
  Verdict="Pass";                                                                      //This will happen when battery status recieved other than Mission Critical
  ROS_INFO ("Final Verdict: %s\n", Verdict.c_str());
  ros::shutdown();
  }
}
//*************************************************************//

//*************Publisher Timer callback function***************//
void Pub_timer_Callback(const ros:: TimerEvent& event)                                         
{
  static int CallCount = 0;
  if (CallCount == 0)
  {
  battery_pub.publish(msg);
  ROS_INFO ("batteryInfo: %f", msg.percentage);
  CallCount++;
  }
  else{
  ROS_INFO ("Pub_Timed Out Loop: %i\n", CallCount++);                                     // This is just to show the node is alive!
  }
}
//*************************************************************//

//**************************************Main function starts here*************************************//
int main(int argc, char **argv)
{
  ROS_INFO("Test3a_Running");
  ros::init(argc, argv, "test_case_3a");                                              //Test node intialization
  ros::NodeHandle nh;
  
  msg.percentage = Get_Test_Battery_percentage();

  //Publishers//
  battery_pub = nh.advertise<sensor_msgs::BatteryState>("battery_state_test", 1000);

  //Subscribers//
  ros::Subscriber battery_status_sub = nh.subscribe("battery_status", 10, battery_status);

  //Timer for publishing battery information//
  ros::Timer Pub_timer = nh.createTimer(ros::Duration(Init_Wait), Pub_timer_Callback);

  //Continous spin//  
  ros::spin();

  return 0;
}
//********************************************END******************************************************//