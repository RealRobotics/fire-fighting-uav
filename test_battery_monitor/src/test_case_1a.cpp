#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <string>
#include "std_msgs/String.h"

//******Global Variable Declarations*********//
std::string Verdict = "UNSET";
static int Time_Out = 15;                                                                                       //Time out fucntion delay of 15 seconds
ros::Subscriber battery_status_sub;
//******************************************//

//************Battery Status Callback function*************//
void battery_status(const std_msgs::String::ConstPtr& msg) 
{
  ROS_INFO ("Test message received: %s\n", msg->data.c_str());                                                  //This is the test message for debugging
  if (std::string(msg->data.c_str()).compare("Ok") == 0)
  { 
  Verdict = "Fail";
  ROS_INFO ("Final Verdict_to: %s\n", Verdict.c_str());
  ros::shutdown();
  }
  else
  {
  Verdict = "Pass";
  ROS_INFO ("Change_Verdict_to: %s\n", Verdict.c_str());
  }
}
//*********************************************************//

//*************Timeout Timer callback function***************//
void Timeout_Callback(const ros:: TimerEvent& event)
{
  static int CallCount = 0;
  if (CallCount == 0)
  {
  if(std::string(Verdict.c_str()).compare("UNSET") == 0)
  {
  Verdict = "Pass";
  }
  ROS_INFO ("Final Verdict: %s\n", Verdict.c_str());
  CallCount++;
  ros::shutdown();                                                                                              //This is to kill the node after processing finishes!!
  }
  else
  {
  ROS_INFO ("Error after Timed Out Loop: %i\n", CallCount++);                                                    /*We shouldn't reach this case so if we see this output*/
  }                                                                                                               /*it means the shutdown didn't work*/
}
//**********************************************************//

//**************************************Main function starts here*************************************//
int main(int argc, char **argv)
{
  ROS_INFO("Test1a_Running");
  ros::init(argc, argv, "test_case_1a");                                                                        //Test node initilization
  ros::NodeHandle nh;

  //Subscribers//
  battery_status_sub = nh.subscribe("battery_status", 10, battery_status);
  
  //Timer for running test//
  ros::Timer timer = nh.createTimer(ros::Duration(Time_Out), Timeout_Callback);
  
  //Continous spin//
  ros::spin();
  
  return 0;
}
//********************************************END******************************************************//