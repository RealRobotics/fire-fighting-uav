#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <string>
#include "std_msgs/String.h"

//******Global Variable Declarations*********//
static int Time_Out = 15;                                                             //Time out fucntion delay of 15 seconds
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
  ROS_INFO ("Pub_Timed Out Loop: %i\n", CallCount++);                                  // This is just to show the node is alive!
  }
}
//*************************************************************//

//*************Timeout Timer callback function***************//

void Timeout_Callback(const ros:: TimerEvent& event)                                         
{
  static int CallCount = 0;
  if (CallCount == 0)
  {
    if (msg.percentage == 0)
    { 
    Verdict="Fail";
    ROS_INFO ("Verdict: %s\n", Verdict.c_str());
    }
    else
    {
    Verdict="Pass";
    ROS_INFO ("Verdict: %s\n", Verdict.c_str());
    }
    ROS_INFO ("Final Verdict: %s\n", Verdict.c_str());
    CallCount++;
    ros::shutdown();                                                                   //This is to kill the node after processing finishes!!
  }
  else{
  ROS_INFO ("Test2a_Timed Out Loop: %i\n", CallCount++);                               //We shouldn't reach this case so if we see this output it means the shutdown didn't work
  }
}
//*****************************************************//

//**************************************Main function starts here*************************************//
int main(int argc, char **argv)
{
  ROS_INFO("Test2a_Running");
  ros::init(argc, argv, "test_case_2a");                                               //Test node intialization
  ros::NodeHandle nh;
  
  msg.percentage = Get_Test_Battery_percentage();
  
  //Publishers//
  battery_pub = nh.advertise<sensor_msgs::BatteryState>("battery_state_test", 1000);

  //Timers for running the tets//
  ros::Timer Pub_timer = nh.createTimer(ros::Duration(Init_Wait), Pub_timer_Callback);
  ros::Timer Timeout_timer = nh.createTimer(ros::Duration(Time_Out), Timeout_Callback);

  //Continuos spin//
  ros::spin();

  return 0;
}
//********************************************END******************************************************//