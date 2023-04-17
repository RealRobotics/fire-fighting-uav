#include <ros/ros.h>
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <string>
#include "std_msgs/String.h"

//*********Global Variable Declarations******//
std::string batteryStatus = "UNSET";
bool InputAccepted = false;
ros::Publisher battery_status_pub;
ros::Publisher input_pub;
ros::Subscriber battery_sub;
std::array<std::string, 3> Battery_State = {"Ok", "MissionCritical", "SafetyCritical"}; 									//[Ok, Mission_Critical, Safety_Critical]
float Battery_Percentage = float(0); 																						//This is a float value for the battery percentages
static int Battery_Safety_Threshold = 30;
static int Battery_Mission_Threshold = 40;
//******************************************//

//***************************************Callback & message definations************************************************//
void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)														//Battery Callback function
{
	InputAccepted = true;
	if (msg->percentage > Battery_Mission_Threshold) 
	{
	batteryStatus = Battery_State[0];																						//State = OK
	} 
	else if ((msg->percentage <= Battery_Mission_Threshold) && (msg->percentage > Battery_Safety_Threshold )) 
	{
	batteryStatus = Battery_State[1];																						//State = Mission_Critical
	} 
	else
	{		
	batteryStatus = Battery_State[2];																						//State = Safety_Critical
	}

	std_msgs::Bool msg2;																									//Input Accepted Boolean Function//
	msg2.data = InputAccepted;
	input_pub.publish(msg2);

	std_msgs::String msg3;																									//Battery Status Function//
	msg3.data = batteryStatus;
	battery_status_pub.publish(msg3);

	InputAccepted = false;
}	
//**********************************************END********************************************************//

//*******************************This is the injected error function starts*********************************//
void timerCallback(const ros:: TimerEvent& event)
{
	static int CallCount = 0;
	if (CallCount == 0)
	{
	std_msgs::String msg3;
	msg3.data = std::string("Ok");
	battery_status_pub.publish(msg3);
	CallCount++;
	}
	else{
	ROS_INFO ("Battery monitor Timed Out: %i\n", CallCount++);
	}
}
//**********************************************************************************************************//

//**************************************Main function starts here******************************************//
int main(int argc, char **argv)
{
	ROS_INFO("Battery_Monitor_Running");

	if (Battery_Mission_Threshold < Battery_Safety_Threshold)																// READ THE THRESHOLDS FROM SETTINGS FILE; Battery threshold setting check loop
	{																										
	ROS_ERROR("Mission threshold is below Safety threshold! Please fix the settings");
	ros::shutdown();
	}

    ros::init(argc, argv, "battery_monitor");
    ros::NodeHandle nh;
	
	//Publishers//
	input_pub = nh.advertise<std_msgs::Bool>("input_accepted",1000);
	battery_status_pub = nh.advertise<std_msgs::String>("battery_status", 1000);
	
	//Subscribers//
	//ros::Subscriber battery_sub = nh.subscribe("dji_sdk/battery_state", 10, batteryCallback); 							// uncomment for main implementation
	battery_sub = nh.subscribe("battery_state_test", 10, batteryCallback);													// This is for testing; comment out for main implementation
  	
	//Timer for inhected error function//
	ros::Timer timer=nh.createTimer(ros::Duration(10), timerCallback);													
	
	//Continuos spin
	ros::spin();

    return 0;
}
//**********************************************END********************************************************//