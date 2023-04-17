#include <ros/ros.h>
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <string>
#include "std_msgs/String.h"
#include "unistd.h"
#include <iostream>
#include <djiosdk/dji_vehicle.hpp>

using namespace DJI::OSDK;

//*********Global Variables*****************//
std::string batteryStatus = "UNSET";
bool InputAccepted = false;
ros::Publisher battery_status_pub;
ros::Publisher input_pub;
ros::Subscriber battery_sub;
std::array<std::string, 3> Battery_State = {"Ok", "MissionCritical", "SafetyCritical"}; 								//This array has three values in order[Ok, Mission_Critical, Safety_Critical]
float Battery_Percentage = float(0); 																					//This is a float value for the battery percentages
static int Battery_Safety_Threshold = 30;																				//Initial battery safety threshold setting
static int Battery_Mission_Threshold = 40;																				////Initial battery mission threshold setting
//******************************************//

//***************************************Callback & message definations************************************************//
void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)													//Battery Callback function
{
	//InputAccepted = true;			
	if (msg->percentage > Battery_Mission_Threshold) 
	{
		batteryStatus = Battery_State[0];																				//Battery is Ok
		ROS_INFO("Battery OK");
	} 
	else if ((msg->percentage <= Battery_Mission_Threshold) && (msg->percentage > Battery_Safety_Threshold )) 
	{
		batteryStatus = Battery_State[1];																				//Battery is Mission Critical
		ROS_INFO("Mission Critical");
	} 
	else
	{		
		batteryStatus = Battery_State[2];																				//Battery is Safety Critical
		ROS_INFO("Safety Critical");
	}

	//std_msgs::Bool msg2;
	//msg2.data = InputAccepted;
	//input_pub.publish(msg2);

	//std_msgs::String msg3;
	//msg3.data = batteryStatus;
	//battery_status_pub.publish(msg3);

	//InputAccepted = false;
}	
//*********************************************************************************************************************//

//**************************************Main function starts here******************************************//
int main(int argc, char **argv)
{
	ROS_INFO("Battery_Monitor_Running");

	if (Battery_Mission_Threshold<Battery_Safety_Threshold)																// READ THE THRESHOLDS FROM SETTINGS FILE; Battery threshold setting check loop
	{
		ROS_ERROR("Mission threshold is below Safety threshold! Please fix the settings");
		ros::shutdown();
	}

    ros::init(argc, argv, "battery_monitor");
    ros::NodeHandle nh;
	
	//Publishers//
	//input_pub = nh.advertise<std_msgs::Bool>("input_accepted",1000);
	//battery_status_pub = nh.advertise<std_msgs::String>("battery_status", 1000);
	
	//Subscribers//
	ros::Subscriber battery_sub = nh.subscribe("dji_sdk/battery_state", 10, batteryCallback); 						// uncomment for main implementation
	//ros::Subscriber battery_sub = nh.subscribe("battery_state_test", 10, batteryCallback);								// This is for testing; comment out for main implementation
	
	//Continuos spin
	ros::spin();

    return 0;
}
//**********************************************END********************************************************//