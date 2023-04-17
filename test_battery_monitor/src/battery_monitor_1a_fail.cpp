#include <ros/ros.h>
#include "sensor_msgs/BatteryState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <string>
#include "std_msgs/String.h"


//Variable declaration//
std::string batteryStatus;
float batteryInfo_check;
bool InputAccepted = false;
std::string battery_status_check;
ros::Publisher battery_status_pub;
ros::Publisher input_pub;
ros::Publisher battery_status_check_pub;
ros::Publisher battery_info_check_pub;



void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)													//Battery Callback function
{
    
	

	InputAccepted = true;
	
	std::array<std::string, 3> Battery_State = {"Ok", "MissionCritical", "SafetyCritical"}; 							//[Ok, Mission_Critical, Safety_Critical]
	
	float Battery_Percentage = float(0); 																				//This is a float value for the battery percentages
	int Battery_Safety_Threshold = 30;
	int Battery_Mission_Threshold = 40;

	if (Battery_Mission_Threshold<Battery_Safety_Threshold)
	{
		ROS_ERROR("Mission threshold is below Safety threshold! Please fix the settings");
	}
	else
	{					
			if (msg->percentage > Battery_Mission_Threshold) 
							{
								batteryStatus = Battery_State[0];															//State = OK

							} 
			else if ((msg->percentage <= Battery_Mission_Threshold) && (msg->percentage > Battery_Safety_Threshold )) 
								{
									batteryStatus = Battery_State[1];														//State = Mission_Critical
	
									//ROS_INFO("Battery_Mission_Critical");
								} 
			else
									{		
										batteryStatus = Battery_State[2];													//State = Safety_Critical

										//ROS_INFO("Battery_Safety_Critical!!");
									}
		
					
    }
		//Input Accepted Boolean Function//
			std_msgs::Bool msg2;
			msg2.data = InputAccepted;
			input_pub.publish(msg2);
			//ROS_INFO("Input_Publishing");

		//Battery Status Function//
			std_msgs::String msg3;
			msg3.data = batteryStatus;
			battery_status_pub.publish(msg3);
			//ROS_INFO("Publishing_Battery Status");
		
		//Status Check Function//
			std_msgs::String msg4;
   			msg4.data = battery_status_check;
    		battery_status_check_pub.publish(msg);

		//BatteryInfo check Function//
			sensor_msgs::BatteryState msg5;
			msg5.percentage = batteryInfo_check;
			battery_info_check_pub.publish(msg5);

		
}	


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
    ROS_INFO ("Batt Mon Timed Out Loop: %i\n", CallCount++);
  }
}

int main(int argc, char **argv)
{
	ROS_INFO("Battery_Monitor_Running");
	//sensor_msgs::BatteryState msg;
	//ROS_INFO("batteryInfo: %f", msg.percentage);

    ros::init(argc, argv, "battery_monitor");
    ros::NodeHandle nh;
	
	//Publisher//
	input_pub = nh.advertise<std_msgs::Bool>("input_accepted",1000);
	battery_status_pub = nh.advertise<std_msgs::String>("battery_status", 1000);
	battery_status_check_pub = nh.advertise<std_msgs::String>("battery_status_check", 1000);
	battery_info_check_pub = nh.advertise<sensor_msgs::BatteryState>("batteryInfo_check", 1000);
	
	//Subscriber//
    ros::Subscriber battery_sub = nh.subscribe("test", 10, batteryCallback);
	

	//This is an injected Error!!!!
  	ros::Timer timer=nh.createTimer(ros::Duration(10),timerCallback);


	ros::spin();

    return 0;
}