#ifndef PUMP_ACTION_H
#define PUMP_ACTION_H

#include <actionlib/server/simple_action_server.h>
#include <uav_msgs/LeedsPumpPumpAction.h>
#include <ros/ros.h>

class Communications;

class PumpAction
{
public:
  PumpAction(std::string& name, Communications *comms);
  ~PumpAction();
  void executeCallBack(const uav_msgs::LeedsPumpPumpGoalConstPtr& goal);

protected:
  ros::NodeHandle node_handle_;
  // NodeHandle instance must be created before this line. Otherwise strange
  // error occurs.
  actionlib::SimpleActionServer<uav_msgs::LeedsPumpPumpAction> action_server_;
  std::string action_name_;
  // Create messages that are used to published feedback/result
  uav_msgs::LeedsPumpPumpFeedback feedback_;
  uav_msgs::LeedsPumpPumpResult result_;

private:
  void publishLitresRemaining(const float litres_remaining);
  Communications *comms_;
  ros::Publisher litres_publisher_;
};

#endif
