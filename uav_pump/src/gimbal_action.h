#ifndef GIMBAL_ACTION_H
#define GIMBAL_ACTION_H

#include <actionlib/server/simple_action_server.h>
#include <uav_msgs/LeedsPumpGimbalAction.h>
#include <ros/ros.h>

class Communications;

class GimbalAction
{
public:
  GimbalAction(std::string& name, Communications* comms);
  ~GimbalAction();
  void executeCallBack(const uav_msgs::LeedsPumpGimbalGoalConstPtr& goal);

protected:
  ros::NodeHandle node_handle_;
  // NodeHandle instance must be created before this line. Otherwise strange
  // error occurs.
  actionlib::SimpleActionServer<uav_msgs::LeedsPumpGimbalAction> action_server_;
  std::string action_name_;
  // Create messages that are used to published feedback/result
  uav_msgs::LeedsPumpGimbalFeedback feedback_;
  uav_msgs::LeedsPumpGimbalResult result_;

private:
  Communications* comms_;
};

#endif
