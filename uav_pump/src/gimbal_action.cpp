#include "gimbal_action.h"

#include <actionlib/server/simple_action_server.h>
#include <uav_msgs/LeedsPumpGimbalAction.h>
#include <ros/ros.h>

#include "communications.h"

// Consts
// Rate of looping. 10.0 = 100ms per loop.
#define UPDATE_RATE_PER_S (10.0)

GimbalAction::GimbalAction(std::string& name, Communications* comms)
  : action_server_(node_handle_, name, boost::bind(&GimbalAction::executeCallBack, this, _1), false)
  , action_name_(name)
  , comms_(comms)
{
  action_server_.start();
}

GimbalAction::~GimbalAction()
{
}

void GimbalAction::executeCallBack(const uav_msgs::LeedsPumpGimbalGoalConstPtr& goal)
{
  ros::Rate rate_s(UPDATE_RATE_PER_S);
  bool success = false;
  int pitch = 0;
  int yaw = 0;

  ROS_DEBUG("%s: Executing call back. pitch %d, yaw %d", action_name_.c_str(), goal->pitch, goal->yaw);

  // Send command
  comms_->setGimbal(goal->pitch, goal->yaw);
  ROS_DEBUG("%s: Executing call back. 1", action_name_.c_str());
  // Wait for command to complete
  while (ros::ok())
  {
    // Break out of loop if preempted or ROS dies.
    if (action_server_.isPreemptRequested())
    {
      action_server_.setPreempted();
      success = false;
      break;
    }
    // Get current pitch value.
    success = comms_->getGimbal(&pitch, &yaw);
    if (success)
    {
      break;
    }
    // Report feedback
    feedback_.pitch = pitch;
    feedback_.yaw = yaw;
    action_server_.publishFeedback(feedback_);
    // Slow the loop down so other things can happen.
    rate_s.sleep();
  }
  ROS_DEBUG("%s: Executing call back. 2", action_name_.c_str());

  // Notfy action complete
  if (success)
  {
    result_.pitch = pitch;
    ROS_DEBUG("%s: Succeeded.", action_name_.c_str());
    action_server_.setSucceeded(result_);
  }
}
