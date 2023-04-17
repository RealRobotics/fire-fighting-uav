#include "pump_action.h"

#include <ros/ros.h>
#include <std_msgs/Int16.h>

#include <actionlib/server/simple_action_server.h>
#include <uav_msgs/LeedsPumpPumpAction.h>
#include <uav_msgs/LeedsPumpLevel.h>

#include "communications.h"

// Consts
// Rate of looping. 2.0 = 500ms per loop.
#define UPDATE_RATE_PER_S (2.0)

PumpAction::PumpAction(std::string& name, Communications* comms)
  : action_server_(node_handle_, name, boost::bind(&PumpAction::executeCallBack, this, _1), false)
  , action_name_(name)
  , comms_(comms)
{
  action_server_.start();
  ros::NodeHandle node_handle_;
  litres_publisher_ = node_handle_.advertise<uav_msgs::LeedsPumpLevel>("pump/level", 10);
}

PumpAction::~PumpAction()
{
}

void PumpAction::executeCallBack(const uav_msgs::LeedsPumpPumpGoalConstPtr& goal)
{
  ros::Rate rate_s(UPDATE_RATE_PER_S);
  bool done = false;
  bool running = false;

  ROS_DEBUG("%s: Executing call back. running %i", action_name_.c_str(), goal->running);

  // Send command
  comms_->setPump(goal->running);
  // Wait for command to complete
  while (ros::ok())
  {
    // Break out of loop if preempted or ROS dies.
    if (action_server_.isPreemptRequested())
    {
      // TODO tell pump to stop.
      action_server_.setPreempted();
      done = false;
      break;
    }
    // Get current running value.
    float litres_remaining = 0.0;
    done = comms_->getPump(&running, &litres_remaining);
    publishLitresRemaining(litres_remaining);
    if (done)
    {
      break;
    }
    // Report feedback
    ROS_DEBUG("%s: Publishing feedback: %d", action_name_.c_str(), running);
    feedback_.running = running;
    action_server_.publishFeedback(feedback_);
    // Slow the loop down so other things can happen.
    rate_s.sleep();
  }

  // Notfy action complete
  if (done)
  {
    result_.running = running;
    ROS_INFO("%s: Succeeded.", action_name_.c_str());
    action_server_.setSucceeded(result_);
  }
}


// Publish litres remaining.
void PumpAction::publishLitresRemaining(const float litres_remaining) {
  ROS_DEBUG("%s: Publishing litres remaining: %f", action_name_.c_str(), litres_remaining);
  uav_msgs::LeedsPumpLevel message;
  message.litres_remaining = litres_remaining;
  litres_publisher_.publish(message);
}
