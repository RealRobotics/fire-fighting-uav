#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <uav_msgs/LeedsPumpGimbalAction.h>
#include <uav_msgs/LeedsPumpPumpAction.h>

#define TIMEOUT_S (10)

std::shared_ptr<ros::NodeHandle> nh;


TEST(TESTSuite, pumpGoal)
{
  actionlib::SimpleActionClient<uav_msgs::LeedsPumpPumpAction> pump_action_client("pump", true);
  uav_msgs::LeedsPumpPumpGoal pump_goal;

  pump_action_client.waitForServer();  // will wait for infinite time

  // Drop crane
  ROS_DEBUG("Turning pump on...");
  pump_goal.running = true;
  pump_action_client.sendGoal(pump_goal);

  // wait for the actions to return
  // TODO Change to callback.
  ROS_DEBUG("Waiting...");
  bool pump_ok = pump_action_client.waitForResult(ros::Duration(TIMEOUT_S));
  EXPECT_TRUE(pump_ok);

  ROS_DEBUG("Pump complete.");
}

TEST(TESTSuite, gimbalPitchGoal)
{
  actionlib::SimpleActionClient<uav_msgs::LeedsPumpGimbalAction> gimbal_action_client("gimbal", true);
  uav_msgs::LeedsPumpGimbalGoal gimbal_goal;
  bool gimbal_ok = false;
  gimbal_action_client.waitForServer();  // will wait for infinite time

  // Send negative pitch.
  ROS_DEBUG("Sending gimbal goal.");
  gimbal_goal.pitch = -25;
  gimbal_action_client.sendGoal(gimbal_goal);

  // wait for the actions to return
  // TODO Change to callback.
  ROS_DEBUG("Waiting...");
  gimbal_ok = gimbal_action_client.waitForResult(ros::Duration(TIMEOUT_S));
  EXPECT_TRUE(gimbal_ok);

  // Send pitch.
  ROS_DEBUG("Sending gimbal goal.");
  gimbal_goal.pitch = 25;
  gimbal_action_client.sendGoal(gimbal_goal);

  // wait for the actions to return
  // TODO Change to callback.
  ROS_DEBUG("Waiting...");
  gimbal_ok = gimbal_action_client.waitForResult(ros::Duration(TIMEOUT_S));
  EXPECT_TRUE(gimbal_ok);
}

TEST(TESTSuite, gimbalYawGoal)
{
  actionlib::SimpleActionClient<uav_msgs::LeedsPumpGimbalAction> gimbal_action_client("gimbal", true);
  uav_msgs::LeedsPumpGimbalGoal gimbal_goal;
  bool gimbal_ok = false;
  gimbal_action_client.waitForServer();  // will wait for infinite time

  // Send negative yaw.
  ROS_DEBUG("Sending gimbal goal.");
  gimbal_goal.yaw = -35;
  gimbal_action_client.sendGoal(gimbal_goal);

  // wait for the actions to return
  // TODO Change to callback.
  ROS_DEBUG("Waiting...");
  gimbal_ok = gimbal_action_client.waitForResult(ros::Duration(TIMEOUT_S));
  EXPECT_TRUE(gimbal_ok);

  // Send yaw.
  ROS_DEBUG("Sending gimbal goal.");
  gimbal_goal.yaw = 35;
  gimbal_action_client.sendGoal(gimbal_goal);

  // wait for the actions to return
  // TODO Change to callback.
  ROS_DEBUG("Waiting...");
  gimbal_ok = gimbal_action_client.waitForResult(ros::Duration(TIMEOUT_S));
  EXPECT_TRUE(gimbal_ok);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "leeds_pump_client");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
