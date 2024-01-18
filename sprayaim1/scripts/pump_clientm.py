#!/usr/bin/env python
"""
ROS Pump Control Node

This node provides a function 'pump_control' to start or stop a pump using ROS actionlib.
It initializes a SimpleActionClient to communicate with the 'pump_action' action server
and sends a goal message to control the pump action. The result from the action server is
used to log whether the pump is running or stopped.

Author: Nabl
Last Changed Date: 7-08-2023 

Usage:
- To start the pump: pump_control(True)
- To stop the pump: pump_control(False)
"""
import rospy
import actionlib
from sprayaim1.msg import pumpAction, pumpGoal, pumpFeedback, pumpResult

def pump_control(start_pump):
    """
    Control the pump action.

    :param start_pump: If True, start the pump. If False, stop the pump.
    :type start_pump: bool
    """
    # Initialize the ROS node for the pump client
   # rospy.init_node('pump_client')

    # Create a SimpleActionClient for the 'pump_action' action server
    client = actionlib.SimpleActionClient('pump_action', pumpAction)
    client.wait_for_server()

    # Create a goal message to send to the action server
    goal = pumpGoal()
    goal.start_pump = start_pump

    # Log the action that is being sent
    rospy.loginfo("Sending Pump Action Goal: {}".format("Start" if start_pump else "Stop"))

    # Send the goal to the action server
    client.send_goal(goal)

    # Log that we are waiting for the action to complete
    rospy.loginfo("Waiting for Pump Action to Complete...")

    # Wait for the action to complete and get the result
    client.wait_for_result()
    result = client.get_result()

    # Check the result to determine if the pump is running or stopped
    if result.running:
        rospy.loginfo("Pump Action Completed: Pump is Running")
    else:
        rospy.loginfo("Pump Action Completed: Pump is Stopped")
    return result