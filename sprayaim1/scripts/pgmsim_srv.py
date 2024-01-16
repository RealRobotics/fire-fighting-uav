#!/usr/bin/env python

"""
ROS Node: control_server

This ROS node serves as a simulated control server for managing gimbal and pump actions. It utilizes actionlib for handling action requests. 
The node provides two action servers, 'gimbal_control' and 'pump_action', for simulating gimbal positions and pump states, respectively.

Author: Nabil
Date Changed: 08/10/2023

Usage:
- To run the node: rosrun sprayaim1 pgmsim.py
"""

import rospy
import actionlib
from sprayaim1.msg import gimbalAction, gimbalFeedback, gimbalResult, pumpAction, pumpResult

UPDATE_RATE_PER_S = 10
pitch, yaw = 0, 0

class ControlServer:
    def __init__(self):
        self.gimbal_server = actionlib.SimpleActionServer('gimbal_control', gimbalAction, execute_cb=self.gimbal_execute_callback, auto_start=False)
        self.gimbal_server.start()
        
        self.pump_server = actionlib.SimpleActionServer('pump_action', pumpAction, execute_cb=self.pump_execute_callback, auto_start=False)
        self.pump_server.start()

    def gimbal_execute_callback(self, goal):
        rate_s = rospy.Rate(UPDATE_RATE_PER_S)

        rospy.loginfo("Gimbal Control: Executing callback. pitch %d, yaw %d", goal.pitch, goal.yaw)

        rospy.sleep(2)  # Simulating some processing time

        result = gimbalResult()
        result.pitch = goal.pitch
        result.yaw = goal.yaw

        rospy.loginfo("Gimbal Control: Succeeded.")
        self.gimbal_server.set_succeeded(result)

    def pump_execute_callback(self, goal):
        rospy.logdebug("Pump Control: Executing callback. running %r", goal.start_pump)

        rospy.sleep(2)  # Simulating some processing time

        result = pumpResult()

        # Assuming the pump is always successful for demonstration purposes
        result.running = goal.start_pump

        rospy.loginfo("Received Pump Action Goal: {}".format("Start" if goal.start_pump else "Stop"))
        rospy.loginfo("Pump Control: Current pump state - Running: %s", goal.start_pump)

        self.pump_server.set_succeeded(result)


def main():
    rospy.init_node('control_server')
    server = ControlServer()
    rospy.loginfo("Simulated Gimbal and Pump Control Server is Running")
    rospy.spin()

if __name__ == '__main__':
    main()
