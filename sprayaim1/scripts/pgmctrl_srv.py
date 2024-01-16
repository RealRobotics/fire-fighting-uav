#!/usr/bin/env python
"""
ROS Node: control_server

This ROS node serves as a control server for managing real hardware gimbal and pump actions. It utilizes actionlib for handling action requests. The node provides two action servers, 'gimbal_control' and 'pump_action', for controlling gimbal positions and pump states, respectively.

Author: Nabil
Date Changed: 08/10/2023

Usage:
- To run the node: rosrun sprayaim1 pgmctrl.py
"""
import rospy
import actionlib
from sprayaim1.msg import gimbalAction, gimbalFeedback, gimbalResult, pumpAction, pumpResult
from communications import Communications

UPDATE_RATE_PER_S = 10
pitch, yaw = 0, 0

class ControlServer:
    def __init__(self, comms):
        self.gimbal_server = actionlib.SimpleActionServer('gimbal_control', gimbalAction, execute_cb=self.gimbal_execute_callback, auto_start=False)
        self.gimbal_server.start()
        
        self.pump_server = actionlib.SimpleActionServer('pump_action', pumpAction, execute_cb=self.pump_execute_callback, auto_start=False)
        self.pump_server.start()

        self.comms = comms

    def gimbal_execute_callback(self, goal):
        rate_s = rospy.Rate(UPDATE_RATE_PER_S)
        success = False

        rospy.loginfo("Gimbal Control: Executing callback. pitch %d, yaw %d", goal.pitch, goal.yaw)

        pitch, yaw = self.comms.getGimbal()  

        self.comms.setGimbal(goal.pitch, goal.yaw)  

        while not rospy.is_shutdown():
            if self.gimbal_server.is_preempt_requested():
                self.gimbal_server.set_preempted()
                success = False
                break

            success = self.comms.getGimbal()
            if success:
                break

            rate_s.sleep()

        if success:
            result = gimbalResult()
            result.pitch = pitch  
            result.yaw = yaw
            rospy.loginfo("Gimbal Control: Succeeded.")
            self.gimbal_server.set_succeeded(result)

    def pump_execute_callback(self, goal):
        rospy.logdebug("Pump Control: Executing callback. running %r", goal.start_pump)

        # Set pump status based on the goal received
        self.comms.setPump(goal.start_pump)

        # Create an instance of result message to send back to the client
        result = pumpResult()

        # Log the received pump action goal, indicating whether it's a start or stop command
        rospy.loginfo("Received Pump Action Goal: {}".format("Start" if goal.start_pump else "Stop"))

        # Get the current state of the pump
        current_running, current_litres_remaining = self.comms.getPump()

        # Log the current pump state
        rospy.loginfo("Pump Control: Current pump state - Running: %s, Litres Remaining: %f", current_running, current_litres_remaining)

        # Set the result message to indicate the final status of the pump after executing the goal
        result.running = current_running

        # Indicate the action has been successfully completed with the provided result
        self.pump_server.set_succeeded(result)


def main():
    rospy.init_node('control_server')
    comms = Communications()
    comms.init()
    server = ControlServer(comms)
    rospy.loginfo("Arduino Pump and Gimbal Control_Server is Running")
    rospy.spin()

if __name__ == '__main__':
    main()
