#!/usr/bin/env python
"""
ROS Node: spa_node with input accepted

This node interacts with a water level monitoring system and controls a fire-fighting UAV's pump and gimbal.
It subscribes to various topics to receive information about water levels, pump commands, and fire target tracking.
The node updates the water level status, manages the pump's state (ON/OFF), and controls the gimbal's orientation
based on fire target tracking data.

Author: Nabil

Usage:
- To run the node: rosrun [Package name= sprayaim1] [Node name= spa_node.py]
"""
import rospy
import math
from std_msgs.msg import String, Header
from uav_msgs.msg import FireTarget, iafiretarget
from pump_clientm import pump_control
from gimbal_clientm import gimbal_control_client
from uav_msgs.msg import PumpStatus, iapumpstatus
from uav_msgs.msg import WaterStatus, iawaterstatus
from uav_msgs.msg import SprayGoal, SprayResult, SprayFeedback, SprayAction
from uav_msgs.msg import GimbalStatus
import actionlib

class SpaNode:
    def __init__(self):
        # Define the possible states of the state machine
        self.STATE_OFF = 0
        self.STATE_ON = 1

        # Initialize the state to OFF
        self.current_state = self.STATE_OFF
        self.water_level_status = WaterStatus.WaterOK
        # Create a timer to periodically publish status
        rospy.init_node('spa_node', anonymous=False)
        
        self.publish_pump_status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_pump_status_callback)

        self.pump_pub = rospy.Publisher('spa_pump_status_topic', String, queue_size=10)
        self.gimbal_pub = rospy.Publisher('spa_gimbal_status_topic', GimbalStatus, queue_size=10)
        self.iaft_pub = rospy.Publisher('ia_firetarget_topic', iafiretarget, queue_size=10)
        self.iafp_pub = rospy.Publisher('ia_pumpstatus_topic', iapumpstatus, queue_size=10)
        self.iawm_pub = rospy.Publisher('ia_waterstatus_topic', iawaterstatus, queue_size=10)

        # Subscribe to the topics of interest
        rospy.Subscriber('water_monitor_node/water_status_topic', WaterStatus, self.water_monitor_Callback)
        rospy.Subscriber('dft_node_topic', FireTarget, self.dft_callback)

        # Create an action server for Spray
        self.spray_server = actionlib.SimpleActionServer('spray_planner', SprayAction, execute_cb=self.spray_server_Callback, auto_start=False)
        self.spray_server.start()

        rospy.spin()  # Keep the node running

    def water_monitor_Callback(self, data):
        """
        Callback function for receiving water level status.
        Updates the water level status based on received data.
        """
        is_true = True
        sequence_id = data.header.seq
        received_time = data.header.stamp
        frame_id = data.header.frame_id
        water_status = data.status

        iawaterstatus_msg = iawaterstatus()
        iawaterstatus_msg.is_true = is_true
        iawaterstatus_msg.sequence_id = sequence_id
        iawaterstatus_msg.received_time = received_time
        iawaterstatus_msg.frame_id = frame_id
        iawaterstatus_msg.water_status = water_status

        self.iawm_pub.publish(iawaterstatus_msg)

        if water_status == WaterStatus.WaterOK:
            self.water_level_status = WaterStatus.WaterOK
            rospy.loginfo("Water Level OK")
        elif water_status == WaterStatus.WaterLow:
            self.water_level_status = WaterStatus.WaterLow
            rospy.loginfo("Water Level Low")

    def spray_server_Callback(self, goal):
        """
        Callback function for handling pump control goals from mission controller
        """
        rospy.loginfo("Received Goal:\n%s", goal)
        pump_goal_status = goal.pump_control.status

        rospy.loginfo("Received Goal:")
        rospy.loginfo("Pump Status: %s", pump_goal_status)

        iapumpstatus_msg = iapumpstatus()
        iapumpstatus_msg.is_true = True
        iapumpstatus_msg.sequence_id = goal.pump_control.header.seq
        iapumpstatus_msg.received_time = goal.pump_control.header.stamp
        iapumpstatus_msg.frame_id = goal.pump_control.header.frame_id
        iapumpstatus_msg.pump_status = pump_goal_status

        self.iafp_pub.publish(iapumpstatus_msg)

        result = SprayResult()

        if pump_goal_status == PumpStatus.ON and self.current_state == self.STATE_OFF and self.water_level_status == WaterStatus.WaterOK:
            result.done = pump_control(True)
            if result.done:
                rospy.loginfo("Pump is ON.")
                self.current_state = self.STATE_ON
            else:
                self.spray_server.set_aborted()

        elif pump_goal_status == PumpStatus.OFF and (self.current_state == self.STATE_ON or self.water_level_status == WaterStatus.WaterLow):
            result.done = pump_control(False)
            if result.done:
                rospy.loginfo("Pump is OFF.")
                self.current_state = self.STATE_OFF
            else:
                self.spray_server.set_aborted()

        if self.current_state == self.STATE_ON:
            self.pump_pub.publish("on")
            self.spray_server.set_succeeded(result)
        elif self.current_state == self.STATE_OFF:
            self.pump_pub.publish("off")
            self.spray_server.set_succeeded(result)

    def dft_callback(self, data):
        """
        Callback function for receiving fire target tracking data.
        Controls the gimbal's orientation based on tracking data.
        Publishes gimbal status.
        """
        rospy.loginfo("Received Status: %s", data.status)

        custom_msg = iafiretarget()
        custom_msg.is_true = True
        custom_msg.sequence_id = data.header.seq
        custom_msg.received_time = data.header.stamp
        custom_msg.frame_id = data.header.frame_id
        custom_msg.x = data.Fire_x
        custom_msg.y = data.Fire_y
        custom_msg.depth = data.Fire_depth
        custom_msg.status = data.status

        self.iaft_pub.publish(custom_msg)

        if data.status == FireTarget.TRACKED:
            rospy.loginfo("Tracked")
            yaw_angle_rad = math.atan2(data.Fire_y, data.Fire_x)
            pitch_angle_rad = -math.atan2(data.Fire_depth, math.sqrt(data.Fire_y**2 + data.Fire_y**2))

            yaw_angle_degree = math.degrees(yaw_angle_rad)
            pitch_angle_degree = math.degrees(pitch_angle_rad)

            if abs(yaw_angle_degree) > 90 or abs(pitch_angle_degree) > 90:
                self.gimbal_pub.publish(data.header,GimbalStatus.OUT_OF_BOUNDARY)
                rospy.loginfo("Gimbal OOB")
            elif abs(yaw_angle_degree) <= 90 and abs(pitch_angle_degree) <= 90:
                self.gimbal_pub.publish(data.header,GimbalStatus.TRACKING)
                rospy.loginfo("Gimbal Tracking")
                rospy.loginfo("Formula Yaw Angle: %f, Formula Pitch Angle: %f", yaw_angle_degree, pitch_angle_degree)
                gimbal_control_client(yaw_angle_degree, pitch_angle_degree)
        elif data.status == FireTarget.NO_FIRE:
            rospy.loginfo("No Fire")
            self.gimbal_pub.publish(data.header,GimbalStatus.IDLE)
        elif data.status == FireTarget.DETECTED:
            rospy.loginfo("Detected")
            self.gimbal_pub.publish(data.header,GimbalStatus.IDLE)
    
    def publish_pump_status_callback(self, event):
        if self.current_state == self.STATE_ON:
            self.pump_pub.publish("on")
        elif self.current_state == self.STATE_OFF:
            self.pump_pub.publish("off")

if __name__ == '__main__':
    try:
        spa_node = SpaNode()
    except rospy.ROSInterruptException:
        pass
