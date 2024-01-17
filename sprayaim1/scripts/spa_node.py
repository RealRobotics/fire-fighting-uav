#!/usr/bin/env python
"""
ROS Node: spa_node with input accepted 

This node interacts with a water level monitoring system and controls a fire-fighting UAV's pump and gimbal.
It subscribes to various topics to receive information about water levels, pump commands, and fire target tracking.
The node updates the water level status, manages the pump's state (ON/OFF), and controls the gimbal's orientation
based on fire target tracking data.

Author: Nabil   
Last Changed Date: 15-08-2023 

Usage:
- To run the node: rosrun [Package name= sprayaim1] [Node name= spa_node.py]
"""
import rospy
import math
from std_msgs.msg import String, Header
from uav_msgs.msg  import FireTarget, iafiretarget
from pump_clientm import pump_control
from gimbal_clientm import gimbal_control_client
from uav_msgs.msg  import PumpStatus, iapumpstatus
from uav_msgs.msg  import WaterStatus, iawaterstatus
from uav_msgs.msg import SprayGoal, SprayResult, SprayFeedback,SprayAction
import actionlib

# Define the possible states of the state machine
STATE_OFF = 0
STATE_ON = 1

# Initialize the state to OFF
current_state = STATE_OFF
water_level_status = WaterStatus.WaterOK

pump_pub = rospy.Publisher('pump_status', String, queue_size=10)
gimbal_pub = rospy.Publisher('gimbal_status', String, queue_size=10)
iaft_pub = rospy.Publisher('ia_firetarget', iafiretarget, queue_size=10)
iafp_pub = rospy.Publisher('ia_pumpstatus', iapumpstatus, queue_size=10)
iawm_pub = rospy.Publisher('ia_waterstatus', iawaterstatus, queue_size=10)

def dwm_callback(data):
    """
    Callback function for receiving water level status.
    Updates the water level status based on received data.
    """
    global current_state, water_level_status

    # Extract information from the received message
    is_true = True  # accepted (True/False)
    sequence_id = data.header.seq
    received_time = data.header.stamp
    frame_id = data.header.frame_id
    water_status = data.water_status

    # Create and publish information to the 'iawaterstatus' topic
    iawaterstatus_msg = iawaterstatus()
    iawaterstatus_msg.is_true = is_true
    iawaterstatus_msg.sequence_id = sequence_id
    iawaterstatus_msg.received_time = received_time
    iawaterstatus_msg.frame_id = frame_id
    iawaterstatus_msg.water_status = water_status

    iawm_pub.publish(iawaterstatus_msg)

    # Update water_level_status based on received data
    if data.water_status ==  WaterStatus.WaterOK:
        water_level_status = WaterStatus.WaterOK
        rospy.loginfo("Water Level OK")  # Display water level status
    elif data.water_status == WaterStatus.WaterLow:
        water_level_status =  WaterStatus.WaterLow
        rospy.loginfo("Water Level Low")  # Display water level status

# def dfp_callback(data):
#     """
#     Callback function for receiving pump control commands.
#     Manages the pump's state based on received data and water level status.
#     Publishes pump status.
#     """
#     global current_state, water_level_status
#     # Extract information from the received message
#     is_true = True  # accepted (True/False)
#     sequence_id = data.header.seq
#     received_time = data.header.stamp
#     frame_id = data.header.frame_id
#     pump_status = data.pump_status

#     # Create and publish information to the 'iapumpstatus' topic
#     iapumpstatus_msg = iapumpstatus()
#     iapumpstatus_msg.is_true = is_true
#     iapumpstatus_msg.sequence_id = sequence_id
#     iapumpstatus_msg.received_time = received_time
#     iapumpstatus_msg.frame_id = frame_id
#     iapumpstatus_msg.pump_status = pump_status

#     iafp_pub.publish(iapumpstatus_msg)

#     # Turn ON/OFF pump based on received data and water level status
#     # If data indicates "on" and pump is OFF and water level is OK, turn ON the pump
#     if data.pump_status == PumpStatus.ON and current_state == STATE_OFF and water_level_status == WaterStatus.WaterOK:
#         current_state = STATE_ON
#         pump_control(True)  # Start the pump (True)
#         rospy.loginfo("Pump is ON.")
#     # If data indicates "off" and pump is ON or water level is Low, turn OFF the pump
#     elif data.pump_status == PumpStatus.OFF and (current_state == STATE_ON or water_level_status == WaterStatus.WaterLow):
#         current_state = STATE_OFF
#         pump_control(False)  # Start the pump (True)
#         rospy.loginfo("Pump is OFF.")
#     # Pump status output
#     if current_state == STATE_ON:
#         pump_pub.publish("on")  # Publish "on" status
#     elif current_state == STATE_OFF:
#         pump_pub.publish("off")  # Publish "off" status
        
def spray_server_Callback(goal):
    """
    Callback function for handling pump control goals.
    """
    global current_state, water_level_status

    # Extract goal information
    pump_goal_status = goal.pump_control.status

    # Create and publish information to the 'iapumpstatus' topic
    iapumpstatus_msg = iapumpstatus()
    iapumpstatus_msg.is_true = True  # accepted (True/False)
    iapumpstatus_msg.sequence_id = goal.header.seq
    iapumpstatus_msg.received_time = goal.header.stamp
    iapumpstatus_msg.frame_id = goal.header.frame_id
    iapumpstatus_msg.pump_status = pump_goal_status

    iafp_pub.publish(iapumpstatus_msg)

    # Turn ON/OFF pump based on received data and water level status
    # If data indicates "on" and pump is OFF and water level is OK, turn ON the pump
    if pump_goal_status == PumpStatus.ON and current_state == STATE_OFF and water_level_status == WaterStatus.WaterOK:
        current_state = STATE_ON
        result.done =pump_control(True)  # Start the pump (True)
        rospy.loginfo("Pump is ON.")
    # If data indicates "off" and pump is ON or water level is Low, turn OFF the pump
    elif pump_goal_status == PumpStatus.OFF and (current_state == STATE_ON or water_level_status == WaterStatus.WaterLow):
        current_state = STATE_OFF
        result.done=pump_control(False)  # Start the pump (True)
        rospy.loginfo("Pump is OFF.")
    # Pump status output
    if current_state == STATE_ON:
        pump_pub.publish("on")  # Publish "on" status
    elif current_state == STATE_OFF:
        pump_pub.publish("off")  # Publish "off" status

    # Simulate pump control result
    result = spray_server.Result()
    result.done
    # Send result to the action client
    if result.done and spray_server.is_active():
        spray_server.set_succeeded(result)

def dft_callback(data):
    """
    Callback function for receiving fire target tracking data.
    Controls the gimbal's orientation based on tracking data.
    Publishes gimbal status.
    """
    global current_state, water_level_status
    # Log messages based on data status
    rospy.loginfo("Received Status: %s", data.status)  # Debugging line

    # Create a new iafiretarget message with the received data and additional information
    custom_msg = iafiretarget()
    custom_msg.is_true = True
    custom_msg.sequence_id = data.header.seq
    custom_msg.received_time = data.header.stamp
    custom_msg.frame_id = data.header.frame_id
    custom_msg.x = data.Fire_x
    custom_msg.y = data.Fire_y
    custom_msg.depth= data.Fire_depth
    custom_msg.status = data.status

    # Publish the modified iafiretarget message
    iaft_pub.publish(custom_msg)

    # Log the published custom message (for debugging only)
    # rospy.loginfo("Input accepted message from iafiretarget: %s", custom_msg)

    if data.status == FireTarget.TRACKED:
        rospy.loginfo("Tracked")
        # constant_a = -65
        # constant_b = 0.1796875  # pixel_to_angle_ratio = 115/640
        # yaw_angle_degree = int(constant_a + data.Fire_x * constant_b)
        # pitch_angle_degree = int((data.Fire_depth - 4.5933) / 0.0403)

        # Calculate yaw and pitch
        yaw_angle_rad = math.atan2(data.Fire_y, data.Fire_x)
        pitch_angle_rad = -math.atan2(data.Fire_depth, math.sqrt(data.Fire_y**2 + data.Fire_y**2))

        # Convert angles to degrees
        yaw_angle_degree = math.degrees(yaw_angle_rad)
        pitch_angle_degree = math.degrees(pitch_angle_rad)

        if abs(yaw_angle_degree) > 90 or abs(pitch_angle_degree) > 90:
            gimbal_pub.publish("OOB")
            rospy.loginfo("Gimbal OOB")
        elif abs(yaw_angle_degree) <= 90 and abs(pitch_angle_degree) <= 90:
            gimbal_pub.publish("Tracking")
            rospy.loginfo("Gimbal Tracking")
            rospy.loginfo("Formula Yaw Angle: %f, Formula Pitch Angle: %f", yaw_angle_degree, pitch_angle_degree)  # Debugging line
            gimbal_control_client(yaw_angle_degree, pitch_angle_degree)
    elif data.status == FireTarget.NO_FIRE:
        rospy.loginfo("No Fire")
        gimbal_pub.publish("Idle")
    elif data.status == FireTarget.DETECTED:
        rospy.loginfo("Detected")
        gimbal_pub.publish("Idle")

def spa_node():
    rospy.init_node('spa_node', anonymous=False)

    # Subscribe to the topics of interest
    rospy.Subscriber('dwm_node_topic', WaterStatus, dwm_callback)
    #rospy.Subscriber('dfp_node_topic', PumpStatus, dfp_callback)
    rospy.Subscriber('dft_node_topic', FireTarget, dft_callback)
    # Create an action server for Spray
    global spray_server
    spray_server = actionlib.SimpleActionServer('spray_planner', SprayAction, execute_cb=spray_server_Callback, auto_start=False)
    spray_server.start()

    

    rospy.spin()  # Keep the node running

if __name__ == '__main__':
    try:
        spa_node()
    except rospy.ROSInterruptException:
        pass
