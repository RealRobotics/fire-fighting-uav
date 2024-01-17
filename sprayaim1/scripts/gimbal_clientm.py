#!/usr/bin/env python
"""
This script is a ROS node that controls a gimbal using the gimbal_control action server.
It defines a yaw_pitch_callback to handle the action result and perform goal/result comparisons.

Author: Nabil
Last Changed Date: 08/08/2023

Description:
- This ROS node interacts with a gimbal_control action server to control the yaw and pitch angles of a gimbal.
- The script defines a callback function, yaw_pitch_callback, which is called when the action result is received.
- The yaw_pitch_callback function performs goal/result comparisons, logs received and achieved angles, and checks if the result matches the goal within a certain tolerance.

Functions:
1. yaw_pitch_callback(yaw, pitch, goal_yaw, goal_pitch):
    - Callback function to handle the result of the gimbal control action.
    - Compares the achieved result with the original goal and logs relevant information.
    - Calculates and logs the differences between the goal and achieved angles.
    - Checks if the result matches the goal within a defined tolerance.

2. gimbal_control_client(yaw_angle, pitch_angle):
    - Creates a SimpleActionClient for the 'gimbal_control' action server.
    - Sends a goal to the action server with the desired pitch and yaw angles.
    - Waits for the result and invokes the yaw_pitch_callback to handle the result.
    - Logs relevant information, including goal and achieved angles.

Usage:
- Use the gimbal_control_client function to set the desired yaw and pitch angles and interact with the gimbal action server.
"""
import rospy
import actionlib
from sprayaim1.msg import gimbalAction, gimbalGoal, gimbalResult
import threading
import time

# Create a mutex to protect shared resources
mutex = threading.Lock()

def yaw_pitch_callback(yaw, pitch, goal_yaw, goal_pitch):
    def callback(status, result):
        with mutex:
            if status == actionlib.GoalStatus.SUCCEEDED:
                # Log received goal angles
                rospy.loginfo("Received goal: Yaw = %d, Pitch = %d", goal_yaw, goal_pitch)

                # Log achieved result angles
                rospy.loginfo("Achieved result: Yaw = %d, Pitch = %d", result.yaw, result.pitch)

                # Calculate the difference between goal and result angles
                yaw_difference = abs(result.yaw - goal_yaw)
                pitch_difference = abs(result.pitch - goal_pitch)

                # Log the calculated differences
                rospy.loginfo("Yaw Difference: %f", yaw_difference)
                rospy.loginfo("Pitch Difference: %f", pitch_difference)

                if yaw_difference < 1e-3 and pitch_difference < 1e-3:
                    rospy.loginfo("Received result matches the goal!")
                else:
                    rospy.loginfo("Received result does not match the goal.")
            else:
                rospy.loginfo("Gimbal Action Failed")

    return callback

def gimbal_control_client(yaw_angle, pitch_angle):
    # Get the in time
    in_time = time.time()

    # Create a SimpleActionClient for the 'gimbal_control' action server
    client = actionlib.SimpleActionClient('gimbal_control', gimbalAction)
    client.wait_for_server()
    rospy.loginfo("Connected to gimbal_control action server")

    # Create a goal with desired pitch and yaw angles
    goal = gimbalGoal()
    goal.pitch = pitch_angle
    goal.yaw = yaw_angle

    # Send the goal with the specified callback
    client.send_goal(goal, done_cb=yaw_pitch_callback(yaw_angle, pitch_angle, goal.yaw, goal.pitch))
    rospy.loginfo("Goal sent: Yaw = %d, Pitch = %d", goal.yaw, goal.pitch)

    # Create a separate thread to wait for the result
    def wait_for_result():
        with mutex:
            client.wait_for_result()
            result = client.get_result()
            if result is not None:
                rospy.loginfo("Goal result received: Yaw = %d, Pitch = %d", result.yaw, result.pitch)
            else:
                rospy.loginfo("No result received.")

            # Get the out time
            out_time = time.time()

            # Calculate and print the execution time
            execution_time = out_time - in_time
            rospy.loginfo("Execution Time: %f seconds", execution_time)

    result_thread = threading.Thread(target=wait_for_result)
    result_thread.start()
    result_thread.join()  # Wait for the result thread to finish before continuing
