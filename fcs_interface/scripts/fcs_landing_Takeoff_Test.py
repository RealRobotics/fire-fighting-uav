#!/usr/bin/env python

import rospy
import actionlib
from uav_msgs.msg import SpecialMovement
from uav_msgs.msg import SpecialMovementAction, SpecialMovementGoal

def feedback_callback(feedback):
    rospy.loginfo("Feedback: {}".format(feedback))
    # Add your logic here based on the feedback if needed

def result_callback(status, result):
    rospy.loginfo("Goal Status: {}".format(status))
    rospy.loginfo("Result: {}".format(result))

def send_special_movement_goal(movement_type):
    # Initialize the ROS node
    rospy.init_node('drone_special_movement_client')

    # Create a SimpleActionClient for the SpecialMovement action
    client = actionlib.SimpleActionClient('/fcs_interface/special_movement', SpecialMovementAction)

    # Wait for the action server to start
    client.wait_for_server()

    # Create a SpecialMovementGoal message
    goal = SpecialMovementGoal()
    goal.movement.data = movement_type

    # Send the goal to the action server
    client.send_goal(goal, feedback_cb=feedback_callback)

    # Use a Rate object to check for feedback at regular intervals
    rate = rospy.Rate(1)  # 1 Hz
    while not rospy.is_shutdown() and not client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rate.sleep()

    # Get the result after the goal is completed
    result = client.get_result()
    rospy.loginfo("Final Result: {}".format(result))

if __name__ == '__main__':
    try:
        # Take off
        send_special_movement_goal(SpecialMovement.TAKE_OFF)

        # Wait for 20 seconds
        rospy.sleep(20.0)

        # Land
        send_special_movement_goal(SpecialMovement.LAND)

    except rospy.ROSInterruptException:
        pass
