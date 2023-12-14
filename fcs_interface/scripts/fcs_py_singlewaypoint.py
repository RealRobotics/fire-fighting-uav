#!/usr/bin/env python

import rospy
import actionlib
from uav_msgs.msg import FlyToWPAction, FlyToWPGoal

def feedback_callback(feedback):
    # Print feedback during the execution
    print("Current Location:", feedback.current_location)

def send_fly_to_waypoint_goal(latitude, longitude, altitude, loc_precision):
    # Initialize the ROS node
    rospy.init_node('fly_to_waypoint_client')

    # Initialize the FlyToWP action client
    fly_to_wp_client = actionlib.SimpleActionClient('fcs_interface/fly_to_wp', FlyToWPAction)

    # Wait for the action server to start
    fly_to_wp_client.wait_for_server()

    # Create a FlyToWP goal
    goal = FlyToWPGoal()

    # Set the waypoint location
    goal.goal.location.latitude = latitude
    goal.goal.location.longitude = longitude
    goal.goal.location.altitude = altitude

    # Set the location precision
    goal.goal.loc_precision = loc_precision

    # Set the feedback callback function
    fly_to_wp_client.feedback_cb = feedback_callback

    # Send the goal to the action server
    fly_to_wp_client.send_goal(goal)

    # Wait for the action to finish
    fly_to_wp_client.wait_for_result()

    # Get the result of the action
    result = fly_to_wp_client.get_result()

    # Print the final result
    print("Result:", result)

if __name__ == '__main__':
    # Replace these values with your desired coordinates and precision
    send_fly_to_waypoint_goal(53.8095, -1.5590, 20.0, 1.0)
