#!/usr/bin/env python
import rospy
import actionlib
from uav_msgs.msg import FlyToWPAction, FlyToWPGoal
import math

def calculate_intermediate_waypoints(start, end, num_intermediates):
    waypoints = []
    for i in range(num_intermediates + 1):
        ratio = float(i) / num_intermediates
        intermediate = (
            start[0] + ratio * (end[0] - start[0]),
            start[1] + ratio * (end[1] - start[1]),
            start[2] + ratio * (end[2] - start[2]),
            start[3] + ratio * (end[3] - start[3])
        )
        waypoints.append(intermediate)
    return waypoints

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

def send_multiple_waypoints(waypoints):
    for i in range(len(waypoints) - 1):
        current_waypoint = waypoints[i]
        next_waypoint = waypoints[i + 1]
        
        # Calculate intermediate waypoints between current and next
        intermediates = calculate_intermediate_waypoints(current_waypoint, next_waypoint, num_intermediates=5)

        # Send each intermediate waypoint
        for waypoint in intermediates:
            send_fly_to_waypoint_goal(*waypoint)

if __name__ == '__main__':
    # Define waypoints with small distance increments
    waypoints = [
        (53.8095, -1.5590, 20.0, 1.0),
        (53.8096, -1.5585, 21.0, 1.0),
        (53.8097, -1.5588, 22.0, 1.0),
        # Add more waypoints as needed
    ]

    # Send multiple waypoints with small distance increments
    send_multiple_waypoints(waypoints)
