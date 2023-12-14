#!/usr/bin/env python

import rospy
import actionlib
from uav_msgs.msg import SearchWPAction, SearchWPGoal, WayPtSearch
from sensor_msgs.msg import NavSatFix  # Add import statement for NavSatFix

def feedback_callback(feedback):
    # Print feedback during the execution
    print("Current Location:", feedback.current_location)

def send_search_wp_goal(status):
    # Initialize the ROS node
    rospy.init_node('search_wp_client')

    # Initialize the SearchWP action client
    search_wp_client = actionlib.SimpleActionClient('fcs_interface/fly_to_wp', SearchWPAction)

    # Wait for the action server to start
    search_wp_client.wait_for_server()

    # Create a SearchWPGoal message
    search_wp_goal = SearchWPGoal()

    # Fill in the goal
    search_wp_goal.goal = WayPtSearch()
    search_wp_goal.goal.status = status  # Set the desired status

    # Set the feedback callback function
    search_wp_client.feedback_cb = feedback_callback

    # Send the goal to the action server
    search_wp_client.send_goal(search_wp_goal)

    # Wait for the action to finish
    search_wp_client.wait_for_result()

    # Get the result of the action
    result = search_wp_client.get_result()

    # Print the final result
    print("Result:", result)

if __name__ == '__main__':
    # Replace this value with the desired status
    send_search_wp_goal(WayPtSearch.WSP1)  # Example: status = WSP1
