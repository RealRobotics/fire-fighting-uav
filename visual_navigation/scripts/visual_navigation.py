#!/usr/bin/env python

import rospy
import math
import actionlib
from uav_msgs.msg import FireTarget, WallMetrics, VisualNavigation, RelativePositionAction, RelativePositionGoal
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

class VisualNavigationNode:
    def __init__(self):

        # Initialize wall_metrics_msg to a default value (you can adjust this based on your message structure)
        self.wall_metrics_msg = WallMetrics()

        # Initialize prev_fire_status
        self.prev_fire_status = FireTarget.NO_FIRE

        # Initialize tracked_distance (4 meters)
        self.tracked_distance = 4.0

        # Initialize scan_distance (6 meters)
        self.scan_distance = 6.0


        rospy.init_node('visual_navigation_node', anonymous=False)

        # Subscribe to the "fire_target" topic with the custom message type
        rospy.Subscriber('dft_node_topic', FireTarget, self.fire_target_callback)

        # Subscribe to the "WallMetrics" topic with the custom message type
        rospy.Subscriber('wall_metrics_topic', WallMetrics, self.wall_metrics_callback)

        # Create a publisher for visual_status
        self.visual_status_pub = rospy.Publisher('visual_status_topic', VisualNavigation, queue_size=10)

        # Create an action client for RelativePosition
        self.relative_position_client = actionlib.SimpleActionClient('/fcs_interface/relative_position', RelativePositionAction)
        self.relative_position_client.wait_for_server()

        rospy.spin()

    def fire_target_callback(self, msg):

        wall_distance_left = self.wall_metrics_msg.wall_distance_left
        wall_distance_center = self.wall_metrics_msg.wall_distance_center
        wall_distance_right = self.wall_metrics_msg.wall_distance_right

        # Process the received custom message from "fire_target" topic
        header = msg.header
        fire_x = msg.Fire_x
        fire_y = msg.Fire_y
        fire_depth = msg.Fire_depth
        fire_status = msg.status

        # Transform the coordinates to ENU frame
        #enu_coordinates = self.transform_coordinates_to_enu(fire_x, fire_y, fire_depth)

        # Publish the VisualNavigation message
        visual_msg = VisualNavigation()
        visual_msg.header = header
        visual_msg.visual_x = fire_depth
        visual_msg.visual_y = fire_x  
        visual_msg.visual_z = fire_y 
        visual_msg.visual_yaw =  self.calculate_yaw_angle(wall_distance_left,wall_distance_center ,wall_distance_right)    
        visual_msg.status = self.calculate_visual_status(fire_x, fire_y, fire_depth, wall_distance_left, wall_distance_center, wall_distance_right)
        self.visual_status_pub.publish(visual_msg)

        # Check if fire status changes from DETECTED to NO_FIRE
        if self.prev_fire_status == FireTarget.DETECTED and fire_status == FireTarget.NO_FIRE:
            # If previously detected and now no fire, move to the scan distance
            self.send_relative_position_goal(visual_msg.visual_x-self.scan_distance , visual_msg.visual_y, visual_msg.visual_z, visual_msg.visual_yaw )
            rospy.loginfo(" From  Detected to No Fire Status and Send Goal to FCS")
        elif self.prev_fire_status == FireTarget.NO_FIRE and fire_status == FireTarget.DETECTED:
            # Move 4 meters from the wall
                self.send_relative_position_goal(visual_msg.visual_x-self.tracked_distance , visual_msg.visual_y, visual_msg.visual_z, visual_msg.visual_yaw )
                rospy.loginfo(" From No Fire to Detected Status")
        elif self.prev_fire_status == FireTarget.TRACKED and fire_status == FireTarget.NO_FIRE:
            # Check the distance from the wall if fire is detected
            rospy.loginfo(" From Tracked to No Fire Status")
        
       # Update the previous fire status
        self.prev_fire_status = fire_status

    def send_relative_position_goal(self, x, y, z, yaw):
        # Create a RelativePositionGoal with the ENU coordinates
        goal = RelativePositionGoal()

        # Assuming there is a Header field in RelativePositionGoal
        #goal.header.stamp = rospy.Time.now()
        goal.goal.header.stamp = rospy.Time.now()
        # Set the relative position attributes
        goal.goal.xv_relative = x
        goal.goal.yv_relative = y
        goal.goal.zv_relative = z
        goal.goal.yawv_relative = yaw

        # Send the goal to the relative_position_action server
        self.relative_position_client.send_goal(goal, done_cb=self.relative_position_goal_done_callback)
        rospy.loginfo("Sending Relative Position Goal: xv={}, yv={}, zv={}, yawv={}".format(
        goal.goal.xv_relative, goal.goal.yv_relative, goal.goal.zv_relative, goal.goal.yawv_relative))

    def relative_position_goal_done_callback(self, state, result):
        # This callback is called when the goal is completed
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("RelativePosition goal succeeded! Drone is at the target.")
        else:
            rospy.logwarn("RelativePosition goal did not succeed. State: {}".format(state))

    def wall_metrics_callback(self, msg):
        # Process the received custom message from "WallMetrics" topic
        self.wall_metrics_msg = msg  # Assign the received message to the attribute

        header = msg.header
        self.wall_distance_left = msg.wall_distance_left
        self.wall_distance_center = msg.wall_distance_center
        self.wall_distance_right = msg.wall_distance_right

        # Calculate the yaw angle from WallMetrics
        try:
            yaw_angle = self.calculate_yaw_angle(self.wall_distance_left,self.wall_distance_center ,self.wall_distance_right)
            rospy.loginfo("Calculated Yaw Angle: {}".format(yaw_angle))
        except Exception as e:
            rospy.logerr("Error calculating yaw angle or visual status: {}".format(str(e)))

        # Print the received information
        rospy.loginfo("Received WallMetrics message:")
        # rospy.loginfo("Header: {}".format(header))
        # rospy.loginfo("Wall_distance_left: {}".format(self.wall_distance_left))
        # rospy.loginfo("Wall_distance_center: {}".format(self.wall_distance_center))
        # rospy.loginfo("Wall_distance_right: {}".format(self.wall_distance_right))

    def calculate_yaw_angle(self, left_distance, wall_distance_center, right_distance):
        try:
            # Calculate distance ratios instead of difference
            ratio_left = (wall_distance_center - left_distance) / (wall_distance_center + left_distance)
            ratio_right = (right_distance - wall_distance_center) / (right_distance + wall_distance_center)

            # Calculate average ratio
            average_ratio = (ratio_left + ratio_right) / 2

            # Calculate yaw angle using arctangent
            yaw_angle = 0.5 * math.atan(average_ratio)

            # Convert the angle to degrees
            yaw_angle_degrees = math.degrees(yaw_angle)

            rospy.loginfo("Yaw Angle: {}".format(yaw_angle_degrees))

            return yaw_angle_degrees

        except ZeroDivisionError:
            rospy.logwarn("Cannot calculate yaw angle: Division by zero.")
        return 0.0

    def calculate_visual_status(self, x, y, z, left_distance,centre_distance ,right_distance):
        # Set thresholds for considering the drone as centered or not
        centering_threshold_xy = 0.01
        centering_threshold_z = 0.01
        wall_alignment_threshold = 0.01

        # Check if the drone is centered in x-y and close to the wall
        if (abs(x) < centering_threshold_xy and
                abs(y) < centering_threshold_xy and
                abs(z) < centering_threshold_z and
                abs(left_distance - right_distance) < wall_alignment_threshold):
            return VisualNavigation.CENTRED
        else:
            return VisualNavigation.NOTCENTRED

if __name__ == '__main__':
    try:
        VisualNavigationNode()
    except rospy.ROSInterruptException:
        pass
