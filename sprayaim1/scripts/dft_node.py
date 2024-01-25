#!/usr/bin/env python
################################################################################
# ROS Node: dft_node
# This node simulates a fire target system in ROS. It publishes a custom message
# 'FireTarget' containing x, y, depth, and status information of the fire target.
# The node also provides dynamic reconfigure functionality to update the parameters
# of the fire target in runtime.
#
# Author: Nabil
# Change Date: 25-07-2023
#
# Dependencies:
#   1. Custom Message: FireTarget.msg
#      - The custom message file 'FireTarget.msg' defines the structure of the 
#        message containing the x, y, depth, and status information of the fire target.
#      - The message should be located in the 'sprayaim1/msg' directory.
#
#   2. Dynamic Reconfigure Configuration: FireTargetConfig.cfg
#      - The dynamic reconfigure configuration file 'FireTargetConfig.cfg' defines
#        the parameters that can be updated in runtime using dynamic reconfigure.
#      - The configuration file should be located in the 'sprayaim1/cfg' directory.
#
# Usage:
#   - To run the node: rosrun sprayaim1 dft_node.py 
#   - or with pump and gimal server Launch file roslaunch sprayaim1 spa_wpgs.launch
#   - To update the parameters of the fire target in runtime using rqt:
#       a. command "rosrun rqt_reconfigure rqt_reconfigure"
#       b. Select the 'dft_node' in the 'Node' dropdown menu
#       c. Update the parameters in the 'FireTarget' section
# Important : If GUI freeze then clodse (ctrl+c) the node and run command "rqt --clear-config" and close
# and run again rosrun rqt_reconfigure rqt_reconfigure
################################################################################
import rospy
from uav_msgs.msg import FireTarget  # Import the custom FireTarget message
from uav_msgs.msg import WallMetrics
from sprayaim1.cfg import FireTargetConfig  # Import the generated config message FireTargetConfig is not the file name its came from exit(gen.generate(PACKAGE, "sprayaim1", "FireTarget")) last argment Config is added to it.
from dynamic_reconfigure.server import Server
from std_msgs.msg import Header

fire_data = FireTarget()
wall_data= WallMetrics()
def reconfigure_callback(config, level):
    global fire_data, wall_data
    fire_data.Fire_x = config['Fire_x']  # Update the x-coordinate from the dynamic reconfigure parameter
    fire_data.Fire_y = config['Fire_y']  # Update the y-coordinate from the dynamic reconfigure parameter
    fire_data.Fire_depth = config['Fire_depth']  # Update the depth from the dynamic reconfigure parameter
    fire_data.status = config['status']  # Update the fire status from the dynamic reconfigure parameter
    wall_data.wall_distance_left = config['wall_distance_left']
    wall_data.wall_distance_center = config['wall_distance_center']
    wall_data.wall_distance_right = config['wall_distance_right']
  
    return config


def ft_node():
    rospy.init_node('dft_node', anonymous=False)  # Initialize the ROS node with the name 'dft_node' and make it non-anonymous
    pub_fire_data = rospy.Publisher('dft_node_topic', FireTarget, queue_size=10)  # Create a publisher for FireTarget messages on the 'dft_node_topic' topic
    pub_wall_data = rospy.Publisher('wall_metrics_topic', WallMetrics, queue_size=10) 
    # Create the dynamic reconfigure server with the FireTargetConfig and the reconfigure_callback function
    srv = Server(FireTargetConfig, reconfigure_callback)

    rate = rospy.Rate(1)  # Set the loop rate to 1 Hz

    while not rospy.is_shutdown():  # Main loop that runs until ROS shutdown
        # Populate header information
        header = Header()
        header.stamp = rospy.Time.now()  # Use current time
        header.frame_id = "dft"  # Set your desired frame ID

        # Assign the header to the FireTarget message
        fire_data.header = header
        wall_data.header = header
        # Publish the FireTarget message on the 'dft_node_topic' topic
        pub_fire_data.publish(fire_data)
        pub_wall_data.publish(wall_data)

        rate.sleep()  # Sleep to maintain the loop rate

if __name__ == '__main__':
    try:
        ft_node()  # Call the ft_node function to start the node
    except rospy.ROSInterruptException:
        pass
