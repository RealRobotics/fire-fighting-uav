#!/usr/bin/env python
################################################################################
# ROS Node: fire_detection_node
################################################################################
import rospy
from uav_msgs.msg import FireTarget  # Import the custom FireTarget message
from uav_msgs.msg import WallMetrics
from sprayaim1.cfg import FireTargetConfig  # Import the generated config message FireTargetConfig is not the file name its came from exit(gen.generate(PACKAGE, "sprayaim1", "FireTarget")) last argment Config is added to it.
from std_msgs.msg import Header

fire_data = FireTarget()
wall_data= WallMetrics()


def ft_node():
    rospy.init_node('fire_target_node', anonymous=False)  # Initialize the ROS node with the name 'dft_node' and make it non-anonymous
    pub_fire_data = rospy.Publisher('dft_node_topic', FireTarget, queue_size=10)  # Create a publisher for FireTarget messages on the 'dft_node_topic' topic
    pub_wall_data = rospy.Publisher('wall_metrics_topic', WallMetrics, queue_size=10) 

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
