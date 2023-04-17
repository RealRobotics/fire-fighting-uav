""" TODO """

import rospy


class FireDetection(object):
    """ TODO """

    NODE_NAME = "fire_detection"

    def __init__(self):
        pass

    def start(self):
        """ TODO """
        rospy.logdebug("Starting " + self.NODE_NAME + "...")
        # Set up node, topics and services.
        rospy.init_node(self.NODE_NAME, log_level=rospy.DEBUG)
        rospy.on_shutdown(self.stop)
        rospy.loginfo(self.NODE_NAME + " ready.")
        rospy.spin()

    def stop(self):
        rospy.loginfo("Stopping " + self.NODE_NAME + "...")

