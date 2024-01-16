#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8
from uav_msgs.msg import PumpStatus, WaterStatus

class WaterMonitor:
    def __init__(self, initial_counter_value=30):
        rospy.init_node('water_monitor_node', anonymous=False)
        self.counter_value = initial_counter_value
        self.intial_counter=initial_counter_value
        self.pump_status = PumpStatus.OFF 
        self.pub_water_status = rospy.Publisher('water_status_topic', WaterStatus, queue_size=10)
        rospy.Subscriber('pump_status_topic', PumpStatus, self.pump_status_callback)

    def pump_status_callback(self, msg):
        previous_pump_status = self.pump_status  # Save previous pump status
        self.pump_status = msg.status

        if self.pump_status != previous_pump_status:
            # If the pump status changes, take appropriate actions
            if self.pump_status == PumpStatus.ON and self.counter_value > 0:
                # If the pump is turned on, start the down counter
                self.start_down_counter()
                rospy.loginfo('Pump is ON from OFF. Down counter started.')
            elif self.pump_status == PumpStatus.ON and self.counter_value  <= 0:
                rospy.loginfo('Pump OFF to ON but water empty.')
            elif self.pump_status == PumpStatus.OFF:
                # If the pump is turned off, stop the down counter
                rospy.loginfo('Pump is OFF from ON .')
        elif self.pump_status == PumpStatus.OFF and self.counter_value == self.intial_counter:
            # Publish an initial water status message indicating "full"
            initial_water_status = WaterStatus()
            initial_water_status.header.stamp = rospy.Time.now()
            initial_water_status.status = WaterStatus.WaterOK
            self.publish_water_status(initial_water_status)
            rospy.loginfo('Intial condition water is full.')

        # Print the current pump status and counter value
        # rospy.loginfo('Pump Status: %s' % ('ON' if self.pump_status == PumpStatus.ON else 'OFF'))
        # rospy.loginfo('Counter Value: %s' % self.counter_value)

    def start_down_counter(self):
        # Start the down counter timer
        rospy.Timer(rospy.Duration(1), self.down_counter_callback, oneshot=False)

    def down_counter_callback(self, event):
        if self.pump_status == PumpStatus.ON and self.counter_value > 0:
            # If the pump is still ON, decrease the counter value
            self.counter_value -= 1
            rospy.loginfo('Counter Value: %s' % self.counter_value)
            water_status = WaterStatus()
            water_status.header.stamp = rospy.Time.now()
            water_status.status = WaterStatus.WaterOK
            self.publish_water_status(water_status)

        elif self.pump_status == PumpStatus.OFF and self.counter_value > 0:
            # If the pump is still ON, decrease the counter value
            rospy.loginfo('Counter Value: %s' % self.counter_value)
            water_status = WaterStatus()
            water_status.header.stamp = rospy.Time.now()
            water_status.status = WaterStatus.WaterOK
            self.publish_water_status(water_status)
            rospy.loginfo('Pump is OFF and Water is NOT Empty!,counter is greater than zero')

        elif self.counter_value <= 0:
            # If the counter reaches zero or less, publish water empty status
            water_status = WaterStatus()
            water_status.header.stamp = rospy.Time.now()
            water_status.status = WaterStatus.WaterLow
            self.publish_water_status(water_status)
            rospy.loginfo('Water is empty! counter is zero')
            rospy.loginfo('Counter Value: %s' % self.counter_value)


    def publish_water_status(self, water_status):
        # Publish water status message
        self.pub_water_status.publish(water_status)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    water_monitor = WaterMonitor()
    water_monitor.run()