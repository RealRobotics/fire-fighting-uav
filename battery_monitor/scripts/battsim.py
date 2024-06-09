#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from uav_msgs.msg import BatterySim

def publish_battery_state():
    rospy.init_node('battery_state_publisher', anonymous=False)
    pub = rospy.Publisher('/dji_sim/battery_state', BatterySim, queue_size=10)
    rate = rospy.Rate(0.5) # 0.5 Hz, i.e., every 2 seconds

    battery_percentage = 90.0
    last_update_time = rospy.Time.now()

    while not rospy.is_shutdown():
        # Creating BatterySim message
        battery_state_msg = BatterySim()
        battery_state_msg.header.seq = 24036
        battery_state_msg.header.stamp = rospy.Time.now()
        battery_state_msg.header.frame_id = ''
        battery_state_msg.voltage = 50416.0
        battery_state_msg.current = -556.0
        battery_state_msg.charge = float('nan')
        battery_state_msg.capacity = 11369.0
        battery_state_msg.design_capacity = float('nan')
        battery_state_msg.percentage = battery_percentage
        battery_state_msg.power_supply_status = 0
        battery_state_msg.power_supply_health = 0
        battery_state_msg.power_supply_technology = 0
        battery_state_msg.present = True
        battery_state_msg.cell_voltage = []
        battery_state_msg.location = ''
        battery_state_msg.serial_number = ''

        # Publishing message
        pub.publish(battery_state_msg)

        # Update battery percentage every two minutes
        current_time = rospy.Time.now()
        if (current_time.secs - last_update_time.secs) >= 100:
            battery_percentage -= 1
            last_update_time = current_time

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_battery_state()
    except rospy.ROSInterruptException:
        pass
