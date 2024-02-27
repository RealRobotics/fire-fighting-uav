#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8
from uav_msgs.msg import PumpStatus, WaterStatus
from uav_msgs.srv import EnableWaterMonitor, EnableWaterMonitorResponse
from std_msgs.msg import Time

import threading

class WaterMonitor:
    def __init__(self, initial_counter_value=30):
        rospy.init_node('water_monitor_node', anonymous=False)
        rospy.loginfo('Water Monitor Node Running')
        self.counter_value = initial_counter_value
        self.intial_counter=initial_counter_value
        self.publisher_thread_started = False  # Flag to track whether the publisher thread has started
        self.pump_status = PumpStatus.OFF 
        self.pub_water_status = rospy.Publisher('water_monitor_node/water_status_topic', WaterStatus, queue_size=10)
        # publisher will start publishinh as soon as it intilaized.
        water_status = WaterStatus()
        water_status.header.stamp = rospy.Time.now()
        water_status.status = WaterStatus.WaterOK
        self.publish_water_status(water_status)
        
        # Create a separate thread for the publisher 
        self.publisher_thread = threading.Thread(target=self.publisher_thread_function, name='water_status_threard')
        self.publisher_thread.daemon = True  # The thread will exit when the main program exits
        self.publisher_thread.start()
        while not self.publisher_thread_started:
         rospy.loginfo("Thread not started wait")
         rospy.sleep(0.1)
          
        self.pub_service_water_monitor_request = rospy.Publisher('water_monitor_node/service_water_monitor_request_topic', Time , queue_size=10)
        self.pub_service_water_monitor_respond = rospy.Publisher('water_monitor_node/service_water_moniter_respond_topic', Time , queue_size=10)
        # service is initialized after the thread
        service = rospy.Service('EnableWaterMonitor', EnableWaterMonitor, self.handle_enable_water_monitor)

    def handle_enable_water_monitor(self, req):

        Inst=False # flag for service instrumentation
        previous_pump_status = self.pump_status  # Save previous pump status
        self.pump_status = req.status

        if self.pump_status != previous_pump_status:
            # Pump status changed
            rospy.loginfo("Pump status changed: Previous={}, New={}".format(previous_pump_status, self.pump_status))
            rospy.loginfo('Pump is ON from OFF. Down counter started.')
            current_time = rospy.Time.now()
            rospy.loginfo('Publishing current time for request: {}'.format(current_time))
            self.pub_service_water_monitor_request.publish(current_time)
            Inst=True

            if self.pump_status == PumpStatus.ON:
                if self.counter_value > 0:
                    # Start down counter
                    self.start_down_counter()
                else:
                    rospy.loginfo('Pump OFF to ON but water is empty.')
            elif self.pump_status == PumpStatus.OFF:
                # Pump is turned off
                rospy.loginfo('Pump is OFF from ON.')
            else:
                return EnableWaterMonitorResponse(False)  # Unknown pump status, return failure

        elif self.pump_status == PumpStatus.OFF and self.counter_value == self.intial_counter:
            rospy.loginfo('Pump OFF send by client and Water is full.')
        
        if Inst == True:
            current_time = rospy.Time.now()
            rospy.loginfo('Publishing current time for respond {}'.format(current_time))
            self.pub_service_water_monitor_respond.publish(current_time)
            Inst =False

        return EnableWaterMonitorResponse(True)  # Successful response
    
    def start_down_counter(self):
        # Start the down counter timer
        rospy.Timer(rospy.Duration(1), self.down_counter_callback, oneshot=False)

    def down_counter_callback(self, event):
        if self.pump_status == PumpStatus.ON and self.counter_value > 0:
            # If the pump is still ON, decrease the counter value
            self.counter_value -= 1
            rospy.loginfo('Counter Value: %s' % self.counter_value)

        elif self.pump_status == PumpStatus.OFF and self.counter_value > 0:
            # If the pump is still ON, decrease the counter value
            rospy.loginfo('Counter Value: %s' % self.counter_value)
            rospy.loginfo('Pump is OFF and Water is NOT Empty!,counter is greater than zero')

        elif self.counter_value <= 0:
            rospy.loginfo('Water is empty! counter is zero')
            rospy.loginfo('Counter Value: %s' % self.counter_value)


    def publish_water_status(self, water_status):
        # Publish water status message
        self.pub_water_status.publish(water_status)
    
    def publisher_thread_function(self):
        self.publisher_thread_started = True  # Set the flag to indicate that the thread has started
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            if self.counter_value > 0:
                water_status = WaterStatus()
                water_status.header.stamp = rospy.Time.now()
                water_status.status = WaterStatus.WaterOK
                rospy.loginfo('Publishing water status ok with timestamp: {}'.format(water_status.header.stamp))
                self.publish_water_status(water_status)
            elif self.counter_value <= 0:
                water_status = WaterStatus()
                water_status.header.stamp = rospy.Time.now()
                water_status.status = WaterStatus.WaterLow
                self.publish_water_status(water_status)
            rate.sleep()

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    water_monitor = WaterMonitor()
    water_monitor.run()
    
