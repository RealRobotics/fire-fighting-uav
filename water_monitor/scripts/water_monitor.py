#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt8
from uav_msgs.msg import PumpStatus, WaterStatus,ServiceInstrumentation
from uav_msgs.srv import EnableWaterMonitor, EnableWaterMonitorResponse
from std_msgs.msg import Time

import threading

class WaterMonitor:
    def __init__(self, initial_counter_value=30):
        rospy.init_node('water_monitor_node', anonymous=False)
        rospy.loginfo('Water Monitor Node Running')
        self.counter_value = initial_counter_value
        self.intial_counter=initial_counter_value
        self.pump_status = PumpStatus.OFF 
        self.pub_water_status = rospy.Publisher('water_monitor_node/water_status_topic', WaterStatus, queue_size=10)
        # self.pub_service_instrumentation = rospy.Publisher('water_monitor_node/service_instrumentation_topic', ServiceInstrumentation , queue_size=10)
        
        self.pub_service_water_monitor_request = rospy.Publisher('water_monitor_node/service_water_monitor_request_topic', Time , queue_size=10)
        self.pub_service_water_monitor_respond = rospy.Publisher('water_monitor_node/service_water_moniter_respond_topic', Time , queue_size=10)

        # rospy.Subscriber('pump_status_topic', PumpStatus, self.pump_status_callback)
        service = rospy.Service('EnableWaterMonitor', EnableWaterMonitor, self.handle_enable_water_monitor)
        
        # Create a separate thread for the publisher 
        self.publisher_thread = threading.Thread(target=self.publisher_thread_function, name='water_status_threard')
        self.publisher_thread.daemon = True  # The thread will exit when the main program exits
        self.publisher_thread.start()

    def handle_enable_water_monitor(self, req):

        # service_status=ServiceInstrumentation()
        # service_status.header.stamp = rospy.Time.now()
        # service_status.status = ServiceInstrumentation.serviceXRequest
        # self.pub_service_instrumentation(service_status)
        current_time = rospy.Time.now()
        rospy.loginfo(f'Publishing current time for request: {current_time}')
        self.pub_service_water_monitor_request(current_time)

        previous_pump_status = self.pump_status  # Save previous pump status
        self.pump_status = req.status

        if self.pump_status != previous_pump_status:
            # Pump status changed
            rospy.loginfo("Pump status changed: Previous={}, New={}".format(previous_pump_status, self.pump_status))

            if self.pump_status == PumpStatus.ON:
                if self.counter_value > 0:
                    # Start down counter
                    self.start_down_counter()
                    rospy.loginfo('Pump is ON from OFF. Down counter started.')
                else:
                    rospy.loginfo('Pump OFF to ON but water is empty.')
            elif self.pump_status == PumpStatus.OFF:
                # Pump is turned off
                rospy.loginfo('Pump is OFF from ON.')
            else:
                current_time = rospy.Time.now()
                rospy.loginfo(f'Publishing current time for respond: {current_time}')
                self.pub_service_water_monitor_respond(current_time)
                return EnableWaterMonitorResponse(False)  # Unknown pump status, return failure

        elif self.pump_status == PumpStatus.OFF and self.counter_value == self.intial_counter:
            rospy.loginfo('Pump OFF send by client and Water is full.')

        current_time = rospy.Time.now()
        rospy.loginfo(f'Publishing current time for respond: {current_time}')
        self.pub_service_water_monitor_respond(current_time)
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
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            if self.counter_value > 0:
                water_status = WaterStatus()
                water_status.header.stamp = rospy.Time.now()
                water_status.status = WaterStatus.WaterOK
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
    