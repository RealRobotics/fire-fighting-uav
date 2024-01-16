import os
import time
import threading
from serial import Serial
import rospy

SERIAL_TIME_OUT_MS = 250
UPDATE_RATE_PER_S = 10.0
POLL_RATE_PER_S = 1.0
SERIAL_MAX_COMMAND_LENGTH = 10
STATUS_UPDATE_INTERVAL_S = 0.2
MAX_LITRES = 4

PITCH_MIN_DEGREES = -90
PITCH_MAX_DEGREES = 90
PITCH_OFFSET_DEGREES = 90
YAW_MIN_DEGREES = -90
YAW_MAX_DEGREES = 90
YAW_OFFSET_DEGREES = 90

class Communications:
    def __init__(self):
        self.serial_ = None
        self.readLoopThread_ = None
        self.arduino_ready_ = False
        self.gimbal_ = {
            'last_pitch': 0,
            'last_yaw': 0,
            'requested_pitch': 0,
            'requested_yaw': 0,
            'done_pitch': False,
            'done_yaw': False
        }
        self.pump_ = {
            'running': False,
            'litres_remaining': MAX_LITRES,
            'done': False
        }
        self.port = '/dev/ttyACM0'  # Hardcoded port
        self.baud_rate = 9600  # Hardcoded baud rate

    def __del__(self):
        if hasattr(self, 'serial_') and self.serial_:
            self.readLoopThread_.join()
            self.serial_.close()

    def init(self):
        try:
            timeout = SERIAL_TIME_OUT_MS / 1000
            self.serial_ = Serial(self.port, self.baud_rate, timeout=timeout,xonxoff=False)
            time.sleep(2)  # Wait for Arduino to initialize
            if self.serial_.is_open:
                self.serial_.flushInput()
                self.serial_.flushOutput()
                self.startReadLoop()
            else:
                rospy.loginfo("Failed to open serial port on {}".format(self.port))
        except Exception as e:
            if hasattr(self, 'serial_') and self.serial_:
                del self.serial_
            rospy.loginfo("Exception: {}".format(e))
            raise

    def getGimbal(self):
        return self.gimbal_['last_pitch'], self.gimbal_['last_yaw']

    def setGimbal(self, pitch, yaw):
        if self.gimbal_['last_pitch'] != pitch:
            if PITCH_MIN_DEGREES <= pitch <= PITCH_MAX_DEGREES:
                self.gimbal_['requested_pitch'] = pitch
                self.gimbal_['done_pitch'] = False
                server_pitch = pitch + PITCH_OFFSET_DEGREES
                command = "p" + str(server_pitch)
                self.write(command)
            else:
                rospy.loginfo("pitch {} must be in range {} to {} degrees.".format(pitch, PITCH_MIN_DEGREES, PITCH_MAX_DEGREES))
        if self.gimbal_['last_yaw'] != yaw:
            if YAW_MIN_DEGREES <= yaw <= YAW_MAX_DEGREES:
                self.gimbal_['requested_yaw'] = yaw
                self.gimbal_['done_yaw'] = False
                server_yaw = yaw + YAW_OFFSET_DEGREES
                command = "y" + str(server_yaw)
                self.write(command)
            else:
                rospy.loginfo("yaw {} must be in range {} to {} degrees.".format(yaw, YAW_MIN_DEGREES, YAW_MAX_DEGREES))

    def getPump(self):
        return self.pump_['running'], self.pump_['litres_remaining']

    def setPump(self, running):
        if self.pump_['running'] != running:
            command = "f" if running else "s"
            self.write(command)

    def write(self, command):
        if hasattr(self, 'serial_') and self.serial_ and self.serial_.is_open:
            #rospy.loginfo("Sending '{}\\n'".format(command))
            command_with_new_line = command + '\n'
            self.serial_.write(command_with_new_line.encode())
            rospy.loginfo("Sent command: {}".format(command_with_new_line))

    def startReadLoop(self):
        self.readLoopThread_ = threading.Thread(target=self.readLoop)
        self.readLoopThread_.daemon = True
        self.readLoopThread_.start()
        self.waitForArduino()

    def waitForArduino(self):
        rospy.loginfo("In Wait for Arduino function call...")
        rate_s = 1 / POLL_RATE_PER_S
        while True:
            self.write("q")
            time.sleep(rate_s)
            if self.arduino_ready_:
                rospy.loginfo("Arduino ready.")
                break
            else:
                rospy.loginfo("Still waiting for Arduino...")

    def updateLitresRemaining(self, reply):
        try:
            value = float(reply[1:])
            self.pump_['litres_remaining'] = value
        except ValueError as e:
            rospy.loginfo("Invalid argument: {}".format(e))

    def readLoop(self):
         while not rospy.is_shutdown():
            try:
                if self.serial_.in_waiting > 0:
                    rospy.loginfo("Reading from Arduino serial...")
                    reply = self.serial_.readline(SERIAL_MAX_COMMAND_LENGTH).decode().strip()
                    if reply:
                        if len(reply) >= 2:
                            action_char, done_char = reply[0], reply[1]
                            rospy.loginfo("Processing reply '{}', action {}, done {}".format(reply, action_char, done_char))
                            if action_char == 'q':
                                pass
                            elif action_char == 'r':
                                self.arduino_ready_ = True
                                self.updateLitresRemaining(reply)
                            elif action_char == 's':
                                self.pump_['done'] = True
                                self.pump_['running'] = False
                                self.updateLitresRemaining(reply)
                            elif action_char == 'f':
                                self.pump_['running'] = True
                            elif action_char == 'y':
                                if done_char == 'd':
                                    self.gimbal_['last_yaw'] = self.gimbal_['requested_yaw']
                                    self.gimbal_['done_yaw'] = True
                                else:
                                    change = self.gimbal_['requested_yaw'] - self.gimbal_['last_yaw']
                                    self.gimbal_['last_yaw'] += change // 2
                            elif action_char == 'p':
                                if done_char == 'd':
                                    self.gimbal_['last_pitch'] = self.gimbal_['requested_pitch']
                                    self.gimbal_['done_pitch'] = True
                                else:
                                    change = self.gimbal_['requested_pitch'] - self.gimbal_['last_pitch']
                                    self.gimbal_['last_pitch'] += change // 2
                            else:
                                rospy.loginfo("Unhandled action '{}'".format(action_char))
                        else:
                            rospy.loginfo("Received reply '{}' is too short. Expected at least 2 characters.".format(reply))
                    else:
                        rospy.loginfo("No reply received from Arduino.")
                else:
                    #rospy.loginfo("No data available in the receive buffer.")
                    time.sleep(0.2)  # Wait 
            except Exception as e:
                rospy.logerr("Error while reading from Arduino: {}".format(e))