import math #imports
import RPi.GPIO as GPIO
from rgb_thermal_depth import waterLocationX
from rgb_thermal_depth import waterLocationY
from rgb_thermal_depth import fireLocationX
from rgb_thermal_depth import fireLocationY
from gpiozero import Servo
from time import sleep
import os     #importing os library so as to communicate with the system
import time   #importing time library to make Rpi wait because its too impatient 
os.system ("sudo pigpiod") #Launching GPIO library
time.sleep(1) # As i said it is too impatient and so if this delay is removed you will get an error
import pigpio #importing GPIO library
print("ready")

waterLocation=(rgb_thermal_depth.waterLocationX,rgb_thermal_depth.waterLocationY) #import the coordinates of the water stream from the water detection program, storing them as coordinates
fireLocation=(rgb_thermal_depth.fireLocationX,rgb_thermal_depth.fireLocationY) #do the same for the fire coordinates

print("recieved coordinates")
print("opening nozzle program")
yaw = AngularServo(17, min_angle=-90, max_angle=90) #17 is pin and 50 Hz pulse
pitch = AngularServo(18, min_angle=-90, max_angle=90) #18 is pin and 50 Hz
ESC=27  #Connect the ESC in this GPIO pin 

pitch.mid()
yaw.min()
pi = pigpio.pi();

def stop(): #This will stop every action your Pi is performing for ESC ofcourse.
    pi.set_servo_pulsewidth(ESC, 0)
    pi.stop()
 
def arm(): #This is the arming procedure of an ESC 
    print ("Connect the battery and press Enter")
    inp = input()    
    if inp == '':
        pi.set_servo_pulsewidth(ESC, 0)
        time.sleep(1)
        pi.set_servo_pulsewidth(ESC, 2000)
        time.sleep(1)
        stop()

while True: #begins the loop
    #if combined_fire_detection.enableStream==1: #checks if the 'enableStream' variable from the fire detection program is set to 1 (a fire is detected)
    time.sleep(2)
    yaw.max()
    #pump on
    #inp = input()
    inp = input()    
    if inp == "arm":
        arm()
        
        seperationX=fireLocation[0]-waterLocation[0] #calculates the change in x between the two coordinates
        seperationY=fireLocation[1]-waterLocation[1] #and the change in y
    
    	#servoRequiredDistance=math.sqrt(pow(seperationX,2)+pow(seperationY,2)) #uses seperationX and seperationY, and pythagoras' theorem, to calculate the distance the servo motor needs to move
        servoRequiredDirection=math.atan2(seperationY,seperationX) #uses math.atan2 to calculate the required angle needed for the servo motor
        servoAngle = float(servoRequiredDirection)
       
        pitch.angle = servoAngle #moves the servo motor by the required distance, in the direction of the required direction
        time.sleep(2)
    else:
        pi.stop() #the water stream is disabled (A0 pin set to low)



    


