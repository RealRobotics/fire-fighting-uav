#importing libraries
import cv2 #imports
import pyrealsense2 as prs2 #import pyrealsense2
import numpy as np
import math
import time,board,busio
import adafruit_mlx90640
from gpiozero import AngularServo, Device
from gpiozero.pins.pigpio import PiGPIOFactory
Device.pin_factory = PiGPIOFactory()
import os     #importing os library so as to communicate with the system
os.system ("sudo pigpiod") #Launching GPIO library
time.sleep(1) # As i said it is too impatient and so if this delay is removed you will get an error
import pigpio #importing GPIO library
from simple_pid import PID

Pipeline=prs2.pipeline() #the pipeline's purpose is to oversee the dataflow from the depth camera/bag file
Configuration=prs2.config() #produce configuration to allow the program to work with the intel realsense depth camera
Configuration.enable_stream(prs2.stream.color, 640, 480, prs2.format.bgr8, 15)
Configuration.enable_stream(prs2.stream.depth, 640, 480, prs2.format.z16, 15)
Object=prs2.align(prs2.stream.color) #ensures the RGB and depth data is synchronised
Pro=Pipeline.start(Configuration) #begins the pipeline

pitch = AngularServo(17, min_angle=-90, max_angle=90)
yaw = AngularServo(18, min_angle = -90, max_angle=90)
ESC=27  #Connect the ESC in this GPIO pin
pi = pigpio.pi();

def set_water_nozzle(water_nozzle):
    if water_nozzle:
        pi.set_servo_pulsewidth(ESC, 2000)
    else:
        pi.set_servo_pulsewidth(ESC, 0)

#thermal camera setup
i2c = busio.I2C(board.SCL, board.SDA, frequency=800000) # setup I2C
mlx = adafruit_mlx90640.MLX90640(i2c) # begin MLX90640 with I2C comm
print("MLX addr detected on I2C", [hex(i) for i in mlx.serial_number])
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_4_HZ # set refresh rate
mlx_shape = (24,32)
frame =  [0] * 768 
frames=Pipeline.wait_for_frames()
color_frame = frames.get_color_frame()
cv2.imwrite("background.jpeg", np.asanyarray(color_frame.get_data()))


# Make a list of images
image_list = []
depth_list = []
zero_list = [0,0]
water_nozzle = False
set_water_nozzle(water_nozzle)

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
output_video_r = cv2.VideoWriter('output_video_r.mp4', fourcc, 30.0, (640, 480))
output_video_t = cv2.VideoWriter('output_video_t.mp4', fourcc, 30.0, (640, 480))
output_video_y = cv2.VideoWriter('output_video_y.mp4', fourcc, 30.0, (640, 480))

#looping through the video frames
while True:
    np.asanyarray(color_frame.get_data())
    frames=Pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    depth_frame = frames.get_depth_frame()
    rgb_frame = np.asanyarray(color_frame.get_data())
    
    mlx.getFrame(frame)
    framesFromThermal=(np.reshape(frame,mlx_shape)) #obtain each frame from thermal source
    framesFromThermal=cv2.resize(framesFromThermal,(640,480)) #resize the frame to 800x400
    thermal_cam=np.uint8(framesFromThermal)    

    cv2.imwrite("rgb_frame.jpeg", rgb_frame)

    # Make a copy of the rgb frame to draw contours on
    result = rgb_frame.copy()
    rgb_frame_copy = rgb_frame.copy()

    ##FIRE DETECTION##
    #THERMAL#
    # Apply binary thresholding to find the fire
    _, thresh_fire = cv2.threshold(thermal_cam, 70, 255, cv2.THRESH_BINARY)

    # Find contours of the thermal fire location
    _, thermo_contours, _ = cv2.findContours(thresh_fire, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the largest contour and only display that
    sorted_contours = sorted(thermo_contours, key=cv2.contourArea, reverse=True)
  
    # Draw the contours on the thermal frame
    if len(sorted_contours) > 0 and cv2.contourArea(sorted_contours[0]) > 100:
        cv2.drawContours(thermal_cam, sorted_contours[0], -1, (0, 255, 0), 3)
        M = cv2.moments(sorted_contours[0])
        # Thermal location of fire
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cv2.circle(thermal_cam, (cx,cy), 5, (0, 0, 255), -1)
    else:
        cx = 640
        cy = 480
    #THERMAL END#

    #RGB#
    # Define the range of colours to detect
    lower_fire = np.array([0, 60, 130])
    upper_fire = np.array([75, 255, 255])

    hsv = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2HSV)

    # Create a mask for the fire
    mask = cv2.inRange(hsv, lower_fire, upper_fire)
    
    #find the contour of the rgb fire location
    _, rgb_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # Draw the contours on the rgb frame
    cv2.drawContours(rgb_frame, rgb_contours, -1, (0, 255, 0), 3)
    #RGB END#

    #SEE IF RGB and THERMAL agree#
    # if rgb_contours are within a 50 pixel radius of the thermal_contours, show the thermal_contours
    # Calculates the centre of each rgb_contour
    for rgb_contour in rgb_contours:
        M = cv2.moments(rgb_contour)
        if M["m00"] != 0:
            tx = int(M['m10']/M['m00'])
            ty = int(M['m01']/M['m00'])
        else :
            # May cause an error
            tx = 0
            ty = 0

        distance = np.sqrt((cx - tx)**2 + (cy - ty)**2)

        if distance <= 300 and len(sorted_contours) >0:
            print ("Turning water nozzle ON")
            water_nozzle = True # turn water nozzle is on with arduino pin, may need to initiliase as false
            set_water_nozzle(water_nozzle)
            time.sleep(10)            
            cv2.drawContours(result, sorted_contours[0], -1, (0, 255, 0), 3)
            cv2.circle(result, (cx,cy), 5, (0, 0, 255), -1)
            fx = cx
            fy = cy
            dist_fire = depth_frame.get_distance(fx, fy)
            cv2.circle(result, (tx,ty), 5, (255, 0, 0), -1)
        else:
            water_nozzle = False
            set_water_nozzle(water_nozzle)
            break
    #END OF SEE IF RGB and THERMAL agree#
    #OUTPUTS fx, fy which is the fire location#
    ##END OF FIRE DETECTION##
    
    ##WATER STREAM DETECTION##
     #convert the rgb frame to lab
    rgb_lab = cv2.cvtColor(rgb_frame_copy, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(rgb_lab)
    l = cv2.add(l, -100)
    l = np.clip(l, 0, 255)
    clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(8,8))
    cl = clahe.apply(l)
    limg = cv2.merge((cl,a,b))
    enhanced_rgb_image = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    gray = cv2.cvtColor(enhanced_rgb_image, cv2.COLOR_BGR2GRAY)

    #Read background image
    background = cv2.imread("background.jpeg")

    # enhance background image
    background_lab = cv2.cvtColor(background, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(background_lab)
    l = cv2.add(l, -100)
    l = np.clip(l, 0, 255)
    clahe = cv2.createCLAHE(clipLimit=1.0, tileGridSize=(8,8))
    cl = clahe.apply(l)
    limg = cv2.merge((cl,a,b))
    enhanced_background_image = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    background_gray = cv2.cvtColor(background, cv2.COLOR_BGR2GRAY)

    # adjust brightness of background
    background_gray = cv2.add(background_gray, -30)
    background_gray = np.clip(background_gray, 0, 255)

    # Compute the absolute difference between the current frame and the background
    difference_image = cv2.absdiff(background_gray, gray)
    time.sleep(1)

    ##WATER STREAM PICUTRE OUTPUT, EXTRACT WHITE BITS##

    # detects water stream if water nozzle is on, if so runs
    if water_nozzle == True:
        print('Detecting water stream')
        background_mean = np.mean(image_list[20:40], axis=0)
        cv2.imwrite("background.jpeg", background_mean)
        # find contours
        _, thresh = cv2.threshold(difference_image, 60, 255, cv2.THRESH_BINARY) #best value needs to be found. 60 does not detect all of floor image
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # if the contour area is greater than 3000, draw a circle on the top point and draw the contour of the water stream
        water_sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
        time.sleep(1)
        if cv2.contourArea(water_sorted_contours[0]) > 3000:
            contour = water_sorted_contours[0]
            y_coordinates = [point[0][1] for point in contour]
            top_point = contour[y_coordinates.index(min(y_coordinates))][0]
            cv2.drawContours(result, [contour], -1, (0, 255, 0), 3)

            ### NEEDS TO BE CALIBRATED ### DISTANCE MATTERS, CONSIDER DEPTH CAMERA
            # Pixel to angle ratio yaw
            pixel_to_angle_ratio = 115/640


            const = -65

            # Pixel to angle ratio pitch
            #pixel_to_angle_ratio_pitch = 480/100
            
            # Target coordinates - NEED TO BE DEFINED BY FIRE
            target_x = fx
            target_y = fy

            # Initialize the PID controllers for x and y directions
            pid_x = PID(Kp=1.1, Ki=0, Kd=0.005, setpoint=target_x)
            
            # define x and y
            x = top_point[0]
            
            # Calculate the PID output for x and y directions
            pid_output_x = pid_x(x) 
            
            # Update the coordinates of the red dot - THIS NEEDS TO UPDATE THE WATER NOZZLE POSITION
            x += pid_output_x
            
            # Ensure that the red dot remains within the dimensions of the image
            x = max(0, min(x, 639))
            
            # Convert the pixel coordinates to angles
            yaw_output_angle = const+(x*pixel_to_angle_ratio)
            pitch_output_angle = (dist_fire - 4.5933)/0.0403

            # # Send the angle value to the Arduino board as a PWM signal
            # WRITE TO ARDUINO
            print("adjusting nozzle")
            yaw.angle = input (yaw_output_angle)
            pitch.angle = input (pitch_output_angle) 

            # Draw the red dot and the blue dot on the image
            cv2.circle(result, (int(x),int(y)), 1, (0, 0, 255), 5)
            print(top_point)

            # Draw the blue dot for the target point
            cv2.circle(result, (target_x, target_y), 1, (255, 0, 0), 5)

            # if x == target_x and y == target_y:
            #     consecutive_frames += 1
            # else:
            #     consecutive_frames = 0
    else:
        image_list.insert(0, enhanced_rgb_image)
        if len(image_list) > 50:
            image_list.pop()

    output_video_r.write(rgb_frame)
    output_video_t.write(thermal_cam)
    output_video_y.write(result)


    # cv2.imshow('Enhanced Thermal', enhanced_thermal_frame)
    cv2.imshow('Result', result)
    cv2.imshow('Thermal',thermal_cam)
    cv2.imshow('Rgb',rgb_frame)
    
    #output_video.write(result)

    #exiting with pressing 'q'
    if cv2.waitKey(24) & 0xFF == ord('q'):
        break

#releasing and destroying windows
#rgb_video.release()
output_video_r.release()
output_video_t.release()
output_video_y.release()
cv2.destroyAllWindows()
