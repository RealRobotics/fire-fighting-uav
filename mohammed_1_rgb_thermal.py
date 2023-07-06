import cv2 #imports
import pyrealsense2 as prs2 #import pyrealsense2
import numpy 
import math
import board,busio
import adafruit_mlx90640
from gpiozero import AngularServo
from time import sleep
import os     #importing os library so as to communicate with the system
import time   #importing time library to make Rpi wait because its too impatient 
os.system ("sudo pigpiod") #Launching GPIO library
#time.sleep(1) # As i said it is too impatient and so if this delay is removed you will get an error
import pigpio #importing GPIO library

armCheck=True
 
global fireLocationX,fireLocationY,enableStream, waterLocationX, waterLocationY #lets the program know that fireLocationX and fireLocationY are global coords so they can be called from the nozzle control program
enableStream=0 


Pipeline=prs2.pipeline() #the pipeline's purpose is to oversee the dataflow from the depth camera/bag file
Configuration=prs2.config() #produce configuration to allow the program to work with the intel realsense depth camera
Configuration.enable_stream(prs2.stream.color, 640, 480, prs2.format.bgr8, 15)
Configuration.enable_stream(prs2.stream.depth, 640, 480, prs2.format.z16, 15)
Pro=Pipeline.start(Configuration) #begins the pipeline
Object=prs2.align(prs2.stream.color) #ensures the RGB and depth data is synchronised

cv2.namedWindow("Combined_fire_detection",cv2.WINDOW_NORMAL)
cv2.resizeWindow("Combined_fire_detection", 1280,480)

yaw = AngularServo(17, min_angle=-90, max_angle=90) #17 is pin and 50 Hz pulse
pitch = AngularServo(18, min_angle=-90, max_angle=90) #18 is pin and 50 Hz
ESC=27  #Connect the ESC in this GPIO pin 

pitch.mid()
yaw.mid()
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
        #stop()


#thermal camera setup
i2c = busio.I2C(board.SCL, board.SDA, frequency=800000) # setup I2C
mlx = adafruit_mlx90640.MLX90640(i2c) # begin MLX90640 with I2C comm
print("MLX addr detected on I2C", [hex(i) for i in mlx.serial_number])
mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_4_HZ # set refresh rate
mlx_shape = (24,32)
frame =  [0] * 768

def nozzle(fireX, fireY, waterX, waterY):
	seperationX = fireX-waterX #calculates the change in x between the two coordinates
	seperationY = fireY-waterY #and the change in y
	#servoRequiredDistance=math.sqrt(pow(seperationX,2)+pow(seperationY,2)) #uses seperationX and seperationY, and pythagoras' theorem, to calculate the distance the servo motor needs to move
	servoRequiredDirection=math.atan2(seperationY,seperationX) #uses math.atan2 to calculate the required angle needed for the servo motor
	servoAngle = float(servoRequiredDirection)
	pitch.angle = servoAngle #moves the servo motor by the required distance, in the direction of the required direction
	time.sleep(2)

yaw.mid()
#pump on
#inp = input()

fireLocationX = 0
fireLocationY = 0
waterLocationX = 0
waterLocationY = 0
while True: # start loop 
    
    frames=Pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    framesFromRGB = numpy.asanyarray(color_frame.get_data())
    
    mlx.getFrame(frame)
    framesFromThermal=(numpy.reshape(frame,mlx_shape)) #obtain each frame from thermal source
    framesFromThermal=cv2.resize(framesFromThermal,(640,480)) #resize the frame to 800x400
    thermal_cam=numpy.uint8(framesFromThermal)
    coloured_thermal = cv2.cvtColor(thermal_cam,cv2.COLOR_GRAY2RGB)
    _,binary=cv2.threshold(thermal_cam,35, 255,cv2.THRESH_BINARY) #produce a binary frame from the greyscale frame.    
    

    maxAreaContour=None #variable for contour with  biggest area 
    maxAreaValue=0 #variable for area value of contour with biggest area.
    
    _, contourDetection, _=cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE) #detect contours within binary
    
    for singleContour in contourDetection: #loops through every contour in 'contourDetection'
        areaValue=cv2.contourArea(singleContour) #area of the contours being looped through
        if areaValue>maxAreaValue: #if a contour has a bigger area than the area held in 'maxAreaValue'...
            maxAreaValue=areaValue #...then this becomes the new 'maxAreaValue'
            maxAreaContour=singleContour # this marks this contour as the new 'maxAreaContour'.
            
    if maxAreaContour is not None: #if there is a value present in the variable
        cv2.drawContours(framesFromThermal,[maxAreaContour],0,(255,0,0),3) #this contour is displayed to the user
        moment=cv2.moments(maxAreaContour) #finds the center of 'maxAreaContour' using moments.
        if moment["m00"]!=0: #if the moment m00 is non-zero...
            central_x_thermal=int(moment["m10"]/moment["m00"]) #then the x coordinate of the center of the contour is obtained,
            central_y_thermal=int(moment["m01"]/moment["m00"]) #as well as the y coordinate

            
            #framesFromRGB=cv2.resize(framesFromRGB,(800,400), fx=0,fy=0, interpolation = cv2.INTER_CUBIC) #resize frame to 800x400 
            framesFromRGB=cv2.GaussianBlur(framesFromRGB,(3,3),0) # apply Gaussian Blur to reduce noise
            hsv_frame=cv2.cvtColor(framesFromRGB,cv2.COLOR_BGR2HSV) #RGB to HSV colour space conversion for the frame
            

            lowBounds=(0,30,30) #the lower and upper bounds of the HSV colours that the program is to look between, obtained iteratively through trial and error.
            highBounds=(10,255,255)
            videoMask=cv2.inRange(hsv_frame,lowBounds,highBounds)  #use lowBounds and highBounds to produce a mask of only the wanted colours
            erodeMask=cv2.erode(videoMask,None,iterations=1) #erosion and dilation to reduce noise and segregate elements
            dilateMask=cv2.dilate(erodeMask,None,iterations=1) 

            edgeDetection=cv2.Canny(dilateMask,100,200) #detect edges in the video
            _, contourDetection, _=cv2.findContours(edgeDetection,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE) #obtains contours from 'edgeDetection'. The use of 'cv2.RETR_EXTERNAL' means only external contours will be searched for, and the use of 'cv2.CHAIN_APPROX_SIMPLE' means every point on the contour is saved, not just the ends.
            maxAreaContour=None #variable for contour with biggest area
            maxAreaValue=0 #variable for area value of contour with biggest area
            
            for singleContour in contourDetection: #loops through every contour in 'contourDetection'
                areaValue=cv2.contourArea(singleContour) #area of the contours being looped through
                if areaValue>maxAreaValue: #if a contour has a bigger area than the area held in 'maxAreaValue'...
                    maxAreaValue=areaValue #...then this becomes the new 'maxAreaValue'
                    maxAreaContour=singleContour #this marks this contour as the new 'maxAreaContour'.
            
            if maxAreaContour is not None: #if there is a value present in the variable
                cv2.drawContours(framesFromRGB,[maxAreaContour],0,(255,0,0),3) #this contour is displayed to the user.
                moment=cv2.moments(maxAreaContour) #finds the center of the 'maxAreaContour' using moments.
                
                if moment["m00"]!=0: #if the moment m00 is non-zero...
                    central_x_rgb=int(moment["m10"]/moment["m00"]) #then the x coordinate of the center of the contour is obtained,
                    central_y_rgb=int(moment["m01"]/moment["m00"]) #as well as the y coordinate.
                    
                    seperation=math.sqrt(pow(central_x_thermal-central_x_rgb,2)+pow(central_y_thermal-central_y_rgb,2)) #find the seperation between the 2 contour centres. 
                    if seperation<100: #if this seperation is < than the set value (in this instance, 55)...
                        cv2.circle(framesFromRGB,(central_x_rgb,central_y_rgb),5,(255,0,255),3) # purple circle displayed at contour centre
                        fireLocationX=central_x_rgb #the global coordinates are updated, which can be called from the nozzle control program
                        fireLocationY=central_y_rgb
                        enableStream=1
                    else:
                        enableStream=0
    
    ####depth code beginning
    videoFramesDepth=Pipeline.wait_for_frames() #holds the loop until the following frame
    
    framesLineUpDepth=Object.process(videoFramesDepth) #lines up the RGB and depth frames using 'Object'
    framesFromDepth=framesLineUpDepth.get_depth_frame() # obtains depth frames 
    framesFromColourDepth=framesLineUpDepth.get_color_frame() #obtains RGB frames
    
    depthToNumpy=numpy.asanyarray(framesFromDepth.get_data()) #depth to numpy, required as the format of the depth and RGB information doesn't work with opencv, but numpy arrays do.
    colourToNumpyDepth=numpy.asanyarray(framesFromColourDepth.get_data()) #colour to numpy, for the same reason as 'depthToNumpy'
   
    depthInfo=numpy.asanyarray(framesFromDepth.get_data()) #obtain depth information
    depthInfo=depthInfo/1000 #mm to m
    proximityMaskDepth=numpy.where((depthInfo>0)&(depthInfo<=1),1,0).astype(numpy.uint8) #mask of the parts of the video that are within a meter of the camera.
    
    _, contourDetectionDepth, _ =cv2.findContours(proximityMaskDepth,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #contourDetection,unusedVariable=cv2.findContours(proximityMask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE) #use the mask to produce contour lines around parts of the video that are within a meter, i.e. the water stream.
    for singleContourDepth in contourDetectionDepth: #a loop to go through each of the contours
        cv2.drawContours(colourToNumpyDepth,[singleContourDepth],0,(255,0,0),3) #these contours are displayed. 'colourToNumpy' is the frame, 'singleContour' is the contour, the following bracket is for the contour colour (blue), and the final number is the contour thickness.
    
    if len(contourDetectionDepth)>0: #first ensures more than 0 contours have been detected to avoid an error.
        streamLandPoint=min(contourDetectionDepth,key=lambda x:cv2.boundingRect(x)[1]) #detects contour highest on the screen. 
        
        xVal,yVal,wVal,hVal=cv2.boundingRect(streamLandPoint) #calculates the bounding rectangle values of the contour highest on the screen.
        xValCentre=xVal+wVal//2 #x coord of the rectangle centre
        yValCentre=yVal+hVal//2 #y coord
        
        cv2.circle(colourToNumpyDepth,(xValCentre,yValCentre),5,(255,0,255),3) #circle displayed at location of contour highest on video
        waterLocationX=xValCentre #updates the global water stream coordinates
        waterLocationY=yValCentre
    
    ####depth code end

                        
    bothFrames = cv2.hconcat([framesFromRGB,coloured_thermal]) #combines the two frames together using hconcat   
    cv2.imshow("Combined_fire_detection", bothFrames) 
    cv2.imshow("water detection (depth)",colourToNumpyDepth) #show the result to the user  
    waitKey=cv2.waitKey(1) #how long the program will wait between each frame

    if armCheck==True:
        inp = input("Press Enter")    
        inp == arm()
        #time.sleep(7)
        armCheck=False
        #print(armCheck)

    nozzle(fireLocationX,fireLocationY,waterLocationX,waterLocationY)

    if waitKey==ord("x"): #if x is pressed by the user...
        cv2.destroyAllWindows() #the window is closed, and...
        break #the loop breaks






   
