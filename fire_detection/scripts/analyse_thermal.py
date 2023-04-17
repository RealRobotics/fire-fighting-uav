import cv2
import sys
import numpy as np

def nothing(x):
    pass

def temp_tune(frame_real,thermal):
    tuning = True
    
    while tuning:
        frame = frame_real.copy()

        temp = cv2.getTrackbarPos('temp','temp_mask')
        mask = cv2.inRange(thermal,temp,255)
        masked = cv2.bitwise_and(frame,frame,mask=mask)
        # detect_circles(frame,masked)
        cv2.imshow('temp_mask',masked)
        # otsu_thresholding(thermal)
        key = cv2.waitKey(10) & 0xFF
        if key == ord('q') or key == ord('Q'):
            tuning = False
    return cv2.getTrackbarPos('temp','temp_mask')
def otsu_thresholding(thermal):
    _,otsu = cv2.threshold(thermal,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    blur = cv2.GaussianBlur(thermal,(5,5),0)
    _,otsu2 = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    cv2.imshow('otsu',otsu)
    cv2.imshow('otsu2',otsu2)
def detect_circles(frame,masked):
    dp = cv2.getTrackbarPos('dp','temp_mask')/10.0
    minDist = cv2.getTrackbarPos('minDist','temp_mask')
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    
    circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,dp=dp,minDist=(minDist))#,
        # minRadius=int(frame.shape[0] / 10.0), maxRadius = int(frame.shape[0] / 3.0))
    if circles is not None:
        circles = np.round(circles[0,:]).astype('int')
        for (x,y,r) in circles:
            cv2.circle(masked,(x,y),r,(0,255,0),4)
    return circles

def find_hotspot(frame):
    pass
def circle_distance(circles):
    pass        

def mask_thermal_rgb(result_path):
    thermal = cv2.VideoCapture(result_path + '/thermal_raw.avi')
    rgb = cv2.VideoCapture(result_path + '/rgb_raw.avi')
    fire_temp = 70
    cv2.namedWindow('temp_mask')
    cv2.createTrackbar('temp','temp_mask',fire_temp,255,nothing)
    cv2.createTrackbar('dp','temp_mask',20,100,nothing)
    cv2.createTrackbar('minDist','temp_mask',20,200,nothing)
    
    while (thermal.isOpened() and rgb.isOpened()):
        _,thermal_frame = thermal.read()
        _,rgb_frame = rgb.read()
        
        if rgb_frame is None or thermal_frame is None:
            if rgb_frame is not None:
                print 'rgb is',rgb_frame.shape
            else:
                print 'rgb is none'

            if thermal_frame is not None:
                print 'thermal is',thermal_frame.shape
            else:
                print 'thermal is none'
            break
        thermal_frame = cv2.cvtColor(thermal_frame,cv2.COLOR_BGR2GRAY)
        thermal_frame = cv2.GaussianBlur(thermal_frame,(3,3),0)
        # otsu_thresholding(thermal_frame)
        (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(thermal_frame)
        print minVal,thermal_frame.mean().mean(),maxVal
        if maxVal < 200:
            cv2.setTrackbarPos('temp','temp_mask',int(maxVal * 0.5))
        
        
            fire_temp = cv2.getTrackbarPos('temp','temp_mask')
            
            mean_mask = cv2.inRange(thermal_frame,fire_temp,255)
            
            if int(cv2.__version__[0]) > 3:
                #opencv 4.x.x
                contours, hierarchy = cv2.findContours(mean_mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            else:
                #opencv 3.x.x
                _,contours,hierarchy = cv2.findContours(mean_mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            #skip the loop if no contours found
            if len(contours) == 0:
                continue
            cnt = None
            centre = None
            for c in contours:
                area = cv2.contourArea(c)
                frame_area = thermal_frame.shape[0] * thermal_frame.shape[1]
                if area > frame_area * 0.15:
                    break
                c_mask = np.zeros(thermal_frame.shape,np.uint8)
                cv2.drawContours(c_mask,[c],0,255,-1)
                cmin,cmax,cminloc,cmaxloc = cv2.minMaxLoc(thermal_frame,mask=c_mask)
                gradient_threshold = 10
                if (1000 < area < frame_area * 0.1) and \
                    cv2.pointPolygonTest(c,maxLoc,False) and \
                        (cmax - cmin) > gradient_threshold:

                    cnt = c
                    M = cv2.moments(c)
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centre = (cx,cy)

        
        cv2.imshow('thermal',thermal_frame)
        if centre is not None:
            cv2.drawContours(rgb_frame,[cnt],0,(255,0,0),5)
            cv2.circle(rgb_frame,centre,2,(0,0,255),-1)
        cv2.imshow('rgb',rgb_frame)
        # print mean_mask.shape,rgb_frame.shape
        masked = cv2.bitwise_and(rgb_frame,rgb_frame,mask=mean_mask)
        # detect_circles(rgb_frame,masked)
        cv2.imshow('temp_mask',masked)
        key = cv2.waitKey(10) & 0xFF
        if key == ord('q') or key == ord('Q'):
            break
        if key == ord('p') or key == ord('P'):
            fire_temp = temp_tune(rgb_frame,thermal_frame)
    cv2.destroyAllWindows()

mask_thermal_rgb(sys.argv[1])