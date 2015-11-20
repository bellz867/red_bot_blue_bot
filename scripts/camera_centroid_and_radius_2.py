#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import time
from numpy import nonzero
import cv
from math import degrees
from math import atan2
from math import fabs
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import roslib
roslib.load_manifest('trying_again')

def nothing(x):
    pass

# Intializes everything and publishes stuff
def start():
    #print "entered start"
    # the command is guvcview
    # setting up the camera to video1 where the ps3eye camera is attached.
    cap = cv2.VideoCapture(1)
    cap.set(cv2.cv.CV_CAP_PROP_FPS, 60)
    # initializing color values
    # brightness
    b = 0
    # contrast
    c = 35
    # saturation
    s = 65
    # hue
    h = 90
    # gain
    g = 15
    # gaussian blur value
    blur_val = 10

    # initializing pink threshold values
    # upper threshold
    upper_p_hue = 179
    upper_p_sat = 255
    upper_p_val = 255
    # lower threshold
    lower_p_hue = 123
    lower_p_sat = 86
    lower_p_val = 0

    # upper threshold
    upper_p_hue2 = 0
    upper_p_sat2 = 0
    upper_p_val2 = 0
    # lower threshold
    lower_p_hue2 = 0
    lower_p_sat2 = 0
    lower_p_val2 = 0

    # calibration image color switch and mask switch
    cal_image_color = False
    cal_p_mask = False
    cal_p_mask2 = False
    cal_canny = False

    # define range for colors
    upper_p_thresh = np.array([upper_p_hue,upper_p_sat,upper_p_val])
    lower_p_thresh = np.array([lower_p_hue,lower_p_sat,lower_p_val])
    # define range for colors
    upper_p_thresh2 = np.array([upper_p_hue2,upper_p_sat2,upper_p_val2])
    lower_p_thresh2 = np.array([lower_p_hue2,lower_p_sat2,lower_p_val2])

    
    # kernel for the eroding and dialation routines
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))

    # center for the frame in pixels
    cx = 319.5
    cy = 239.5
    # focal lengths in pixels
    f = 5.5933430628532710e+02
    # actual diameter of the cylinder
    p_w = 9*25.4
    # actual height of the cylinder
    p_h = 9*25.4
    # moment values for a pink contour
    p_moment = 0
    # centroid values for a pink contour in pixels
    p_px = 0
    p_py = 0
    # area of a contour
    p_area = 0
    # values for the minimum enclosing rectangle
    p_fit_x = 0
    p_fit_y = 0
    p_fit_w = 0
    p_fit_h = 0
    p_fit_a = 0
    p_fit_center = 0
    # error for the pan angle between object center and center of frame
    p_thxe = 0
    # error for the tilt angle between object center and center of frame
    p_thye = 0
    # distance as seen from the width triangle
    p_zw = 0
    # distance as seen from the height triangle
    p_zh = 0
    # distance average
    p_z = 0
    # distance difference between the two
    p_z_diff = 0

    # time stamp values to get loop time
    t1 = 0
    t2 = 0
    time_diff = 0

     
    ### initializing the publishers for the two motors ###
    thx_pub = rospy.Publisher('pan_servo_error', Float32, queue_size=1)
    thy_pub = rospy.Publisher('tilt_servo_error', Float32, queue_size=1)
    z_pub = rospy.Publisher('distance', Float32, queue_size=1)
    area_pub = rospy.Publisher('area', Int32, queue_size=1)
    image_pub = rospy.Publisher('image_out', Image, queue_size=1)
    ### starting the node
    rospy.init_node('object_position')
    print "initialized nodes"
    
    bridge = CvBridge()
    
    r = rospy.Rate(45)

    while not rospy.is_shutdown():
        #print "entered loop"
        # reading in the image
        is_read, read = cap.read()
        #print is_read
        t1 = t2
        t2 = time.time()
        time_diff = t2- t1

        if is_read:
            

            # making the blurring value odd
            val = blur_val * 2 + 1
            
            # smoothing the read image using a low pass gaussian filter to remove noise
            image = cv2.GaussianBlur(read,(val,val),0)

            # Convert BGR image to a HSV image to enable easy thresholding for a specific shade
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # adjusting the threshold upper and lower values for the mask
            upper_p_thresh = np.array([upper_p_hue,upper_p_sat,upper_p_val])
            lower_p_thresh = np.array([lower_p_hue,lower_p_sat,lower_p_val])

            # adjusting the threshold upper and lower values for the second mask
            upper_p_thresh2 = np.array([upper_p_hue2,upper_p_sat2,upper_p_val2])
            lower_p_thresh2 = np.array([lower_p_hue2,lower_p_sat2,lower_p_val2])
            
            # masking the HSV image to get only certain shades of color using the threshold values
            mask_p_before = cv2.inRange(hsv, lower_p_thresh, upper_p_thresh)

            # masking the HSV image to get only certain shades of color using the second threshold values
            mask_p_before2 = cv2.inRange(hsv, lower_p_thresh2, upper_p_thresh2)

            # opening the image to filter out noise, opening is another name for eroding then dialating
            mask_p = cv2.morphologyEx(mask_p_before, cv2.MORPH_OPEN, kernel)

            # opening the image to filter out noise, opening is another name for eroding then dialating
            mask_p2 = cv2.morphologyEx(mask_p_before2, cv2.MORPH_OPEN, kernel)

            # or the masks together
            mask_f = cv2.bitwise_or(mask_p,mask_p2)

            # Bitwise-AND to get the masked image in color but with the pixels removed by the mask gone
            filtered_p_image = cv2.bitwise_and(image,image, mask = mask_p)

            # Bitwise-AND to get the masked image in color but with the pixels removed by the second mask gone
            filtered_p_image2 = cv2.bitwise_and(image,image, mask = mask_p2)

            # Bitwise-OR to combine the two masks
            filtered_p_imageout = cv2.bitwise_and(image,image,mask = mask_f)
        
            # trying to get the contours in the new image of only edges
            contours_p, hierarchy_p = cv2.findContours(mask_f,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)

            #getting the moments of the contours if the contour array has values
            if len(contours_p) > 0:
                #print "\nlength = %r" % len(contours_p)
                # creating a temporary array the length of the contours
                temp_areas_p = np.ones(len(contours_p))
                i = 0
                # getting all the areas
                while i < len(temp_areas_p):
                    # getting the area of the contour
                    area = cv2.contourArea(contours_p[i])
                    #print "\t area = %r" % area
                    temp_areas_p[i] = area
                    #print temp_areas_p[i]
                    #print i
                    i = i + 1
                i = 0
                # getting the maximum area that has an aspect ratio that is close to 1
                while i < len(temp_areas_p):
                    # checking if the current value is the max
                    if temp_areas_p[i] == max(temp_areas_p):
                        (t_x,t_y),(t_w,t_h),t_a = cv2.minAreaRect(contours_p[i])
                        if t_h > 0:
                            t_h = t_h
                        else:
                            t_h = 0.0001
                        t_aspect = t_w/t_h
                        #print t_aspect
                        # if the aspect the value was found to be max area will now check the aspect
                        # ratio to see if it is close to 1. If it is keep the value otherwise
                        # delete the value from the area and contour arrays then start over
                        if (t_aspect>0.75) and (t_aspect<1.25):
                            break
                        else:
                            contours_p.pop(i)
                            temp_areas_p = np.delete(temp_areas_p,i,None)                        
                            i = 0
                    else:
                        i = i + 1
                    if len(temp_areas_p) <= 0:
                        break

                if len(temp_areas_p) > 0:
                     # getting the bounding rectangle of the contour
                    #print i
                    pr_fit_x,pr_fit_y,pr_fit_w,pr_fit_h = cv2.boundingRect(contours_p[i])
                    
                    # getting the minimum enclosing rectangle 
                    # the return goes: center = (cx,cy), sides = (width,height), angle not sure
                    (p_fit_x,p_fit_y),(p_fit_w,p_fit_h),p_fit_a = cv2.minAreaRect(contours_p[i])
                    
                    p_cx = int(p_fit_x)
                    p_cy = int(p_fit_y)

                    # print the frames per second
                    print "\nfps = %r" % (1.0/time_diff)
                    #print "Pink Values"
                    print "\tmax area: %r" % temp_areas_p[i]
                    print "\t\tcx,cy %r,%r" % (p_cx,p_cy)
                    print "\t\t\tpixel width = %r \n\t\t\t\tpixel height = %r" % (p_fit_w,p_fit_h)
                    print "\t\t\t\t\tangle = %r" % p_fit_a
                    # getting the minimum enclosing circle
                    #(tx,ty),tr = cv2.minEnclosingCircle(contours_p[i])
                    # drawing the circle
                    #cv2.circle(image,(int(tx),int(ty)),int(tr),(0,0,255),2)
                    # adding the pink contours found to the image with a green circle and the green with pink
                    
                    # draw the minimum enclosing rectangle
                    cv2.rectangle(filtered_p_imageout,(pr_fit_x,pr_fit_y),(pr_fit_x+pr_fit_w,pr_fit_y+pr_fit_h),(0,0,255),2)
                    # drawing the minimum enclosing rectangle
                    #cv2.drawContours(image,rect,0,(0,0,255),2)
                    # draw the center of the minimum rectangle in blue
                    cv2.circle(filtered_p_imageout,(p_cx,p_cy),2,(255,0,0),2)
                    # updating the pan error angle error
                    thxe = -1.0*degrees(atan2((p_cx - cx),f))
                    # updating the tilt error angle error
                    thye = degrees(atan2((p_cy - cy),f))
                    print "thetax = %r\n\tthetay = %r" %(thxe,thye)
                    # updating the distance based on the width
                    p_zw = p_w*f/p_fit_w
                    # updating the distance based on the left value
                    p_zh = p_h*f/p_fit_h
                    # updating the average
                    p_z = (p_zw+p_zh)/2.0
                    # updating the differenct 
                    p_z_diff = p_zh - p_zw
                    # updating the area
                    p_area = temp_areas_p[i]
                    print "zw = %r\n\tzh = %r\n\t\tz = %r\n\t\t\tzdiff = %r" % (p_zw,p_zh,p_z,p_z_diff)
                # publishing the x angle error
                thx_pub.publish(thxe)
                # publishing the y angle error
                thy_pub.publish(thye)
                # publishing the z distance
                z_pub.publish(p_z)
                # publishing the area
                area_pub.publish(p_area)      
                # drawing in the center of the image a large white dot to show visual error
                cv2.circle(filtered_p_imageout,(319,239),2,(255,255,255),3)    
                # publishing the image
                try:
                    image_pub.publish(bridge.cv2_to_imgmsg(filtered_p_imageout,encoding="passthrough"))
                except CvBridgeError, e:
                    print e
        # displaying the three images
        #cv2.imshow('image',hsv)
        #cv2.imshow('regular image',image)
        #cv2.imshow('pink mask',filtered_p_image)
        #cv2.imshow('pink mask 2',filtered_p_image2)
        #cv2.imshow('pink mask out',filtered_p_imageout)
        #cv2.imshow('pink mask f',mask_f)


        # if cv2.waitKey(1) & 0xFF == 27:
        #     break
        # publishing the angles and distance

        # cv2.destroyAllWindows()
        r.sleep()

if __name__ == '__main__':
    start()
