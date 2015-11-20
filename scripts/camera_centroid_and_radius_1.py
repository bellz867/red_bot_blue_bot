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
    c = 50
    # saturation
    s = 65
    # hue
    h = 115
    # gain
    g = 14
    # gaussian blur value
    blur_val = 10

    # initializing pink threshold values
    # upper threshold
    upper_p_hue = 179
    upper_p_sat = 96
    upper_p_val = 255
    # lower threshold
    lower_p_hue = 74
    lower_p_sat = 0
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

    #cv.NamedWindow('image',cv.CV_WINDOW_NORMAL)
    # cv2.namedWindow('pink mask')
    # cv2.namedWindow('pink mask 2')
    #cv2.namedWindow('regular image')

    #print "created windows"

    # # create trackbars for color changes
    # cv2.createTrackbar('Brightness','image',b,255,nothing)
    # cv2.createTrackbar('Contrast','image',c,255,nothing)
    # cv2.createTrackbar('Saturation','image',s,255,nothing)
    # cv2.createTrackbar('Hue','image',h,179,nothing)
    # cv2.createTrackbar('Gain','image',g,63,nothing)
    # cv2.createTrackbar('Blur','image',blur_val,50,nothing)

    # print "first trackbars"

    # # trackbars for the pink threshold
    # cv2.createTrackbar('Upper Hue','pink mask',upper_p_hue,179,nothing)
    # cv2.createTrackbar('Upper Saturation','pink mask',upper_p_sat,255,nothing)
    # cv2.createTrackbar('Upper Value','pink mask',upper_p_val,255,nothing)
    # cv2.createTrackbar('Lower Hue','pink mask',lower_p_hue,179,nothing)
    # cv2.createTrackbar('Lower Saturation','pink mask',lower_p_sat,255,nothing)
    # cv2.createTrackbar('Lower Value','pink mask',lower_p_val,255,nothing)

    # print "second trackbars"

    # # trackbars for the pink threshold
    # cv2.createTrackbar('Upper Hue','pink mask 2',upper_p_hue2,179,nothing)
    # cv2.createTrackbar('Upper Saturation','pink mask 2',upper_p_sat2,255,nothing)
    # cv2.createTrackbar('Upper Value','pink mask 2',upper_p_val2,255,nothing)
    # cv2.createTrackbar('Lower Hue','pink mask 2',lower_p_hue2,179,nothing)
    # cv2.createTrackbar('Lower Saturation','pink mask 2',lower_p_sat2,255,nothing)
    # cv2.createTrackbar('Lower Value','pink mask 2',lower_p_val2,255,nothing)

    # # create switches to turn the trackbars on or off
    # switch = 'OFF : 0\nON : 1'
    # cv2.createTrackbar(switch,'image',0,1,nothing)
    # cv2.createTrackbar(switch,'pink mask',0,1,nothing)
    # cv2.createTrackbar(switch,'pink mask 2',0,1,nothing)

    #print "created trackbars"

    # kernel for the eroding and dialation routines
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))

    # center for the frame in pixels
    cx = 319.5
    cy = 239.5
    # focal lengths in pixels
    f = 774.037
    # actual diameter of the cylinder
    p_w = 5.5*25.4
    # actual height of the cylinder
    p_h = 10*25.4
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

    mtx = np.array([[7.7403693935378794e+02, 0., 3.1950000000000000e+02],[0., 7.7403693935378794e+02, 2.3950000000000000e+02],[0., 0., 1.]])
    dist = np.array([-1.8366538970082824e-01, 1.3831285723155446e+00, 0., 0., -6.0521719798787288e+00])
    
    ### initializing the publishers for the two motors ###
    thx_pub = rospy.Publisher('pan_servo_error', Float32, queue_size=1)
    thy_pub = rospy.Publisher('tilt_servo_error', Float32, queue_size=1)
    z_pub = rospy.Publisher('distance', Float32, queue_size=1)
    area_pub = rospy.Publisher('area', Int32, queue_size=1)
    image_pub = rospy.Publisher('image', Image, queue_size=10)
    ### starting the node
    rospy.init_node('object_position')
    print "initialized nodes"
    
    bridge = CvBridge()
    
    r = rospy.Rate(60)

    while not rospy.is_shutdown():
        #print "entered loop"
        # reading in the image
        is_read, read = cap.read()
        #print is_read
        t1 = t2
        t2 = time.time()
        time_diff = t2- t1

        if is_read:
            #print is_read
            # undistorting the image
            # dst = cv2.undistort(read, mtx, dist)

            # # checking if the image switch is at one allowing the image values to be adjusted
            # cal_image_color = (cv2.getTrackbarPos(switch,'image') == 1)
            # if cal_image_color:
            #     # get current positions of the trackbars
            #     b = (1.0*cv2.getTrackbarPos('Brightness','image'))/256.0
            #     c = (1.0*cv2.getTrackbarPos('Contrast','image'))/256.0
            #     s = (1.0*cv2.getTrackbarPos('Saturation','image'))/256.0
            #     h = (1.0*cv2.getTrackbarPos('Hue','image'))/180.0
            #     g = (1.0*cv2.getTrackbarPos('Gain','image'))/64.0
            #     blur_val = cv2.getTrackbarPos('Blur','image')
            #     # image brightness
            #     cap.set(cv2.cv.CV_CAP_PROP_BRIGHTNESS, b)
            #     # image contrast
            #     cap.set(cv2.cv.CV_CAP_PROP_CONTRAST, c)
            #     # image saturation
            #     cap.set(cv2.cv.CV_CAP_PROP_SATURATION, s)
            #     # image hue
            #     cap.set(cv2.cv.CV_CAP_PROP_HUE, h)
            #     # setting the gain 
            #     cap.set(cv2.cv.CV_CAP_PROP_GAIN, g)

            # # checking if the mask switch is at one allowing for the mask values to be adjusted
            # cal_p_mask = (cv2.getTrackbarPos(switch,'pink mask') == 1)
            # if cal_p_mask:
            #     # get current positions of the trackbars
            #     upper_p_hue = cv2.getTrackbarPos('Upper Hue','pink mask')
            #     upper_p_sat = cv2.getTrackbarPos('Upper Saturation','pink mask')
            #     upper_p_val = cv2.getTrackbarPos('Upper Value','pink mask')
            #     lower_p_hue = cv2.getTrackbarPos('Lower Hue','pink mask')
            #     lower_p_sat = cv2.getTrackbarPos('Lower Saturation','pink mask')
            #     lower_p_val = cv2.getTrackbarPos('Lower Value','pink mask')

            # # checking if the mask switch is at one allowing for the mask values to be adjusted
            # cal_p_mask2 = (cv2.getTrackbarPos(switch,'pink mask 2') == 1)
            # if cal_p_mask2:
            #     # get current positions of the trackbars
            #     upper_p_hue2 = cv2.getTrackbarPos('Upper Hue','pink mask 2')
            #     upper_p_sat2 = cv2.getTrackbarPos('Upper Saturation','pink mask 2')
            #     upper_p_val2 = cv2.getTrackbarPos('Upper Value','pink mask 2')
            #     lower_p_hue2 = cv2.getTrackbarPos('Lower Hue','pink mask 2')
            #     lower_p_sat2 = cv2.getTrackbarPos('Lower Saturation','pink mask 2')
            #     lower_p_val2 = cv2.getTrackbarPos('Lower Value','pink mask 2')

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
                # getting the maximum area
                while i < len(temp_areas_p):
                    if temp_areas_p[i] == max(temp_areas_p):
                        break
                    else:
                        i = i + 1
                # getting the bounding rectangle of the contour
                #print i
                p_fit_x,p_fit_y,p_fit_w,p_fit_h = cv2.boundingRect(contours_p[i])
                
                # getting the minimum enclosing rectangle 
                # the return goes: center = (cx,cy), sides = (width,height), angle not sure
                rect = cv2.minAreaRect(contours_p[i])
                # line across top
                #cv2.line(image,)
                # # getting the 
                # box = cv2.boxPoints(rect)
                # box = np.int0(box)
                
                print rect
                p_cx = int(p_fit_x+p_fit_w/2.0)
                p_cy = int(p_fit_y+p_fit_h/2.0)

                # print the frames per second
                print "\nfps = %r" % (1.0/time_diff)
                #print "Pink Values"
                p_area = int(temp_areas_p[i])
                print "max area: %r" % p_area
                print "cx,cy %r,%r" % (p_cx,p_cy)
                print "pixel width = %r \n\tpixel height = %r" % (p_fit_w,p_fit_h)
                # adding the pink contours found to the image with a green circle and the green with pink
                cv2.drawContours(image, contours_p[i], -1, (0,255,0), 2)
                # draw the minimum enclosing rectangle
                cv2.rectangle(image,(p_fit_x,p_fit_y),(p_fit_x+p_fit_w,p_fit_y+p_fit_h),(0,0,255),2)
                # drawing the minimum enclosing rectangle
                #cv2.drawContours(image,rect,0,(0,0,255),2)
                # draw the center of the minimum rectangle in blue
                cv2.circle(image,(p_cx,p_cy),2,(255,0,0),2)
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
        cv2.circle(image,(319,239),2,(255,255,255),3)    
        # publishing the image
        image_pub.publish(bridge.cv2_to_imgmsg(image,"bgr8"))
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
