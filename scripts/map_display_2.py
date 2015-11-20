#!/usr/bin/env python
# its called pinta
import rospy
import sys
import cv2
import cv2.cv as cv
import cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from numpy import nonzero
from math import degrees
from math import radians
from math import cos
from math import sin
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float32
import sys
import roslib
roslib.load_manifest('trying_again')

mapx = 0.0
mapy = 0.0
distance = 0.0
angle = 0.0
a_button_value = 0

def updateangle(angle_new):
    global angle
    # updating angle
    angle = angle_new.data

def updatemapvalues(distance_new):
    global distance
    # updating map
    distance = distance_new.data

# function: a_button_read
# purpose:
# 1) Read "a_button" topic
def a_button_read(data):
    global a_button_value
    a_button_value = data.data

def start():
    global mapx
    global mapy
    global distance
    global angle
    global a_button_value

    ### starting the node
    rospy.init_node('map_update')
    # Subscribe to the camera image and depth topics and set
    # the appropriate callbacks
    mapupdate_sub = rospy.Subscriber("distance", Float32, updatemapvalues, queue_size=1)
    panupdate_sub = rospy.Subscriber("pan_servo_current", Int16, updateangle, queue_size=1)
    a_button_sub = rospy.Subscriber("a_button", Int16, a_button_read,queue_size=1)
    # arrays for the distance and angle values average
    angle_mean = np.zeros((1,3))
    distance_mean = np.zeros((1,3))
    # kernel for the eroding and dialation routines
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
    # loop rate
    r = rospy.Rate(60)
    # index for the number of runs
    index_runs = 0
    # index for the multiple to clean up intermediately
    n = 1
    while not rospy.is_shutdown():
        
        # loading the frame
        frame = cv2.cvtColor(cv2.imread('/home/zack/v1_robotws/src/trying_again/map_21.jpg'),cv2.COLOR_BGR2GRAY)
        # if the number of runs has gone beyond some threshold then begin to do image processing to clena
        # up the map cleans it every minute
        if (index_runs > n*3600):
            print "cleaning"
            # applying a threshold to the image to remove noise and clean it up
            ret,thresh = cv2.threshold(frame,30,255,cv2.THRESH_BINARY)
            # closing to fill in gaps faster
            mask = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
            # applying the mask to the image
            # Bitwise-AND to get the masked image in color but with the pixels removed by the mask gone
            filtered_image = cv2.bitwise_and(frame,frame, mask = mask)
            # updating the frame to be the filtered image
            frame = filtered_image
            # updating the iteration
            n = n + 1
        print index_runs
        #writing dots to show locations on temporary image
        with_dots = cv2.cvtColor(frame,cv2.COLOR_GRAY2BGR)
        
        # getting the new averages
        for index_of in xrange(0,distance_mean.shape[1]-1):
            angle_mean[0,index_of] = angle_mean[0,index_of+1]
            distance_mean[0,index_of] = distance_mean[0,index_of+1]
        angle_mean[0,angle_mean.shape[1]-1] = angle
        distance_mean[0,distance_mean.shape[1]-1] = distance
        
        # each pixel is an inch
        mapx = np.mean(distance_mean,axis=1)/25.4 * cos(radians(float(145-np.mean(angle_mean,axis=1))))
        mapy = np.mean(distance_mean,axis=1)/25.4 * sin(radians(float(145-np.mean(angle_mean,axis=1))))
        
        # new depth value value
        mapx = int(mapx)
        mapy = int(mapy)

        if mapx>(frame.shape[1]-1):
            mapx = frame.shape[1]-1
        elif mapx<0:
            mapx = 0
        if mapy>(frame.shape[0]-1):
            mapy = frame.shape[0]-1
        elif mapy<0:
            mapy = 0

        # correcting the image to draw from the bottom right
        mapx = frame.shape[1]-1 - mapx
        mapy = frame.shape[0]-1 - mapy

        # print "new_depth:%r" % new_depth
        print "\nmapy,mapx: %r,\t%r" % (mapy,mapx)
        previous_depth = frame.item(mapy,mapx)
        # print "previous_depth: %r" % previous_depth
        new_depth = previous_depth + 25

        # if the new value is greater than 255 then set it to 255
        if new_depth >=255:
            new_depth = 255
        else:
            new_depth = new_depth
        # writing the values if a is pressed
        if (a_button_value > 0):
            frame.itemset((mapy,mapx),new_depth)
            print "\twrote cx,cy = %r,%r " % (mapx,mapy)        
            #updating the index
            index_runs = index_runs + 1

        # print "new frame value %r" % frame[mapx,mapy]
        cv2.circle(with_dots,(mapx,mapy),2,(255,0,0),-1)
        cv2.circle(with_dots,(frame.shape[1]-1,frame.shape[0]-1),6,(0,0,255),-1)
        # save image with dots
        cv2.imwrite('/home/zack/v1_robotws/src/trying_again/temp_map.jpg',with_dots)
        #Save image
        cv2.imwrite('/home/zack/v1_robotws/src/trying_again/map_21.jpg', frame)
        r.sleep()


if __name__ == '__main__':
    start()
   