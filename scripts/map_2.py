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

def updateangle(angle_new):
    global angle
    # updating angle
    angle = angle_new.data

def updatemapvalues(distance_new):
    global distance

    # updating map
    distance = distance_new.data


def start():
    global mapx
    global mapy
    global distance
    global angle

    ### starting the node
    rospy.init_node('map_update')
    # Subscribe to the camera image and depth topics and set
    # the appropriate callbacks
    mapupdate_sub = rospy.Subscriber("distance", Float32, updatemapvalues, queue_size=1)
    panupdate_sub = rospy.Subscriber("pan_servo_current", Int16, updateangle, queue_size=1)
    
    # arrays for the distance and angle values average
    angle_mean = np.ones((1,4))
    distance_mean = np.ones((1,4))

    r = rospy.Rate(20)

    while not rospy.is_shutdown():
         # loading the frame
        frame = cv2.cvtColor(cv2.imread('/home/zack/v1_robotws/src/trying_again/map_8.jpg'),cv2.COLOR_BGR2GRAY)
        # getting the new averages
        for index_of in xrange(0,len(distance_mean)-2):
            angle_mean[index_of] = angle_mean[index_of+1]
            distance_mean[index_of] = distance_mean[index_of+1]
        angle_mean[len(angle_mean)-1] = angle
        distance_mean[len(distance_mean)-1] = distance

        # each pixel is an inch
        mapx = np.mean(distance_mean)/25.4 * cos(radians(float(145-np.mean(angle_mean))))
        mapy = np.mean(distance_mean)/25.4 * sin(radians(float(145-np.mean(angle_mean))))
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

        # print "new_depth:%r" % new_depth
        print "\nmapy,mapx: %r,\t%r" % (mapy,mapx)
        previous_depth = frame.item(mapy,mapx)
        # print "previous_depth: %r" % previous_depth
        new_depth = previous_depth + 100

        # if the new value is greater than 255 then set it to 255
        if new_depth >=255:
            new_depth = 255
        else:
            new_depth = new_depth

        frame.itemset((mapy,mapx),new_depth)
        # print "new frame value %r" % frame[mapx,mapy]
        #Save image
        cv2.imwrite('/home/zack/v1_robotws/src/trying_again/map_8.jpg', frame)

if __name__ == '__main__':
    start()
   