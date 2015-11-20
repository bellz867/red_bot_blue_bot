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
from math import cos
from math import sin
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float32
import sys
import roslib
roslib.load_manifest('trying_again')



class MapDisplay():
    def __init__(self):
        self.node_name = "map_viewer"
        
        rospy.init_node(self.node_name)
        # map variables
        self.mapx = 0
        self.mapy = 0
        self.distance = 0
        self.angle = 0
        self.frame = 0


        # Subscribe to the camera image and depth topics and set
        # the appropriate callbacks
        self.mapupdate_sub = rospy.Subscriber("distance", Float32, self.updatemapvalues, queue_size=1)
        self.mapupdate_sub = rospy.Subscriber("pan_servo_desired", Int16, self.updateangle, queue_size=1)

    def updateangle(self, angle_new):
        # updating angle
        print "theta before: %r " % self.angle
        self.angle = angle_new
        print "\ttheta after: %r" % self.angle

    def updatemapvalues(self, distance_new):
        # updating map
        print "x,y before: %r,%r" % (self.mapx,self.mapy)
        self.distance = distance_new
        self.mapx = self.distance * cos(radians(self.angle))
        self.mapy = self.distance * sin(radians(self.angle))
        print "\tx,y after: %r,%r" % (self.mapx,self.mapy)

    def updatemap(self):
        # new depth value value
        new_depth = self.frame[self.mapx,self.mapy] + 5
        # if the new value is greater than 255 then set it to 255
        if new_depth >=255:
            new_depth = 255
        else:
            new_depth = new_depth
        self.frame[self.mapx,self.mapy] = new_depth

        #Save image
        cv2.imwrite('map_1.jpg', self.gray)
        print "saved"

def main(args):       
    try:
        map1 = MapDisplay()
        map1.frame = 
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."

if __name__ == '__main__':
    main(sys.argv)
   