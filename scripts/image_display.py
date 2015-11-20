#!/usr/bin/env python
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
from math import atan2
from math import fabs
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float32
import sys
import roslib
roslib.load_manifest('trying_again')

t1 = 0
t2 = 0

class ImageDisplay():
    def __init__(self):
        self.node_name = "image_viewer"
        
        rospy.init_node(self.node_name)
              
        # Create the cv_bridge object
        self.bridge = CvBridge()
        
        # Subscribe to the camera image and depth topics and set
        # the appropriate callbacks
        self.image_sub = rospy.Subscriber("image_out", Image, self.image_callback, queue_size=1)

    def image_callback(self, ros_image):
        global t1
        global t2
        # print "image callback"
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
            # print frame
        except CvBridgeError, e:
            print e
        #Save image
        cv2.imwrite('/home/zack/v1_robotws/src/trying_again/camera_1.jpg', frame)
        # print "saved"
        # frames per second
        t1 = t2
        t2 = time.time()
        time_diff = t2- t1
        print "fps: %r" % (1.0/time_diff)

    
def main(args):       
    try:
        ImageDisplay()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."

if __name__ == '__main__':
    main(sys.argv)
   