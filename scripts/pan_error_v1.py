#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16

# This node subscribes to the current cx centroid value and publishes a command to the 
# pan servo to maintain the position at the center of the frame. THe center is at 100 degrees

# function: cx_to_error
# purpose:
# 1) Read in the values from the cx publisher
# 2) Make error signal by subtracting from desired cx at center of frame 319
# 3) Make kp small for large change and more large for small change
# 4) turn that error signal into a command by multiplying by kp

def cx_to_error(data):
    # desired cx location is 319
    desired_cx = 319
    # center pan angle value is 100
    pan_center = 100
    # cx value from camera
    cx_read = data.data
    # proportional gain for the error signal, start low
    kp = 0.001
    # desired position is going to be current position of servo + kp*error

    # publishing the result
    errorL.publish(front_ir_read)

# function: start
# purpose:
# 1) Initialize the global variable for the publisher
# 2) Initialize the publisher and set the topic name for the publisher to "front_error"
#    this value will be of the Float32 type, and set the que size to 10
# 3) Initialize the node publishing the topic "front_error". The node's name is "front_error_node".
# 4) Set the update rate to be the same as the rate information is coming in.
def start():
    # erroL is the publisher
    global errorL
    # errorL is the publishing node
    errorL = rospy.Publisher('front_error', Float32, queue_size=10)
    # starting the node 
    rospy.init_node('front_error_node')
    # subscribing to the front ir topic
    rospy.Subscriber('front_ir', UInt16, front_ir_to_error)
    rospy.spin()

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass

