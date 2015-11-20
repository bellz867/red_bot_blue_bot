#!/usr/bin/env python
import rospy
from math import tanh
from random import random
from std_msgs.msg import UInt16
from std_msgs.msg import Float32
import numpy as np

# This node "front_error_node" subscribes to the front IR sensor, converts that value 
# to a measurement in inches. Then it publishes the error found from the desired 
# minimum to the topic "front_error".

front_ir_read = 0
# function: front_ir_to_error
# purpose:
# 1) Read in values from the front ir publishers "front_ir".
# 2) Convert that value to a inches measurment using converts ir to in float 5357.5113406476 x^(-1.0171714166).
# 3) Subtract that measurement from the minimum which will be 12 inches. If the subtraction shows
#    the value to be greater than 0 coerce it to 0. Regardless add 1.
# 4) Take the inverse and bring it to some power, I think 1 will work for now.

def front_ir_to_error(data):
    global front_ir_read
    min_distance = 12
    # value from sensor
    sensor_read = data.data
    # if the value read in was a 0 will set the effort small by making a large read value
    front_ir_read = 0
    if sensor_read > 0:
        # reading and convertin the value
        front_ir_read = 5357.5113406746*pow(sensor_read,-1.0171714166)
    else:
        front_ir_read = 1000
    # coercing the value if it is less than the minimum
    if min_distance > front_ir_read:
        front_ir_read = min_distance
    # taking the inverse
    front_ir_read = tanh(0.75*(front_ir_read - min_distance)) - 1

# function: start
# purpose:
# 1) Initialize the global variable for the publisher
# 2) Initialize the publisher and set the topic name for the publisher to "front_error"
#    this value will be of the Float32 type, and set the que size to 10
# 3) Initialize the node publishing the topic "front_error". The node's name is "front_error_node".
# 4) Set the update rate to be the same as the rate information is coming in.
def start():
    # erroL is the publisher
    global front_ir_read
    global errorL
    # errorL is the publishing node
    errorL = rospy.Publisher('front_error', Float32, queue_size=1)
    # starting the node 
    rospy.init_node('front_error_node')
    # subscribing to the front ir topic
    rospy.Subscriber('front_ir', UInt16, front_ir_to_error)
    
    r = rospy.Rate(25)
    while not rospy.is_shutdown():
        errorL.publish(front_ir_read)
        r.sleep()

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass

