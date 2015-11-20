#!/usr/bin/env python
import rospy
from math import tanh
from random import random
from std_msgs.msg import UInt16
from std_msgs.msg import Float32

# This node "right_error_node" subscribes to the right IR sensor, converts that value 
# to a measurement in inches. Then it publishes the error found from the desired 
# minimum to the topic "right_error".

# function: right_ir_to_error
# purpose:
# 1) Read in values from the right ir publishers "right_ir".
# 2) Convert that value to a inches measurment using converts ir to in float 5357.5113406476 x^(-1.0171714166).
# 3) Subtract that measurement from the minimum which will be 12 inches. If the subtraction shows
#    the value to be greater than 0 coerce it to 0. Regardless add 1.
# 4) Take the inverse and bring it to some power, I think 1 will work for now.

def right_ir_to_error(data):
    min_distance = 12
    # value from sensor
    sensor_read = data.data
    # if the value read in was a 0 will set the effort small by making a large read value
    right_ir_read = 0
    if sensor_read > 0:
        # reading and convertin the value
        right_ir_read = 5357.5113406746*pow(sensor_read,-1.0171714166)
    else:
        right_ir_read = 1000
    # coercing the value if it is less than the minimum
    if min_distance > right_ir_read:
        right_ir_read = min_distance
    # taking the inverse
    right_ir_read = tanh((0.5*random()+1)*(right_ir_read - min_distance)) - 1
    # publishing the result
    errorL.publish(right_ir_read)

# function: start
# purpose:
# 1) Initialize the global variable for the publisher
# 2) Initialize the publisher and set the topic name for the publisher to "right_error"
#    this value will be of the Float32 type, and set the que size to 10
# 3) Initialize the node publishing the topic "right_error". The node's name is "right_error_node".
# 4) Set the update rate to be the same as the rate information is coming in.
def start():
    # erroL is the publisher
    global errorL
    # errorL is the publishing node
    errorL = rospy.Publisher('right_error', Float32, queue_size=1)
    # starting the node 
    rospy.init_node('right_error_node')
    # subscribing to the right ir topic
    rospy.Subscriber('right_ir', UInt16, right_ir_to_error)
    rospy.spin()

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass
