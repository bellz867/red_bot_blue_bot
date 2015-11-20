#!/usr/bin/env python
import rospy
from math import tanh
from random import random
from std_msgs.msg import UInt16
from std_msgs.msg import Float32

# This node "front_ultra_ultra_error_node" subscribes to the front_ultra IR sensor, converts that value 
# to a measurement in inches. Then it publishes the error found from the desired 
# minimum to the topic "front_ultra_error".

# function: front_ultra_ir_to_error
# purpose:
# 1) Read in values from the front ultra
# 2) Convert that value to a inches measurment using converts ir to in float 5357.5113406476 x^(-1.0171714166).
# 3) Subtract that measurement from the minimum which will be 12 inches. If the subtraction shows
#    the value to be greater than 0 coerce it to 0. Regardless add 1.
# 4) Take the inverse and bring it to some power, I think 1 will work for now.

def front_ultra_to_error(data):
    min_distance = 12
    # reading in the value
    sensor_read = data.data
    front_ultra_read = 0
    # checking to see if the error reading on the sensor is noise that is if it is less than 300 if
    # it is will coerce it to a large number to have a small effect otherwise will convert to inches
    if sensor_read < 300:
        front_ultra_read = 100
    else:
        front_ultra_read = sensor_read/25.4;
        
    # coercing the value if it is less than the minimum
    if min_distance > front_ultra_read:
        front_ultra_read = min_distance
    # taking the inverse
    front_ultra_read = tanh((0.5*random()+0.25)*(front_ultra_read - min_distance)) - 1
    # publishing the result
    errorL.publish(front_ultra_read)

# function: start
# purpose:
# 1) Initialize the global variable for the publisher
# 2) Initialize the publisher and set the topic name for the publisher to "front_ultra_error"
#    this value will be of the Float32 type, and set the que size to 10
# 3) Initialize the node publishing the topic "front_ultra_error". The node's name is "front_ultra_error_node".
# 4) Set the update rate to be the same as the rate information is coming in.
def start():
    # erroL is the publisher
    global errorL
    # errorL is the publishing node
    errorL = rospy.Publisher('front_ultra_error', Float32, queue_size=10)
    # starting the node 
    rospy.init_node('front_ultra_error_node')
    # subscribing to the front_ultra topic
    rospy.Subscriber('ultra', UInt16, front_ultra_to_error)
    rospy.spin()

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass

