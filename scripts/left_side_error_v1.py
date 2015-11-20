#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import UInt16
from std_msgs.msg import Float32

# This node "left_error_node" subscribes to the left IR sensor, converts that value 
# to a measurement in inches. Then it publishes the error found from the desired 
# minimum to the topic "left_error".

# function: left_ir_to_error
# purpose:
# 1) Read in values from the left ir publishers "left_ir".
# 2) Convert that value to a inches measurment using converts ir to in float 5357.5113406476 x^(-1.0171714166).
# 3) Subtract that measurement from the minimum which will be 12 inches. If the subtraction shows
#    the value to be greater than 0 coerce it to 0. Regardless add 1.
# 4) Take the inverse and bring it to some power, I think 1 will work for now.

def left_ir_to_error(data):
    min_distance = 12
    # value from sensor
    sensor_read = data.data
    # if the value read in was a 0 will set the effort small by making a large read value
    left_ir_read = 0
    if sensor_read > 0:
        # reading and convertin the value
        left_ir_read = 5357.5113406746*pow(sensor_read,-1.0171714166)
    else:
        left_ir_read = 1000
    # coercing the value if it is less than the minimum
    if min_distance > left_ir_read:
        left_ir_read = min_distance
    # taking the inverse
    left_ir_read = 1.0/(-pow(abs(min_distance - left_ir_read - 1),1/(0.5 + random.random())))
    # publishing the result
    errorL.publish(left_ir_read)

# function: start
# purpose:
# 1) Initialize the global variable for the publisher
# 2) Initialize the publisher and set the topic name for the publisher to "left_error"
#    this value will be of the Float32 type, and set the que size to 10
# 3) Initialize the node publishing the topic "left_error". The node's name is "left_error_node".
# 4) Set the update rate to be the same as the rate information is coming in.
def start():
    # erroL is the publisher
    global errorL
    # errorL is the publishing node
    errorL = rospy.Publisher('left_error', Float32, queue_size=10)
    # starting the node 
    rospy.init_node('left_error_node',anonymous=True)
    # subscribing to the left ir topic
    rospy.Subscriber('left_ir', UInt16, left_ir_to_error)
    rospy.spin()

if __name__ == '__main__':
    try:
        start()
    except rospy.ROSInterruptException:
        pass

