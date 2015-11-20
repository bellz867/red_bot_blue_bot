#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32

# This node "right_motor_speed_node" takes in the speed and count for the right encoder
# then finds the linear velocity.

motor_time = 1
motor_count = 1
speed = 1

# function: right_motor_time
# purpose:
# 1) read in the time
# 2) save the time to a value
def right_motor_time(data):
    # reading and converting the value
    global motor_time 
    motor_time = data.data

# function: right_motor_count
# purpose:
# 1) read in the time
# 2) save the time to a value
def right_motor_count(data):
    # reading and converting the value
    global motor_count
    motor_count = data.data

# function: start()
# purpose:
# 1) initialize variables
# 2) initialize node
# 3) initialize publisher
# 4) initialize subscribers
# 5) compute speed using counts and time
# 6) publish speed
def start():
    global motor_time
    global motor_count
    global speed_pub
    global count_sub
    global time_sub
    global speed

    # starting the publishing node
    speed_pub = rospy.Publisher('right_motor_speed', Float32, queue_size=10)

     # starting the node 
    rospy.init_node('right_motor_speed_node')

    # subscribing to the right motor counts
    count_sub = rospy.Subscriber('right_encoder_count', Int16, right_motor_count)
    
    # subscribing to the right motor time
    time_sub = rospy.Subscriber('right_encoder_time', Int16, right_motor_time)

    # MUST DO THIS FOR SUBSCRIBER PUBLISHER PROGRAMS TO UPDATE MORE THAN ONCE 
    # DO NOT USE SPIN
    # setting the publishing rate
    r = rospy.Rate(25)

    while not rospy.is_shutdown():
        # calculating the speed
        speed = (motor_count * 3.54 * 3.14159 * 1000000)/(1200.0 * motor_time)
        
        #publishing the speed
        speed_pub.publish(speed)

        r.sleep()

if __name__ == '__main__':
    start()

