#!/usr/bin/env python
import rospy
import math
import time
from math import ceil
import numpy as np
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float32

# This ROS Node takes in the radius and centroid of the ball and produces pan and tilt
# motor commands. The feedback control will be a proportional controller that generates 
# commands in a location in degrees 

#### X WILL BE THE LEFT RIGHT PAN VALUE AND Y WILL BE THE UP DOWN TILT VALUE

# the measured pan angle 
thxa = 0
# the measured tilt angle 
thya = 0
# the error in pan angle
thxe = 0.0
# the error in tilt angle
thye = 0.0
# the desired angle for the pan
thxd = 0.0
# the desired angle for the tilt
thyd = 0.0
# area value
area_val = 0

# function: get_pan_angle
# purpose: gets the current pan servo angle  topic "pan_servo_curent"
# 1) read in the pan angle
def get_pan_angle(data):
    global thxa
    # getting the data
    thxa = data.data
    # print "pan angle = %r" % thxa

# function: get_tilt_angle
# purpose: gets the current tilt servo angle topic "tilt_servo_curent"
# 1) read in the tilt angle
def get_tilt_angle(data):
    global thya
    # getting the data
    thya = data.data

# function: get_pan_error
# purpose: gets the current pan servo angle error topic "pan_servo_error"
# 1) read in the pan angle error
def get_pan_error(data):
    global thxe
    # getting the data
    thxe = data.data

# function: get_tilt_error
# purpose: gets the current tilt servo angle error topic "tilt_servo_error"
# 1) read in the tilt angle error
def get_tilt_error(data):
    global thye
    # getting the data
    thye = data.data

# function: get_area
# purpose: gets the current area
# 1) read in the area
def get_area(data):
    global area_val
    # getting the data
    area_val = data.data

# Intializes everything
def start():
    ### declaring all the sensor global variables ###
    global thxa
    global thya
    global thxe
    global thye
    global thxd
    global thyd
    global area_val
    global pan_sub
    global tilt_sub
    global pan_er_sub
    global tilt_er_sub
    global area_sub


    # area threshold for object to be tracked
    area_thresh = 200
    
    ### initializing the publishers for the two servos ###
    pan_pub = rospy.Publisher('pan_servo_desired', Int16, queue_size=1)
    tilt_pub = rospy.Publisher('tilt_servo_desired', Int16, queue_size=1)
    
    ### starting the node
    rospy.init_node('pan_and_tilt')
    
    ### getting the values from the other programs. Start all the other topics before the joystick ###
    ### so it will use the most recent command to updat the desired velocity ###
    # subscribing to the IR and Ultra topics left_error, right_error, front_error, front_ultra_error
    pan_sub = rospy.Subscriber("pan_servo_current", Int16, get_pan_angle)
    tilt_sub = rospy.Subscriber("tilt_servo_current", Int16, get_tilt_angle)
    pan_er_sub = rospy.Subscriber("pan_servo_error", Float32, get_pan_error)
    tilt_er_sub = rospy.Subscriber("tilt_servo_error", Float32, get_tilt_error)
    area_sub = rospy.Subscriber("area", Int32, get_area)

    # creating some arrays for the error terms
    errory = np.ones((1,3))
    errory_derivative = np.ones((1,3))
    errory_integral = np.ones((1,5))
    errorx = np.ones((1,3))
    errorx_derivative = np.ones((1,3))
    errorx_integral = np.ones((1,5))

    # time difference
    t1 = 0
    t2 = 0
    time_diff = 0

    # pid gains
    kp = 0.11
    kd = 0.007
    ki = 0.65

    r = rospy.Rate(30)

    while not rospy.is_shutdown():
        t1 = t2
        t2 = time.time()
        time_diff = t2- t1
        print "loop time: %r" % (1.0/time_diff)
        # checking if the area is greater than the threshold
        # if it is use the desired value
        # otherwise go center frame
        print "area = %r" % (area_val)
        if area_val > area_thresh:
            previous_meany = np.mean(errory)
            previous_meanx = np.mean(errorx)
            # shifting all the arrays one value to the left
            # if it is less than the length-1 of the arrays will update
            for index_of in xrange(0,len(errory_integral)-2):
                if index_of<len(errory)-1:
                    errory[index_of] = errory[index_of+1]
                    errorx[index_of] = errorx[index_of+1]
                    errory_derivative[index_of] = errory_derivative[index_of+1]
                    errorx_derivative[index_of] = errorx_derivative[index_of+1]
                    errory_integral[index_of] = errory_integral[index_of+1]
                    errorx_integral[index_of] = errorx_integral[index_of+1]
                else:
                    errory_integral[index_of] = errory_integral[index_of+1]
                    errorx_integral[index_of] = errorx_integral[index_of+1]
            # updating the last value for the error
            errory[len(errory)-1] = thye
            errorx[len(errorx)-1] = thxe
            # updating the last value for the derivative
            errory_derivative[len(errory_derivative)-1] = (np.mean(errory)-previous_meany)/time_diff
            errorx_derivative[len(errorx_derivative)-1] = (np.mean(errorx)-previous_meanx)/time_diff
            # updating the last value for the integral
            errory_integral[len(errory_integral)-1] = thye
            errorx_integral[len(errory_integral)-1] = thxe

            # updating the desired pan angle and tilt angles
            thyd = kp*np.mean(errory) + kd*np.mean(errory_derivative) + ki*np.mean(np.trapz(errory_integral,dx=time_diff))
            thxd = kp*np.mean(errorx) + kd*np.mean(errorx_derivative) + ki*np.mean(np.trapz(errorx_integral,dx=time_diff))
            print "kp: %r\n\tkd: %r\n\t\tki:%r"% (kp*np.mean(errory),kd*np.mean(errory_derivative),ki*np.mean(np.trapz(errory_integral,dx=time_diff)))
        else:
            print "nope"
            thxd = 1000
            thyd = 1000

        print "pan actual = %r\n\tpan error = %r\n\t\tpan desired = %r" %(thxa,thxe,thxd+thxa)
        print "tilt actual = %r\n\ttilt error = %r\n\t\ttilt desired = %r" %(thya,thye,thyd+thya)
        # publishing the desired angles
        pan_pub.publish(int(thxd))
        tilt_pub.publish(int(thyd))
        
        # delay
        r.sleep()

if __name__ == '__main__':
    start()
