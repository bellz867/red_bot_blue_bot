#!/usr/bin/env python
import rospy
import time
import numpy as np
from random import random
from std_msgs.msg import Int16
from std_msgs.msg import UInt16
from std_msgs.msg import Float32

# This ROS Node converts Joystick inputs from the joy node
# into velocity commands for the motors. If the "A" button is pressed will be autonomous
# otherwise will use the commands from the thumbsticks.

# declaring the global variables that hold values to be passed between functions
left_ir_error_cmd = 0
right_ir_error_cmd = 0
front_ir_error_cmd = 0

left_motor_speed_desired = 0
right_motor_speed_desired = 0

left_motor_speed_act = 0
right_motor_speed_act = 0

left_motor_effort = 0
right_motor_effort = 0

left_thumb_value = 0
right_thumb_value = 0
a_button_value = 0
right_bumper_value = 0

is_blocked = False
dice_roll = 0
lirs = False
rirs = False
firs = False
rturn = False
lturn = False

t1 = 0
t2 = 0
time_diff = 0

# creating some arrays for the error terms
error_left = np.ones((1,4))
error_left_derivative = np.ones((1,4))
error_left_integral = np.ones((1,10))
error_right = np.ones((1,4))
error_right_derivative = np.ones((1,4))
error_right_integral = np.ones((1,10))

# function: desired_velocity
# purpose:
# 1) using the values being updated by the button topics and the IR topics calculates what the desired velocity
#    should be. Max velocity is set to 12 in/sec for now
def desired_velocity_():
    global left_ir_error_cmd
    global right_ir_error_cmd
    global front_ir_error_cmd

    global left_motor_speed_desired
    global right_motor_speed_desired
    
    global left_thumb_value
    global right_thumb_value
    global a_button_value
    global right_bumper_value
    
    global is_blocked
    global dice_roll
    global lirs
    global rirs
    global firs
    global rturn
    global lturn

    ### teleop and autonomous velocity stuff ###
    # max desired velocity when using teleop mode in inches/sec
    v_teleop_max = 12
    # max desited velocity when in autonomous mode
    v_auto_max = 0
    # the front velocity error gain in autonomous mode
    KF = v_auto_max
    # turn gain
    KTG = 0.4
    # the left and right velocity error gain in autonomous mode
    KL = v_auto_max * KTG
    KR = KL

    ### thumbstick values ###
    # the left thumb value velocity is the up or down value read from the controller
    ltv = v_teleop_max * left_thumb_value
    # the right thumb value velocity is the left or right value. Negative because the joy command negates
    rtv = (v_teleop_max*KTG) * right_thumb_value
    # the A button value is the value sent by the A button determining to be in autonomous or tele mode
    abv = a_button_value
    # the left bumber value will tell if it is desired to drive bot around
    bv = right_bumper_value

    ### temporary values and minimum velocity values
    # minimum velocity in/sec to turn on otherwise send 0 in/sec
    mv = 2
    # left motor desired velocity temp
    lmv = 0
    # right motor desired velocity temp
    rmv = 0
    
    ### checking the six possible thumbstick orientations and if A is pressed or not ###
    # left_thumb_up
    ltu = ltv > mv
    # left_thumb_down
    ltd = ltv < -mv
    # right_thumb_right
    rtr = rtv > mv
    # right_thumb_left
    rtl = rtv < -mv
    # left_thumb_off
    lto = (not ltu) and (not ltd)
    # right_thumb_off
    rto = (not rtr) and (not rtl)
    # A button pressed
    abp = abv > 0
    # bumper pressed
    bp = bv > 0
    
    ### If any of the commands are above the threshold value want to entire the blocked procedure
    if (abp) and (not is_blocked):
    	print "entered not is_blocked"
        ### checking if the ir commands are almost saturated for the left, right, and front IR values
        # left ir saturated
        lirs = left_ir_error_cmd < -0.99
        # right ir saturated
        rirs = right_ir_error_cmd < -0.99
        # front ir saturated
        firs = front_ir_error_cmd < -0.99
        ### checking to see if any of the conditions for a problem are detected:
        # checking if all three are saturated
        allsat = lirs and rirs and firs
        # checking if the front is saturated but the sides are not saturated
        frsat = firs and not (lirs and rirs)
        # checking if the sides are saturated but not the front 
        sisat = ( not firs ) and (lirs and rirs)
        # if any of the conditions are true will enter the blocked procedure and roll the dice to
        # determine the direction to turn out of the block
        if allsat or frsat or sisat:
            print "\tentered the is_blocked change spot"
            is_blocked = True
            # rolling the dice to determine which way to turn if greater than 50 turn right otherwise
            # turn left
            dice_roll = random() * 100
            print "\t\tdice %r" % dice_roll
    # not using an elif because want this to happen immediately if the above is true
    # if the a button is pressed and the robot is blocked want to enter the check to see if 
    # either of the two turn conditions are true. If they are not true will se the blocked
    # boolean back to False
    if abp and is_blocked:
    	print "\t\t\tentered the next stage"
        ### checking if the ir commands are almost saturated for the left, right, and front IR values
        # left ir saturated
        lirs = left_ir_error_cmd < -0.75
        # right ir saturated
        rirs = right_ir_error_cmd < -0.75
        # front ir saturated
        firs = front_ir_error_cmd < -0.75

        # checking the two possible block conditions
        if (rirs or firs) and (dice_roll > 50):
            print "\t\t\t\tentered the right turn"
            rturn = True
            lturn = False
        elif (lirs or firs) and (dice_roll <= 50):
            print "\t\t\t\tentered the left turn"
            lturn = True
            rturn = False
        else:
            print "\t\t\t\tentered the make false"
            rturn = False
            lturn = False
            is_blocked = False


    # checking if the A button is pressed will go into autonomous mode to determine the desired
    # velocity otherwise will use teleop mode to determine the desired velocity
    if abp:
        # if the robot is blocked and a right turn was chosen then turning right at max turn
        # if the robot is blocked and a left turn was chosen then turning left at max turn
        # otherwise operating in normal autonomous mode
        if is_blocked and rturn:
            lmv = KL * 2.0 / 1.25
            rmv = -1.0 * KR * 2.0 / 1.25
        elif is_blocked and lturn:
            lmv = -1.0 * KL * 2.0 / 1.25
            rmv = KR * 2.0 / 1.25
        else:
        # calculate the desired velocity values
            lmv = v_auto_max - KL*left_ir_error_cmd + KR * right_ir_error_cmd + KF * front_ir_error_cmd
            rmv = v_auto_max + KL*left_ir_error_cmd - KR * right_ir_error_cmd + KF * front_ir_error_cmd
        
        if (abs(lmv) < mv) and (abs(rmv) < mv):
            lmv = 0
            rmv = 0
        else:
            lmv = lmv
            rmv = rmv
    elif bp:
        # want to disable the block if I release the A button
        is_blocked = False
        # go forward straight: increase both positively
        # go backward straight: increase bot negatively
        if (ltu and rto) or (ltd and rto):
            lmv = ltv
            rmv = ltv
        # go forward and right: increase left motor positively and decrease right motor positively
        # go forward and left: increase right motor positively and decrease left motor positively
        elif (ltu and rtr) or (ltu and rtl):
            lmv = ltv + rtv
            rmv = ltv - rtv
        # go backward and left: increase right motor negatively and decrease left motor negatively
        # go backward and right: decrease right motor negatively and increase left motor negatively
        elif (ltd and rtl) or (ltd and rtr):
            lmv = ltv - rtv
            rmv = ltv + rtv
        # rotate right in place: increase left motor positively and increase right motor negatively
        # rotate left in place: increase left motor negatively and increase right motor positively
        elif (lto and rtr) or (lto and rtl):
            lmv = rtv * 2.5
            rmv = -rtv * 2.5
        else:
            lmv = 0
            rmv = 0
    else:
        # want to disable the block if I release the A button
        is_blocked = False        
        lmv = 0
        rmv = 0
    left_motor_speed_desired = lmv
    right_motor_speed_desired = rmv

# function: closed_loop_command
# purpose:
# 1) The loop will be closed using the calculated linear velocity published on the topics 
#    "right_motor_speed" and "left_motor_speed".
# 2) This error signal will pass through the controller which currently will just be a proportional
#    controller with gain KP
def closed_loop_command():
    # global variables
    global left_motor_speed_desired
    global right_motor_speed_desired
    global left_motor_speed_act
    global right_motor_speed_act
    global left_motor_effort
    global right_motor_effort

    global time_diff

    global error_left
    global error_left_derivative
    global error_left_integral
    global error_right
    global error_right_derivative 
    global error_right_integral

    # max output to divide command over
    max_output = 255
    
    # current controller is just a proportional controller use KCP of 0.01 for now
    KCP = 0.0065
    KCD = 0.0001
    KCI = 0.005

    # getting the previous mean for the derivative calc
    previous_mean_right = np.mean(error_right)
    previous_mean_left = np.mean(error_left)

    # temporary variables for the error
    left_motor_error = left_motor_speed_desired - left_motor_speed_act
    right_motor_error = right_motor_speed_desired - right_motor_speed_act

    # shifting all the arrays one value to the left
    # if it is less than the length-1 of the arrays will update
    for index_of in xrange(0,len(error_left_integral)-2):
        if index_of < len(error_left)-1:
            error_left[index_of] = error_left[index_of+1]
            error_right[index_of] = error_right[index_of+1]
            error_left_derivative[index_of] = error_left_derivative[index_of+1]
            error_right_derivative[index_of] = error_right_derivative[index_of+1]
            error_left_integral[index_of] = error_left_integral[index_of+1]
            error_right_integral[index_of] = error_right_integral[index_of+1]
        else:
            error_left_integral[index_of] = error_left_integral[index_of+1]
            error_right_integral[index_of] = error_right_integral[index_of+1]
    
    # updating the last value for the error
    error_left[len(error_left)-1] = left_motor_error
    error_right[len(error_right)-1] = right_motor_error

    # updating the last value for the derivative
    error_left_derivative[len(error_left_derivative)-1] = (np.mean(error_left)-previous_mean_left)/time_diff
    error_right_derivative[len(error_right_derivative)-1] = (np.mean(error_right)-previous_mean_right)/time_diff

    # updating the last value for the integral
    error_left_integral[len(error_left_integral)-1] = left_motor_error
    error_right_integral[len(error_right_integral)-1] = right_motor_error

    # updating the desired left and right commands
    left_motor_effort = KCP*np.mean(error_left) + KCD*np.mean(error_left_derivative) + KCI*np.mean(np.trapz(error_left_integral,dx=time_diff))
    right_motor_effort = KCP*np.mean(error_right) + KCD*np.mean(error_right_derivative) + KCI*np.mean(np.trapz(error_right_integral,dx=time_diff))
    
    # dividing over the available output
    left_motor_effort = left_motor_effort * max_output
    right_motor_effort = right_motor_effort * max_output
    
    #### checking the cases for the effort
    # left motor saturated positively
    lmsp = left_motor_effort > max_output
    
    # left motor saturated negatively
    lmsn = left_motor_error < (-1 * max_output)
    
    # right motor saturated positively
    rmsp = right_motor_effort > max_output
    
    # right motor saturated negatively
    rmsn = right_motor_error < (-1 * max_output)
    
    # checking if the left motor is saturated and coercing it if it is
    if lmsp:
        left_motor_effort = max_output
    elif lmsn:
        left_motor_effort = (-1 * max_output)
    else:
        left_motor_effort = left_motor_effort

    # checking if the left motor is saturated and coercing it if it is
    if rmsp:
        right_motor_effort = max_output
    elif rmsn:
        right_motor_effort = (-1 * max_output)
    else:
        right_motor_effort = right_motor_effort


# function: right_motor_speed_read
# purpose:
# 1) read "right_motor_speed" topic
def right_motor_speed_read(data):
    # reading and converting the value
    global right_motor_speed_act
    right_motor_speed_act = data.data

# function: left_motor_speed_read
# purpose:
# 1) read "left_motor_speed" topic
def left_motor_speed_read(data):
    # reading and converting the value
    global left_motor_speed_act
    left_motor_speed_act = data.data

# function: front_ir_read
# purpose:
# 1) read "front_ir" topic
def front_ir_read(data):
    # reading and converting the value
    global front_ir_error_cmd
    front_ir_error_cmd = data.data

# function: left_ir_read
# purpose:
# 1) read "left_ir" topic
def left_ir_read(data):
    # reading and converting the value
    global left_ir_error_cmd
    left_ir_error_cmd = data.data

# function: right_ir_read
# purpose:
# 1) read "right_ir" topic
def right_ir_read(data):
    # reading and converting the value
    global right_ir_error_cmd
    right_ir_error_cmd = data.data

# function: right_thumb_read
# purpose:
# 1) Read "right_thumb" topic
def right_thumb_read(data):
    global right_thumb_value
    right_thumb_value = data.data

# function: left_thumb_read
# purpose:
# 1) Read "left_thumb" topic
def left_thumb_read(data):
    global left_thumb_value
    left_thumb_value = data.data

# function: right_bumper_read
# purpose:
# 1) Read "right_bumper" topic
def right_bumper_read(data):
    global right_bumper_value
    right_bumper_value = data.data

# function: a_button_read
# purpose:
# 1) Read "a_button" topic
def a_button_read(data):
    global a_button_value
    a_button_value = data.data

# Intializes everything
def start():
    ### declaring all the sensor global variables ###
    global left_ir_error_cmd
    global right_ir_error_cmd
    global front_ir_error_cmd
    global left_motor_speed_desired
    global right_motor_speed_desired
    global left_motor_speed_act
    global right_motor_speed_act
    global left_motor_effort
    global right_motor_effort
    global left_thumb_value
    global right_thumb_value
    global a_button_value
    global right_bumper_value
    global is_blocked
    ### declaring all the publusher and subscribers ###
    global left_motor_pub
    global right_motor_pub
    
    global left_motor_desired_pub
    global right_motor_desired_pub
    
    global left_thumb_sub
    global right_thumb_sub
    global a_button_sub
    global right_bumper_sub

    global blocked_pub
    global left_ir_sub
    global right_ir_sub
    global fornt_ir_sub
    
    global left_motor_sub
    global right_motor_sub

    global time_diff
    global t1
    global t2

    ### initializing the publishers for the two motors ###
    left_motor_pub = rospy.Publisher('left_motor_effort', Int16, queue_size=1)
    right_motor_pub = rospy.Publisher('right_motor_effort', Int16, queue_size=1)
    left_motor_desired_pub = rospy.Publisher('left_motor_desired', Int16, queue_size=1)
    right_motor_desired_pub = rospy.Publisher('right_motor_desired', Int16, queue_size=1)
    blocked_pub = rospy.Publisher('blocked',Int16,queue_size=1)
    ### starting the node
    rospy.init_node('motor_commands')
    
    ### getting the values from the other programs. Start all the other topics before the joystick ###
    ### so it will use the most recent command to updat the desired velocity ###
    # subscribing to the IR topics left_error, right_error, front_error
    left_ir_sub = rospy.Subscriber("left_error", Float32, left_ir_read)
    right_ir_sub = rospy.Subscriber("right_error", Float32, right_ir_read)
    front_ir_sub = rospy.Subscriber("front_error", Float32, front_ir_read)
    # subscribing to the motor speeds
    left_motor_sub = rospy.Subscriber("left_motor_speed", Float32, left_motor_speed_read)
    right_motor_sub = rospy.Subscriber("right_motor_speed", Float32, right_motor_speed_read)
    #subscribing to button topics
    left_thumb_sub = rospy.Subscriber("left_thumb", Float32, left_thumb_read)
    right_thumb_sub = rospy.Subscriber("right_thumb", Float32, right_thumb_read)
    right_bumper_sub = rospy.Subscriber("right_bumper", Int16, right_bumper_read)
    a_button_sub = rospy.Subscriber("a_button", Int16, a_button_read)
    
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        t1 = t2
        t2 = time.time()
        time_diff = t2- t1
        print "loop time: %r" % (1.0/time_diff)
        # getting the desired velocity
        desired_velocity_()
        # publishing the desired command
        closed_loop_command()
        # publishing the desired velocities
        left_motor_desired_pub.publish(left_motor_speed_desired)
        right_motor_desired_pub.publish(right_motor_speed_desired)
        # publishing the desired efforts
        left_motor_pub.publish(left_motor_effort)
        right_motor_pub.publish(right_motor_effort)
        print "is_blocked %r" % is_blocked
        blocked_pub.publish(is_blocked)
        r.sleep()

if __name__ == '__main__':
    start()
