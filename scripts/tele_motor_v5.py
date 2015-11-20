#!/usr/bin/env python
import rospy
from random import random
from std_msgs.msg import Int16
from std_msgs.msg import UInt16
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

# This ROS Node converts Joystick inputs from the joy node
# into velocity commands for the motors. If the "A" button is pressed will be autonomous
# otherwise will use the commands from the thumbsticks.

# declaring the global variables that hold values to be passed between functions
left_ir_error_cmd = 0
right_ir_error_cmd = 0
front_ir_error_cmd = 0
front_ultra_error_cmd = 0
left_motor_speed_desired = 0
right_motor_speed_desired = 0
left_motor_speed_act = 0
right_motor_speed_act = 0
left_motor_effort = 0
right_motor_effort = 0
left_thumb_value = 0
right_thumb_value = 0
a_button_state = 0
is_blocked = False
dice_roll = 0
lirs = False
rirs = False
firs = False
rturn = False
lturn = False
right_motor_errorp = 0
left_motor_errorp = 0
left_motor_error = 0
right_motor_error = 0


#### NOTE: HAD TO MOVE THE VELOCITY STUFF BECAUSE THIS CALLBACK IS ONLY UPDATED WHEN
#### THE CONTROLLER CHANGES STATE SO IT WONT WORK AS I WANT FOR AUTONOMOUS MODE
#### INSTEAD IN THIS JUST READ IN THE VALUES FROM THE JOY STICKS THEN IN ANOTHER SPOT
#### DO ALL THE VELOCITY STUFF.
# function: controller_read
# purpose:
# 1) Read in joystick values from the "joy" topic.
# 2) If the "A" button, buttons[0], is pressed go into "autonomous" mode otherwise use the commands
#    from the thumbsticks.
# When A is pressed:
# 3) Use a constant desired velocity try 12 in/sec. This will be the "baseline" for the desired velocity.
#    Get the error command values from the four sensors "front_error", "right_error", "left_error", and
#    "front_ultra_error". THe front IR and front Ultra will be averaged.
#    Convert these error commands to velocities using proportional gains KF,KL,KR. KF will equal 
#    the constant desired "baseline" velocity so that if that error command saturates to -1 there
#    will be a 0 desired forward velocity and only rotations will be possible. The left and right turns
#    will be to run at 50% the max desired forward velocity for now.
# When A is not pressed:
# 3) The desired velocity will come from the thumbsticks similar to the open loop method previously used.
#    A max velocity will be selected and the command from the thumbsticks will act as a proportional
#    command for the "baseline" desired velocity. The max will be 36 in/sec
def controller_read(data):
    ### declaring the global values for the autonomous commands and desired velocities ###
    global left_thumb_value 
    global right_thumb_value
    global a_button_state
    ### thumbstick values ###
    # the left thumb value velocity is the up or down value read from the controller
    left_thumb_value = data.axes[1]
    # the right thumb value velocity is the left or right value. Negative because the joy command negates
    right_thumb_value = -1.0*data.axes[2]
    # the A button value is the value sent by the A button determining to be in autonomous or tele mode
    a_button_state = data.buttons[0]

# this is the follow up because the issue with the joystick callback
def desired_velocity_():
    global left_ir_error_cmd
    global right_ir_error_cmd
    global front_ir_error_cmd
    global front_ultra_error_cmd
    global left_motor_speed_desired
    global right_motor_speed_desired
    global left_thumb_value
    global right_thumb_value
    global a_button_state
    global is_blocked
    global dice_roll
    global lirs
    global rirs
    global firs
    global rturn
    global lturn

    ### teleop and autonomous velocity stuff ###
    # max desired velocity when using teleop mode in inches/sec
    v_teleop_max = 36
    # max desited velocity when in autonomous mode
    v_auto_max = 36
    # the front velocity error gain in autonomous mode
    KF = v_auto_max
    # the left and right velocity error gain in autonomous mode
    KL = v_auto_max*1.25/3.0
    KR = KL

    ### thumbstick values ###
    # the left thumb value velocity is the up or down value read from the controller
    ltv = v_teleop_max * left_thumb_value
    # the right thumb value velocity is the left or right value. Negative because the joy command negates
    rtv = (v_teleop_max*1.25/3.0) * right_thumb_value
    # the A button value is the value sent by the A button determining to be in autonomous or tele mode
    abv = a_button_state

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
    
    ### If any of the commands are above the threshold value want to entire the blocked procedure
    if (abp) and (not is_blocked):
    	print "entered not is_blocked"
        ### checking if the ir commands are almost saturated for the left, right, and front IR values
        # left ir saturated
        lirs = left_ir_error_cmd < -0.95
        # right ir saturated
        rirs = right_ir_error_cmd < -0.95
        # front ir saturated
        firs = front_ir_error_cmd < -0.95
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
        lirs = left_ir_error_cmd < -0.5
        # right ir saturated
        rirs = right_ir_error_cmd < -0.5
        # front ir saturated
        firs = front_ir_error_cmd < -0.5

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
            lmv = v_auto_max - KL*left_ir_error_cmd + KR * right_ir_error_cmd + KF * (front_ir_error_cmd + front_ultra_error_cmd)/2.0
            rmv = v_auto_max + KL*left_ir_error_cmd - KR * right_ir_error_cmd + KF * (front_ir_error_cmd + front_ultra_error_cmd)/2.0
        
        if (abs(lmv) < mv) and (abs(rmv) < mv):
            lmv = 0
            rmv = 0
        else:
            lmv = lmv
            rmv = rmv
    else:
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
    left_motor_speed_desired = lmv
    right_motor_speed_desired = rmv

# function: closed_loop_command
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
    ### declaring all the publusher and subscribers ###
    global left_motor_pub
    global right_motor_pub
    global left_motor_desired_pub
    global right_motor_desired_pub
    global right_motor_errorp
    global left_motor_errorp
    global right_motor_error
    global left_motor_error

    # max output to divide command over
    max_output = 255
    
    # current controller is just a proportional controller use KCP of 0.01 for now
    KCP = 0.008
    KCD = 0.00005

    # equating previous error value to store
    left_motor_errorp = left_motor_error
    right_motor_errorp = right_motor_error

    # temporary variables for the error
    left_motor_error = left_motor_speed_desired - left_motor_speed_act
    right_motor_error = right_motor_speed_desired - right_motor_speed_act

    # calculating the commands
    left_motor_effort =  KCP * left_motor_error
    right_motor_effort = KCP * right_motor_error
    
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
# 1) read in the right speed
def right_motor_speed_read(data):
    # reading and converting the value
    global right_motor_speed_act
    right_motor_speed_act = data.data

# function: left_motor_speed_read
# purpose:
# 1) read in the left speed
def left_motor_speed_read(data):
    # reading and converting the value
    global left_motor_speed_act
    left_motor_speed_act = data.data

# function: front_ir_read
# purpose:
# 1) read in the front ir value
def front_ir_read(data):
    # reading and converting the value
    global front_ir_error_cmd
    front_ir_error_cmd = data.data

# function: front_ultra_read
# purpose:
# 1) read in the front ultra value
def front_ultra_read(data):
    # reading and converting the value
    global front_ultra_error_cmd
    front_ultra_error_cmd = data.data

# function: left_ir_read
# purpose:
# 1) read in the left ir value
def left_ir_read(data):
    # reading and converting the value
    global left_ir_error_cmd
    left_ir_error_cmd = data.data

# function: right_ir_read
# purpose:
# 1) read in the right ir value
def right_ir_read(data):
    # reading and converting the value
    global right_ir_error_cmd
    right_ir_error_cmd = data.data

# Intializes everything
def start():
    ### declaring all the sensor global variables ###
    global left_ir_error_cmd
    global right_ir_error_cmd
    global front_ir_error_cmd
    global front_ultra_error_cmd
    global left_motor_speed_desired
    global right_motor_speed_desired
    global left_motor_speed_act
    global right_motor_speed_act
    global left_motor_effort
    global right_motor_effort
    global left_thumb_value
    global right_thumb_value
    global a_button_state
    global is_blocked
    ### declaring all the publusher and subscribers ###
    global left_motor_pub
    global right_motor_pub
    global left_motor_desired_pub
    global right_motor_desired_pub
    global bocked_pub
    global joy_sub
    global left_ir_sub
    global right_ir_sub
    global fornt_ir_sub
    global front_ultra_sub
    global left_motor_sub
    global right_motor_sub

    ### initializing the publishers for the two motors ###
    left_motor_pub = rospy.Publisher('left_motor_effort', Int16, queue_size=10)
    right_motor_pub = rospy.Publisher('right_motor_effort', Int16, queue_size=10)
    left_motor_desired_pub = rospy.Publisher('left_motor_desired', Int16, queue_size=10)
    right_motor_desired_pub = rospy.Publisher('right_motor_desired', Int16, queue_size=10)
    blocked_pub = rospy.Publisher('blocked',Int16,queue_size=10)
    ### starting the node
    rospy.init_node('motor_commands')
    
    ### getting the values from the other programs. Start all the other topics before the joystick ###
    ### so it will use the most recent command to updat the desired velocity ###
    # subscribing to the IR and Ultra topics left_error, right_error, front_error, front_ultra_error
    left_ir_sub = rospy.Subscriber("left_error", Float32, left_ir_read)
    right_ir_sub = rospy.Subscriber("right_error", Float32, right_ir_read)
    front_ir_sub = rospy.Subscriber("front_error", Float32, front_ir_read)
    front_ultra_sub = rospy.Subscriber("front_ultra_error", Float32, front_ultra_read)
    # subscribing to the motor speeds
    left_motor_sub = rospy.Subscriber("left_motor_speed", Float32, left_motor_speed_read)
    right_motor_sub = rospy.Subscriber("right_motor_speed", Float32, right_motor_speed_read)
    # subscribing to the joystick inputs on topic "joy" to get inputs from the controller
    joy_sub = rospy.Subscriber("joy", Joy, controller_read)
    
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
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
