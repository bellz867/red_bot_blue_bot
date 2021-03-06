#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy

# Base on the joystick code by Author: Andrew Dai but converted for my purpose
# This ROS Node converts Joystick inputs from the joy node
# into commands for the motors

# Receives joystick messages (subscribed to Joy topic) then converts that to 
# Int16 messages to the topics for: left_motor_effort and right_motor_effort.
# Then will take the value passed in by the joystick left thumbstick up and down (index 1) and 
# and the value passed in by the joystick right thumbstick left and right (index 2) and 
# shift the values right by 7. Doing this because the numbers coming in are 16 bit signed and 
# the desired output is an 9 bit signed number. The output to the motor is actually an 8 bit 
# unsigned number but I am using the nineth bit for the desired direction and converting 
# it on the Mega. Shifting the number to the left effectively spreading the value over the integer 
# range -255 to +255. This value will be added to the obosite side and subtracted from the same 
# side so the vehicle will turn even if no value is applied to the left thumbstick. An example 
# of the operation being a desired right turn, the thumbstick applied and the value is subtracted # from the right motor rotating it backward and the value will be added to the left motor rotating # it forward allowing for a right turn. If the vehicle is already in motion and the value is 
# applied then the same thing will happen only now they will no longer be adding or subtracting 
# from zero but rather the left thumbstick value. This shows that each right value with just 
# subtract or add to the left value.

def callback(data):
    # the left thumb value is the up or down value read from the controller
    left_thumb_value = 100*data.axes[1]
    # the right thumb value is the left right value.
    right_thumb_value = 100*data.axes[2]
    # right motor command value
    right_motor_command = 0
    # left motor command value
    left_motor_command = 0
    # minimum value to turn on
    mc = 20

    # checking to see if the left thumbstick is up or down
    if left_thumb_value < -mc:
        right_motor_command = int(left_thumb_value - right_thumb_value)
        left_motor_command = int(left_thumb_value + right_thumb_value)

    elif left_thumb_value > mc:
        right_motor_command = int(left_thumb_value + right_thumb_value)
        left_motor_command = int(left_thumb_value - right_thumb_value)

    else:
        right_motor_command = 0
        left_motor_command = 0
        
    
    left_motor.publish(left_motor_command)
    right_motor.publish(right_motor_command)

# Intializes everything
def start():
    # publishing to the left motor and the right motor
    global left_motor
    global right_motor
    
    left_motor = rospy.Publisher('left_motor_effort', Int16, queue_size=10)
    right_motor = rospy.Publisher('right_motor_effort', Int16, queue_size=10)
    
    # subscribing to the joystick inputs on topic "joy" to get inputs from the controller
    rospy.Subscriber("joy", Joy, callback)
    # starting the node 
    rospy.init_node('Joy2Turtle')
    rospy.spin()

if __name__ == '__main__':
    start()

