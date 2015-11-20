#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

# This ROS Node converts Joystick inputs from the joy node and publishes them on topics for red and blue bot

# declaring the global variables that hold values to be passed between functions
blue_left_thumb_value = 0
blue_right_thumb_value = 0
red_left_thumb_value = 0
red_right_thumb_value = 0 
red_a_button_state = 0
blue_a_button_state = 0
blue_bumper_state = 0
red_bumper_state = 0

# function: controller_read
# purpose: Read in joystick values from the "joy" topic
def controller_read(data):
    ### declaring the global values for the button values to publish
    global blue_left_thumb_value    
    global blue_right_thumb_value
    global red_left_thumb_value
    global red_right_thumb_value    
    global red_a_button_state
    global blue_a_button_state
    global blue_bumper_state
    global red_bumper_state
    ### thumbstick values ###
    blue_left_thumb_value = data.axes[1]
    blue_right_thumb_value = -1.0*data.axes[2]
    red_left_thumb_value = blue_left_thumb_value
    red_right_thumb_value = blue_right_thumb_value
    red_a_button_state = data.buttons[0]
    blue_a_button_state = red_a_button_state
    blue_bumper_state = data.buttons[4]
    red_bumper_state = data.buttons[5]
    
# function: start
# purpose: Intializes everything and cycles the publishers publishing the new controller values at a set rate
def start():
    ### declaring all the sensor global variables ###
    global blue_left_thumb_value
    global blue_right_thumb_value
    global red_left_thumb_value
    global red_right_thumb_value    
    global red_a_button_state
    global blue_a_button_state
    global blue_bumper_state
    global red_bumper_state
    ### declaring all the publusher and subscribers ###
    global blue_bumper_pub
    global red_bumper_pub
    global blue_a_pub
    global red_a_pub
    global blue_left_thumb_pub
    global red_left_thumb_pub
    global blue_right_thumb_pub
    global red_right_thumb_pub    

    ### initializing the publishers for the two motors ###
    blue_bumper_pub = rospy.Publisher('/blue_bot/left_bumper', Int16, queue_size=1)
    red_bumper_pub = rospy.Publisher('/red_bot/right_bumper', Int16, queue_size=1)
    blue_a_pub = rospy.Publisher('/blue_bot/a_button', Int16, queue_size=1)
    red_a_pub = rospy.Publisher('/red_bot/a_button', Int16, queue_size=1)
    blue_left_thumb_pub = rospy.Publisher('/blue_bot/left_thumb', Float32, queue_size=1)
    blue_right_thumb_pub = rospy.Publisher('/blue_bot/right_thumb', Float32, queue_size=1)
    red_left_thumb_pub = rospy.Publisher('/red_bot/left_thumb', Float32, queue_size=1)
    red_right_thumb_pub = rospy.Publisher('/red_bot/right_thumb', Float32, queue_size=1)

    ### starting the node
    rospy.init_node('controller_node')
    
    ### getting the values from the other programs. Start all the other topics before the joystick ###
    ### so it will use the most recent command to updat the desired velocity ###
    # subscribing to the joystick inputs on topic "joy" to get inputs from the controller
    joy_sub = rospy.Subscriber("joy", Joy, controller_read)
    
    r = rospy.Rate(20)

    while not rospy.is_shutdown():
        # publishing the buttons thumbstick value
        blue_bumper_pub.publish(blue_bumper_state)
        blue_a_pub.publish(blue_a_button_state)
        blue_left_thumb_pub.publish(blue_left_thumb_value)
        blue_right_thumb_pub.publish(blue_right_thumb_value)

        red_bumper_pub.publish(red_bumper_state)
        red_a_pub.publish(red_a_button_state)
        red_left_thumb_pub.publish(red_left_thumb_value)
        red_right_thumb_pub.publish(red_right_thumb_value)

        r.sleep()

if __name__ == '__main__':
    start()
