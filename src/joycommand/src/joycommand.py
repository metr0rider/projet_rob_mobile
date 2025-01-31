#!/usr/bin/env python3

import numpy as np
import rospy
import roslib

from geometry_msgs.msg  import Twist
from sensor_msgs.msg import Joy

twist_msg = Twist()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rospy.set_param("/retour_base", False)
rospy.set_param("/explo_auto", False)
rospy.set_param("/speed_joy_lin", 4.0)
rospy.set_param("/speed_joy_ang", 1.0)

def joy_callback(msg):
    # This function will be called every time a message is received on the /joy topic.
    # In this example, we're publishing the received message to the /skidbot/cmd_vel topic.
    # Create a Twist message from the Joy message.
    #print(msg.axes)
    lin=rospy.get_param("/speed_joy_lin", 4.0)
    rot=rospy.get_param("/speed_joy_rot", 1.0)
    twist_msg.linear.x = lin * msg.axes[7]  # Map the y-axis of the joystick to the linear velocity.
    twist_msg.angular.z = rot * msg.axes[3]  # Map the x-axis of the joystick to the angular velocity.
    if (msg.buttons[0]==1):
    	rospy.set_param("/retour_base", True)
    if (msg.buttons[1]==1):
    	val=rospy.get_param("/explo_auto", False)
    	rospy.set_param("/explo_auto", not val)
    	
    #print(msg.buttons)
    # Publish the Twist message to the /skidbot/cmd_vel topic.
    pub.publish(twist_msg)

def main():

    # Create a node that subscribes to the /joy topic and publishes to the /cmd_vel topic.
    rospy.init_node('joycommand')

    #pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/joy', Joy, joy_callback, queue_size=10)

    # Spin the node to receive messages and call the joy_callback function for each message.
    rospy.spin()

    # Clean up before exiting.
    rospy.destroy_node()
    rospy.shutdown()

if __name__ == '__main__':
    main()
