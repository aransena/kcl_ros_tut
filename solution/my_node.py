#!/usr/bin/env python
"""
Created on Mon Nov 13 14:25:01 2017

@author: viki
"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

global pub

# Subscriber callback function, could be called anything but must match
# name of callback function when setting up your subscriber below.
def sub_cb(msg):
    # Instead of moving the messages around the program, 
    # we give the callback access to the publisher via a global
    global pub 
    msg.angular.z = msg.angular.z*-1 # Invert angular component
    msg.linear.x = msg.linear.x*-1 # Invert linear component
    pub.publish(msg) # Publish the control message
    

# Program begins executing from here
rospy.init_node('my_node') # initialise the node

# Setup the publisher, with arguments topic name, topic type, queue size
pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)

# Setup the publisher, with arguments topic name, topic type, callback functione
rospy.Subscriber('/turtle1/cmd_vel', Twist, sub_cb)

# Keeps the program running until the ROS node is shut down (Ctrl+c)
# Alternative use while loop method with a Rate sleep as shown in slides
# This line is REQUIRED if not using a while loop, otherwise program will
# simply end after setting up the subscriber.
rospy.spin()