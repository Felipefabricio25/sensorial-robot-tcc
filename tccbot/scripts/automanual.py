#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

var = Int16
vjoy = Twist

time.sleep(5)

def state(data):
    global var
    var = data

def veljoy(data):
    global vjoy
    vjoy = data

def maine():
    global var
    global vjoy
    if var.data == 1:
        #cancel goal
        pub_vel.publish(vjoy)
    if var.data == 2:
        vjoy.linear.x = 0
        vjoy.angular.y = 0
        pub_vel.publish(vjoy)
        #keep goal running

if __name__ == '__main__':
    rospy.init_node('automanual',  anonymous=True)
    rospy.Subscriber("/state_topic", Int16, state)
    rospy.Subscriber("/cmd_vel_joy", Twist, veljoy)
    rate = rospy.Rate(10)
    pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    while not rospy.is_shutdown():
            maine()
            rate.sleep()