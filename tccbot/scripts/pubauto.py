#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Int16

def maine():
    pub_state.publish(1)
    time.sleep(3)
    pub_state.publish(2)
    time.sleep(3)

if __name__ == '__main__':
    rospy.init_node('pubauto',  anonymous=True)
    print("Batata")
    rate = rospy.Rate(10)
    pub_state = rospy.Publisher("/state_topic", Int16, queue_size=10)

    while not rospy.is_shutdown():
        maine()
        rate.sleep()