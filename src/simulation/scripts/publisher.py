#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    pub = rospy.Publisher('topic', String, queue_size=10)
    rospy.init_node('publisher')
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish("hello world")
        rate.sleep()
