#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def handle_msg(data):
    print("got a message!")    

if __name__ == '__main__':
    rospy.init_node('subscriber')
    rospy.Subscriber("topic", String, handle_msg)
    rospy.spin()
