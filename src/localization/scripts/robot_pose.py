#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from localization.srv import *
import math

    
robot_pose_publisher = rospy.Publisher("robot_pose", PoseStamped, queue_size=10)

theta = 0 

def handle_compass(msg):
    global theta
    theta = -msg.data*math.pi/180

def handle_gps(msg):
    rospy.wait_for_service('gps_converter')
    gps_converter = rospy.ServiceProxy('gps_converter', GpsConverter)
    req = GpsConverterRequest()
    req.gps.x = msg.x
    req.gps.y = msg.y
    res = gps_converter(req)
    
    global x
    global y
    x = res.position.x
    y = res.position.y

    pose = PoseStamped()
    pose.header.stamp = rospy.get_rostime()
    pose.header.frame_id = "world"

    pose.pose.position.x = x
    pose.pose.position.y = y 

    pose.pose.orientation = get_orientation()
        
    robot_pose_publisher.publish(pose)
        
def get_orientation():
    q = Quaternion()
    q.w = math.cos(theta/2)
    q.z = math.sin(theta/2)
    return q

if __name__ == '__main__':
    rospy.init_node("robot_pose")
    rospy.Subscriber("compass", Float64, handle_compass)
    rospy.Subscriber("gps", Point, handle_gps)

    rospy.spin()

    '''rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        pose = PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = "world"

        pose.pose.position.x = x
        pose.pose.position.y = y 

        pose.pose.orientation = get_orientation()
        
        robot_pose_publisher.publish(pose)
        rate.sleep()'''
