#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

def create_custom_marker(frame_id, namespace, id, type, action, pose):
    pass


def make_point(x,y,z):
    p = Point()
    p.x = x
    p.y = y
    p.z = z
    return p

def make_color(r,g,b,a):
    c = ColorRGBA()
    c.r = r
    c.g = g
    c.b = b
    c.a = a
    return c

# the main function:
if __name__ == '__main__':
    # initialize the node
    rospy.init_node('simulator')
    
    # make publishers
    robot_publisher = rospy.Publisher('robot', MarkerArray, queue_size=10)

    # make subscribers


    # load robot model
    # robot = load_robot_model(filename)

    x = 0.0

    # make timer and start loop
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # update the robot's coordinates
        robot = MarkerArray()

        base = Marker()
        
        base.header.stamp = rospy.get_rostime()
        base.header.frame_id = "world"

        base.ns = "base"
        base.id = 0

        base.type = Marker.TRIANGLE_LIST
        base.action = Marker.ADD

        base.pose.position.x = 0
        base.pose.position.y = 0
        base.pose.position.z = 0
        base.pose.orientation.x = 0.0
        base.pose.orientation.y = 0.0
        base.pose.orientation.z = 0.0
        base.pose.orientation.w = 1.0

        base.scale.x = 1.0
        base.scale.y = 1.0
        base.scale.z = 1.0

        base.lifetime = rospy.Duration(0,0)
        base.frame_locked = False

        base.points.append(make_point(1,1,0)) 
        base.points.append(make_point(1,-1,0))
        base.points.append(make_point(-1,-1,0))
        base.points.append(make_point(1,1,0))
        base.points.append(make_point(-1,1,0))
        base.points.append(make_point(-1,-1,0))

        base.colors.append(make_color(1,0,0,1))
        base.colors.append(make_color(1,0,0,1))
        base.colors.append(make_color(1,0,0,1))
        base.colors.append(make_color(0,1,0,1))
        base.colors.append(make_color(0,1,0,1))
        base.colors.append(make_color(0,1,0,1))

        robot.markers.append(base)
        
        robot_publisher.publish(robot)
        rate.sleep()

        x = x + 0.1

'''
notes:
    the simulators main job is to:
        get the estimated pose from localization (in latitude, longitude, and bearing)
        convert to x, y, and bearing
        update the robot's frame
        and publish two messages:
            a TF message publishing the robots frame
            a marker array of all the robot config

br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = turtlename
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

'''
