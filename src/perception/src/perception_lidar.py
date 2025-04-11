import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math

pub = rospy.Publisher('lidar_obstacle', Bool, queue_size = 2)
obstacle = Bool()
count = 0

def callback(data):
    global obstacle, count
    print("received data from lidar...")
    # obstacle = Bool()
    # NDArray of angles and distances are collected:i
    for i in range(int((data.angle_max - data.angle_min) / data.angle_increment)):
        # print("current angle: ", i, ": dist: ", data.ranges[i])
        if(data.ranges[i] <= 1.5 and data.ranges[i] >= data.range_min): # if valid point and checks before 2 meters
        #if(data.ranges[i] <= data.range_max and data.ranges[i] >= data.range_min): # if valid point
            if((80 < i) and (i < 100)):  # if obstacle in front
                obstacle.data = True
                print("obstacle detected, stop command sent")
                count = count + 1
                return
            else:
                obstacle.data = False
                #print("path clear so far")
    if (obstacle.data == False):
        print("path clear so far-------------------")
    else:
        print("should not be here...")
    count = count + 1

def main():
    rospy.init_node('perception_node', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    rate = rospy.Rate(2) # every second
    rospy.sleep(10)
    while not rospy.is_shutdown():
        pub.publish(obstacle)
        print(count)
        rate.sleep() # sleep for 1 second

    #rospy.spin()

if __name__ == '__main__':
    main()

