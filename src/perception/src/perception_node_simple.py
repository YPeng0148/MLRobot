# perception_node_simple.py

import rospy
from std_msgs.msg import Bool, String
from perception.msg import FloatArray
count = 0
lidar_obstacle = False
webcam_obstacle = False
lane_clear = True
ultrasonic_obstacle = False

# handle lidar's obstacle message
def handle_lidar_obstacle_info(msg):
    global lidar_obstacle
    lidar_obstacle = msg.data

# handle webcam lane info
def handle_image_info_lane(msg):
    global lane_clear
    if msg.data == "LANE UNCLEAR":
        lane_clear = False
    if msg.data == "LANE CLEAR":
        lane_clear = True

# handle webcam obstacle
def handle_image_info_obstacle(msg):
    global webcam_obstacle
    if msg.data == "CLEAR":
        webcam_obstacle = False
    if msg.data == "STOP":
        webcam_obstacle = True

def handle_ultrasonic_info(msg):
    # handle ultrasonic
    for i in msg.data:
        if (i < 30):
            ultrasonic_obstacle = True
        else:
            ultrasonic_obstacle = False

def sensor_fusion():
    # returns obstacle
    # use priority to determine when to send stop and go
    # first check ultrasonic distance sensor
    if(ultrasonic_obstacle):
        return True
    else:
        # check for lane clear and no lidar obstacles
        if (lane_clear and not lidar_obstacle):
            if(webcam_obstacle):
                return True
            return False
        else:
            return True


def main():
    rospy.init_node('perception_node', anonymous=True)
    rospy.Subscriber("lidar_obstacle", Bool, handle_lidar_obstacle_info)
    rospy.Subscriber("image_info_lane", String, handle_image_info_lane) 
    rospy.Subscriber("image_info_obstacle", String, handle_image_info_obstacle)
    #rospy.Subscriber("ultrasonic_info", FloatArray, handle_ultrasonic_info)

    pub = rospy.Publisher('obstacle', Bool, queue_size=10)
    
    obstacle = Bool()   
    rate = rospy.Rate(2) # every second
    # pause for 10 sec
    rospy.sleep(10)
    while not rospy.is_shutdown():
        obstacle.data = sensor_fusion()
        
        pub.publish(obstacle)
        rate.sleep() # sleep for 1 second


if __name__ == '__main__':
    main()
