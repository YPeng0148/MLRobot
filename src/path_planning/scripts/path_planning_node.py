#!/usr/bin/env python3 
import time
import rospy
import math
import serial
import Adafruit_BBIO.UART as UART

from nav_msgs.msg import Path#, OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Float64, Bool, String
from localization.msg import Instructions

#UART setup 
UART.setup("UART5")
serial = serial.Serial(port = '/dev/ttyO5', baudrate = 9600 )

destination = (0,0)

waypoints = []

#instructions = []

compass_readings = [0,0,0,0,0,0,0,0,0,0]
compass_count = 0
compass_received_10 = False

gps_readings = (0,0)
#gps_readings = [(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0),(0,0)]
gps_count = 0
gps_received_10 = False

estimated_pose = (0,0,0)
initial_orient = True
journey_started = False

obstacle = False
#lane_clear = False
#stop_handling = False
#current_time = 0
#start_time = 0

def wrap(angle):
    if angle > 180:
        return angle - 360
    if angle < -180:
        return angle + 360
    return angle

def avg_com_helper(n):
    x = 0
    y = 0
    for i in range(n):
        x = x + math.cos(compass_readings[i] * math.pi / 180.0)
        y = y + math.sin(compass_readings[i] * math.pi / 180.0)
    return math.atan2(y/n,x/n)

def average_compass():
    if compass_received_10:
        return avg_com_helper(10)
    else:
        if compass_count == 0:
            return 0
        else:
            return avg_com_helper(compass_count)
    
def average_gps():
    '''x = 0
    y = 0
    for gps in gps_readings:
        x = x + gps[0]
        y = y + gps[1]
    
    if gps_received_10:
        return (x/10,y/10)
    else:
        if gps_count == 0:
            return (0,0)
        else:
            return (x/gps_count,y/gps_count)'''
    return gps_readings

def handle_compass(msg):
    global compass_count
    global compass_readings
    global compass_received_10

    compass_readings[compass_count] = -wrap(msg.data + 20)
    compass_count = compass_count + 1
    
    if compass_count == 10:
        compass_received_10 = True
    
    compass_count = compass_count % 10

    #print("compass:",average_compass())

def handle_path(msg):
    global waypoints
    for pose in msg.poses:
        waypoints.append((pose.pose.position.x, pose.pose.position.y))
    print("path planning:",waypoints)
    
def handle_robot_pose(msg):
    global gps_readings
    global gps_count
    global gps_received_10
    
    gps_readings =  (msg.pose.position.x, msg.pose.position.y)

    '''gps_readings[gps_count] = (msg.pose.position.x, msg.pose.position.y)
    gps_count = gps_count + 1

    if gps_count == 10:
        gps_received_10 = True
    
    gps_count = gps_count % 10'''

    #print("gps:",average_gps())
    
def handle_estimated_pose(msg):
    global estimated_pose
    estimated_pose = (msg.x,msg.y,msg.z)
    #print(msg.x,msg.y,msg.z)

def handle_instructions(msg):
    global instructions
    instructions = msg.instructions
    #print(instructions)
    #while len(list_of_waypoints) == 0:
        #continue
    #movement(instructions)

def handle_obstacles(msg):
    # checks for points in front of the robot
    #    xxx|xxx
    #     xx|xx
    #      x|x 
    # ------+-------
    #       |
    #       |
    #       |

    global obstacle
    obstacle = msg.data
    print("obstacle:", obstacle)

'''
    Occupancy Grid stuff
    width = msg.info.width
    height = msg.info.height

    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y
    
    #       450 500     250 250     
    print(width, height, origin_x, origin_y)
    # checks for points in front of the robot
    #     xx|xx
    #     xx|xx
    #     xx|xx
    # ------+-------
    #       |
    #       |
    #       |

    print("Checking for obstacle...")
    global obstacleFree
    y_limit = 2/msg.info.resolution
    x_limit = 1 #/msg.info.resolution
    # note:
    # - need to doublecheck which direction the robot is going in
    # - need to double check width and height is--> just make sure its
    #   not some crazy number
    
    #for y in range(0, height):
        #print(msg.data[y*width: (y + 1)*width])
        
    for row in range(int(origin_y), int(origin_y + y_limit)):
        for col in range(int(origin_x - x_limit), int(origin_x + x_limit)):
    #for row in range(0, int(height)):
    #    for col in range(0, int(width)):
            if(msg.data[row*width +col] > 50):
                print("Object detected at", row, col)
                obstacleFree = False
                return
    obstacleFree = True
    print("obstacle free?", obstacleFree)

def angledif(point):
    x1, y1 = average_gps()
    x2, y2 = point
    angle = math.atan2(y2 - y1, x2 - x1) * 180 / math.pi
    return angle
    
def distancedif(point):
    x1, y1 = average_gps()
    x2, y2 = point
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def movement(instructions):    
    for i in instructions:
        
        target_angle = angledif(list_of_waypoints[i.to_index])
        angledifference = clamp(target_angle + average_compass())
        
        while angledifference < -2 or angledifference > 2:
            
            print("angle diff:", angledifference)

            if angledifference < 0:
                move(0,0.2) #turn right 

            else:
                move(0, -0.2) #turn left ()
                
            target_angle = angledif(list_of_waypoints[i.to_index])
            angledifference = clamp(target_angle + average_compass())
            
        move(0,0) #stop

        distance_to_go = distancedif(list_of_waypoints[i.to_index])
        
        while distance_to_go > 0.1:
            #print("distance to go:", distance_to_go)
            move(0.4, 0) #move forward ()

            distance_to_go = distancedif(list_of_waypoints[i.to_index])
        
        move(0,0) #stop()
'''

def move(v, w):
    # if an obstacle is detected, dont move
    if(obstacle): # or not lane_clear):
        v = 0
        #w = 0
    command = '!'+ str(v) + '@' + str(w) + '#'
    serial.write(command.encode('utf-8'))    

def target_angle():
    x1 = estimated_pose[0]
    y1 = estimated_pose[1]
    x2, y2 = destination
    return math.atan2(y2 - y1, x2 - x1) * 180 / math.pi
    
def orient():
    angle = wrap(target_angle() - estimated_pose[2])
    print("angle", angle)
    w = -angle * 0.01
    
    if w > 0.2:
        w = 0.2
    if w < -0.2:
        w = -0.2
    if w > 0 and w < 0.1:
        w = 0.1
    if w < 0 and w > -0.1:
        w = -0.1

    if angle < 2 and angle > -2:
        w = 0
    return w
        
def calculate_distance():
    x1 = estimated_pose[0]
    y1 = estimated_pose[1]
    x2, y2 = destination
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def translate():
    distance = calculate_distance()
    print("distance:",distance)
    v = distance * 0.2
    
    if v > 0.2:
        v = 0.2
    if v < 0.1:
        v = 0.1
    
    if distance < 0.05:
        v = 0
        
        global initial_orient
        initial_orient = True
       
        global waypoints
        if len(waypoints) > 0:
            waypoints.pop(0)
    
    return v
'''
def handle_image_info_lane(msg):
    global lane_clear#, stop_handling, start_time, current_state
    #if stop_handling:
    #    current_time = time.time()
    #    if (current_time - start_time) > 0:
    #        stop_handling = False
    #    return
    if msg.data == "STOP":
    #    start_time = time.time()
        lane_clear = False
        # set stop_handling to true
     #   stop_handling = True

    if msg.data == "CLEAR":
        lane_clear = True
    #if msg.data == "STAIRS":
    #   obstacle = True



def handle_image_info(msg):
    global obstacle#, stop_handling, start_time, current_state
    #if stop_handling:
    #    current_time = time.time()
    #    if (current_time - start_time) > 0:
    #        stop_handling = False
    #    return
    if msg.data == "STOP":
    #    start_time = time.time()
        obstacle = True
        # set stop_handling to true
     #   stop_handling = True

    if msg.data == "CLEAR":
        obstacle = False
    #if msg.data == "STAIRS":
    #   obstacle = True
'''

if __name__ == "__main__":
    rospy.init_node("motion_commands")
    rospy.Subscriber("path", Path, handle_path)
    
    print("path planning: path subcriber should be ready?")
    rospy.Subscriber("robot_pose", PoseStamped, handle_robot_pose)
    rospy.Subscriber("compass", Float64, handle_compass)
    #rospy.Subscriber("instructions", Instructions, handle_instructions)
    
    rospy.Subscriber("estimated_pose", Point, handle_estimated_pose)
    rospy.Subscriber("obstacle", Bool, handle_obstacles, queue_size = 1, buff_size = 2**24)
    #rospy.Subscriber("image_info", String, handle_image_info)
    #rospy.Subscriber("image_info_lane", String, handle_image_info_lane)
    
    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        
        # trying to see if i can kill all nodes through python 
        
        
        #print(estimated_pose)
        if len(waypoints) > 0:
            destination = waypoints[0]
        
        w = orient()
        if w == 0:
            initial_orient = False
        if initial_orient:
            v = 0
        else:
            v = translate()
        move(v,w)
        rate.sleep()
