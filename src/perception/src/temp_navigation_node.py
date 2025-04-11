import rospy
from nav_msgs.msg import OccupancyGrid

def callback(data):
    width = data.info.width
    height = data.info.height

    origin_x = data.info.origin.position.x
    origin_y = data.info.origin.position.y

    # checks for points in front of the robot 
    #     xx|xx
    #     xx|xx
    #     xx|xx
    # ------+-------
    #       |
    #       |
    #       |

    print("Checking for obstacle...")
    # note: 
    # - need to doublecheck which direction the robot is going in
    # - need to double check width and height is--> just make sure its 
    #   not some crazy number
    for row in range(origin_y, height):
        for col in range(origin_x - width/2.0, origin_x + width/2.0):
            if(data->data[row+col] > 50)
                print("Object detected at", col, width)
                # TODO: add code to stop robot here
            

def main():
    rospy.init_node('navigation_node', anonymous = True)

    rospy.Subscriber("/map", OccupancyGrid, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
