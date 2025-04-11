#!/usr/bin/env python3

# required for ros
import rospy
from geometry_msgs.msg import Point

# required for UART communication 
import serial
import Adafruit_BBIO.UART as UART

if __name__ == '__main__':
    
    count = 0

    # setting up the ros node
    rospy.init_node('gps_node')
    gps_publisher = rospy.Publisher('gps', Point, queue_size=10)
    
    # setting up UART5
    UART.setup('UART1')
    gps = serial.Serial('/dev/ttyO1', 9600)
    
    f = open("wegmans_1.csv", "w")

    # setting up an infinite loop
    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
       
        try:
            # reading the gps info
            data = gps.readline().decode().split(',')
        except UnicodeDecodeError:
            pass
        else:
            # creating the Point message to be published
            point = Point()
            if data[0] == '$GPGGA' and data[6] == '1':
            
                lat_sign = 1
                if data[3] == "S":
                    lat_sign = -1
            
                lon_sign = 1
                if data[5] == "W":
                    lon_sign = -1
            
                lat_deg = int(data[2][:2])
                lat_min = float(data[2][2:]) / 60
            
                lon_deg = int(data[4][:3])
                lon_min = float(data[4][3:]) / 60
            
                point.x =  lat_sign * (lat_deg + lat_min)
                point.y =  lon_sign * (lon_deg + lon_min)
            
                print('gps:',point.x,point.y)

                f.write(str(point.x)+","+str(point.y)+"\n") 
                
                count = count + 1
                if count == 50:
                    break
                gps_publisher.publish(point)
        rate.sleep()

    f.close()

#while gps.inWaiting()==0:
    # print("Waiting for Fix")
