import serial
import rospy
from perception.msg import FloatArray
    
def main():
    # Specify the serial port and baud rate
    serial_port = '/dev/ttyACM0'
    baud_rate = 9600

    # Open the serial port
    ser = serial.Serial(serial_port, baud_rate)
    print("Ultrasounds are connected successfully...")

    # num_us = 1
    us_pub = rospy.Publisher('ultrasonic_info', FloatArray, queue_size = 2)
    rospy.init_node('ultrasonic_node', anonymous=True)
    r = rospy.Rate(2) 

    while not rospy.is_shutdown():
        print("Ultrasound loop is running...")
        # Read the distance measurement from the Arduino
        distance = ser.readline().strip() # b'num, num'
        #print(distance)
        
        distance = str(distance)
        distances = distance.split(", ") 
        print(distances)
        distances[0] = float(distances[0][2:] )
        distances[1] = float(distances[1][:-1])
        
        print(distances)
        
        msg = FloatArray()
        msg.data = distances
        us_pub.publish(msg)

        r.sleep() # Wait for the next cycle
    
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
