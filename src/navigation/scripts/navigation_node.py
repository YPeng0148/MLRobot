#!/usr/bin/env python3
import time
import rospy
from localization.srv import *
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from localization.msg import Instructions
from localization.msg import Instruction

#from geopy.geocoders import Nominatim
import requests
#import folium
#import serial
#import time
#import Adafruit_BBIO.UART as UART

#converts given address to coordinates
def get_lat_long_from_address(address):
   locator = Nominatim(user_agent='myGeocoder')
   location = locator.geocode(address)
   return location.latitude, location.longitude

#gets a json file containings directions
def get_directions_response(lat1, long1, lat2, long2, mode='walk'):
   url = "https://route-and-directions.p.rapidapi.com/v1/routing"
   key = "d683c6f5ffmshf24cc1a7a1f795ep119792jsnefb69665aebd"
   host = "route-and-directions.p.rapidapi.com"
   headers = {"X-RapidAPI-Key": key, "X-RapidAPI-Host": host}
   querystring = {"waypoints":f"{str(lat1)},{str(long1)}|{str(lat2)},{str(long2)}","mode":mode}
   response = requests.request("GET", url, headers=headers, params=querystring)
   
   return response

def create_coordinate_array(response):
   # use the response
   print(response.json())
   result = response.json()['features'][0]['geometry']['coordinates']
   points = [(i[1], i[0]) for i in result[0]]
   return points

def get_waypoint_distances_and_set_of_instructions(response):
    data = []
   #i = 0
    print(response)
    waypoints = response.json()['features'][0]['properties']['legs'][0]['steps']
    #print(waypoints)
    for i in range(len(waypoints)):
      #print(waypoints[i])
      data.append(waypoints[i])
      #data.append(waypoints[i]['instruction'])
    
    return data

def create_map(points):
   m = folium.Map(zoom_start=10)
   # add marker for the start and ending points
   for point in [points[0], points[-1]]:
      folium.Marker(point).add_to(m)
   # add the lines
   folium.PolyLine(points, weight=5, opacity=1).add_to(m)
   return m

def gps_find(m):
   UART.setup("UART5")
   GPS = serial.Serial('/dev/ttyO5', 9600)

   while(1):
      while GPS.inWaiting() == 0:
         print("waiting for fix")
      NMEA = GPS.readline()
      NMEA_array = NMEA.split(",")
      print(NMEA_array)
      latitude = NMEA_array[2]
      longitude = NMEA_array[3]
      folium.Marker([latitude,longitude], icon=folium.Icon(color='blue')).add_to(m)
      m.save('route_map.html') # Save updated map to file
      time.sleep(5) # Wait 5 seconds before updating again

#example
# start_address = '235 Hutchison Rd, Rochester, NY 14620'
# end_address = '120 Trustee Rd, Rochester, NY 14620'
# lat1 = get_lat_long_from_address(start_address[0])
# long1 = get_lat_long_from_address(start_address[1])
# lat2 = get_lat_long_from_address(end_address[0])
# long2 = get_lat_long_from_address(end_address[1])

#response = get_directions_response(43.12674007232403, -77.63006643069292, 43.124816415972354, -77.63061329260651)
#list_of_waypoints = create_coordinate_array(response)
#print(list_of_waypoints)
#print(get_waypoint_distances_and_set_of_instructions(response))
#m = create_map(list_of_waypoints)
#gps_find(m)
#m.save('./route_map.html')

def publish_msgs(lat1, lon1,lat2, lon2):
    print("Publishing waypoints and instructions")
    
    path_publisher = rospy.Publisher('path', Path, queue_size=10)
    instructions_publisher = rospy.Publisher('instructions', Instructions, queue_size=10)
    
    response = get_directions_response(lat1, lon1, lat2, lon2)
    waypoints = create_coordinate_array(response)
    instructions = get_waypoint_distances_and_set_of_instructions(response)
    
    path = Path()
    path.header.stamp = rospy.get_rostime()
    path.header.frame_id = "world"

    # filling up the list
    for waypoint in waypoints:
        rospy.wait_for_service('gps_converter')
        
        gps_converter = rospy.ServiceProxy('gps_converter', GpsConverter)
            
        req = GpsConverterRequest()
        req.gps.x = waypoint[0]
        req.gps.y = waypoint[1]

        res = gps_converter(req)
            
        pose = PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = "world"

        pose.pose.position.x = res.position.x
        pose.pose.position.y = res.position.y
        pose.pose.orientation.w = 1

        path.poses.append(pose)
        
    path_publisher.publish(path)
    
    instructions_msg = Instructions()

    for instruction in instructions:
        instruction_msg = Instruction()
        instruction_msg.from_index = instruction["from_index"]
        instruction_msg.to_index = instruction["to_index"]
        instruction_msg.distance = instruction["distance"]
        instructions_msg.instructions.append(instruction_msg)
    
    instructions_publisher.publish(instructions_msg)


first_message = True

def handle_gps(msg):
    global first_message
    if first_message:
        first_message = False
        publish_msgs(msg.x, msg.y, rospy.get_param('latitude'), rospy.get_param('longitude'))

if __name__ == '__main__':
    rospy.init_node('navigation_node')
    #rospy.Subscriber("gps", Point, handle_gps)
    
    path_publisher = rospy.Publisher('path', Path, queue_size=10)
    
    destination = rospy.get_param("/navigation_node/destination")

    waypoints = []
    
    if destination == 1:
        waypoints = [(14.44,-4.07),(12.22,-17.9)]
    if destination == 2:
        waypoints = [(2.22,13.834),(-12.22,17.9)]
    if destination == 3:
        waypoints = [(7.79, 4.5), (12.29,-3.26)]
    if destination == 4:
        #waypoints = [(0.0,2.0),(-1.0,1.0),(0.0,0.0)]
        waypoints = [(1,1),(1,-1),(0,0)]

    path = Path()
    path.header.stamp = rospy.get_rostime()
    path.header.frame_id = "world"

    # filling up the list
    for waypoint in waypoints:
        pose = PoseStamped()
        pose.header.stamp = rospy.get_rostime()
        pose.header.frame_id = "world"

        pose.pose.position.x = waypoint[0]
        pose.pose.position.y = waypoint[1]
        pose.pose.orientation.w = 1

        path.poses.append(pose)
    
    time.sleep(5)
    path_publisher.publish(path)
    print("navigation:",waypoints)
    rospy.spin()
