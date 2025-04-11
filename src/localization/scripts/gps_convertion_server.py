#!/usr/bin/env python3

from localization.srv import GpsConverter, GpsConverterResponse
import rospy
import math
from geographiclib.geodesic import Geodesic

def convert(req):
    
    geodesic = Geodesic.WGS84.Inverse(43.12663, -77.63024, req.gps.x, req.gps.y)
    distance = geodesic["s12"]
    bearing = geodesic["azi1"]
        
    res = GpsConverterResponse()
    res.position.x = distance * math.cos(bearing * math.pi / 180)
    res.position.y = -distance * math.sin(bearing * math.pi / 180)
    
    return res

if __name__ == "__main__":
    rospy.init_node('gps_convertion_server')
    rospy.Service('gps_converter', GpsConverter, convert)
    rospy.spin()
