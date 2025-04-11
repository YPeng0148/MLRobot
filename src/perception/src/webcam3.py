#!/usr/bin/env python3

import cv2
import time
import numpy as np
import rospy
from std_msgs.msg import String

print(cv2.__version__)

print("files imported")
# Initialize ROS node and publisher
rospy.init_node('image_processing', anonymous=True)
pub_obstacle = rospy.Publisher('image_info_obstacle', String, queue_size=10)
pub_lane = rospy.Publisher('image_info_lane', String, queue_size=10)

obstacle_count = 0
start_time = time.time()


def main():
    global start_time, obstacle_count

    # Define a video capture object
    vid = cv2.VideoCapture(0)
    rospy.loginfo("VideoCapture defined successfully.")

    while not rospy.is_shutdown():
        # Capture the video frame
        # by frame
        ret, frame = vid.read()
        #print("********* 1. Camera data read successfully *************")
        #print("********* 2. Lane detection in progress ... **********")
        try:
            lane_frame = detect_pedestrian_lane(frame)
            frame = lane_frame
        except e:
            print(e)
        #print("********* Lane detection done... **********")
        #print("********* 3. Obstacle detection in progress ... ********")
        try:
            obstacle_frame = detect_obstacles(frame)
            frame = obstacle_frame
        except e:
            print(e)
        #frame = detect_obstacles(frame, 1000, 2000)
        #print("********* Obstacle detection done ********")

        # Display the resulting frame
        cv2.imshow('frame', frame)
        
        # Increase the accuracy of obstacle detection by resetting the count every 3 seconds
        current_time = time.time()
        if current_time >= start_time + 1:
            obstacle_count = 0
            start_time = current_time
        
        # the 'q' button is set as the
        # quitting button you may use any
        # desired button of your choice
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # After the loop release the cap object
    vid.release()

    # Destroy all the windows
    cv2.destroyAllWindows()

def detect_pedestrian_lane(image):
    global pub_lane
    output = image
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # cloudy day, no shadows:
    #lower_pavement = np.array([0, 0, 160])
    #upper_pavement = np.array([90, 40, 210])

    # sunny day:
    #lower_pavement = np.array([0, 9, 60])
    #upper_pavement = np.array([45, 30, 255])
    
    # semi cloudy day
    lower_pavement = np.array([0, 0, 115])
    upper_pavement = np.array([179, 29, 255])

    mask = cv2.inRange(hsv, lower_pavement, upper_pavement)
    
    #define kernel size  
    kernel = np.ones((7,7),np.uint8)

    # Remove unnecessary noise from mask
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    
    # only the pavement:
    segmented_image = cv2.bitwise_and(image, image, mask=mask)
    
    # Find contours from the mask
    contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_contour = [] 
    max_area = 0
    max_val = 0
    x = y = w = h = None
    # find max contour
    for contour in contours:
        #print(contour.shape)
        current_area = cv2.contourArea(contour)
        if (current_area > max_area):
            max_area = current_area
            max_val = contour
            x, y, w, h = cv2.boundingRect(max_val)
            

    # if there are actually contours
    if (max_area > 0):
        max_contour = [max_val]
        output = cv2.drawContours(image, max_contour, 0, (255, 0, 0), 2)
        if(max_area > 190000):
            print("max contour area", max_area)
            # If no obstacle is detected, write "CLEAR" on the frame
            #rospy.loginfo("*** CLEAR!")
            pub_lane.publish("LANE CLEAR")
            cv2.putText(image, "LANE CLEAR", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
        else:
            pub_lane.publish("LANE UNCLEAR")
    else:
        pub_lane.publish("LANE UNCLEAR")


    if(w is not None):
        pts = np.float32([[x+w*.45, y+h*.01],[x + w *.55, y + h*.01],[x + w, y + h],[x, y + h]])
    # Draw rectangle around the lane
    #output = cv2.rectangle(image, (x,y), (x+w, y+h), (0, 255, 0), 2)
    # Draw green lane
    # output = cv2.fillPoly(image, [pts.astype(int)], (0,255,0))

    return output

def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)
    return mask

def detect_obstacles(image,min_area=100, max_area=500):
    global obstacle_count, pub_obstacle

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Define area of interest triangle/lane
    height = image.shape[0]
    width = image.shape[1]
    region_of_interest_vertices = [
        (0, height),
        (width/2, height/3),
        (width, height),
    ]
    
    mask_triangle = region_of_interest(gray, np.array([region_of_interest_vertices], np.int32))
    masked_image = cv2.bitwise_and(gray, mask_triangle)
    blur = cv2.GaussianBlur(masked_image, (5, 5), 0)

    #blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    # Flip the image vertically
    # gray = cv2.flip(gray, 1)
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if(len(contours) > 0):
        # succeeds
        for cnt in contours:
            # Approximate the contour with a polygon
            epsilon = 0.1*cv2.arcLength(cnt,True)
            approx = cv2.approxPolyDP(cnt,epsilon,True)        
            if len(approx) > 3 and (cv2.contourArea(cnt) > min_area) and cv2.contourArea(cnt) < max_area:
                # Draw a red rectangle around a potential obstacle
                x, y, w, h = cv2.boundingRect(cnt)
                output = cv2.drawContours(masked_image, cnt, 0, (255, 0, 0), 2)
                cv2.imshow("Obstacle segmentation", output)

                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 0, 255), 2)
                # Increase the accuracy of obstacle detection by tracking the count every 3 seconds
                obstacle_count += 1
                # Confirm if the count is more than a threshold error value

                if obstacle_count > 2:
                    obstacle_count = 0
                    rospy.loginfo("*** Obstacle detected! *** Area = %.3f", cv2.contourArea(cnt))
                    # If obstacles are detected, publish "STOP" message
                    pub_obstacle.publish("STOP")
                    cv2.putText(image, "STOP", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 2)
                else:
                    pub_obstacle.publish("CLEAR")
            # Detect stairs and ignore small contours
            # Check if the polygon is a rectangle with 4 vertices
            if len(approx) == 4 and cv2.contourArea(cnt) > max_area:
                x, y, w, h = cv2.boundingRect(cnt)
                aspect_ratio = float(w)/h
                #print(aspect_ratio)
                # Check if the rectangle is roughly the size and shape of stairs
                if aspect_ratio > 2 and aspect_ratio < 7:
                    # Draw a yellow rectangle around potential stairs
                    cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 255), 2)
                    # Increase the accuracy of obstacle detection by tracking the count every 3 seconds
                    obstacle_count += 1
                    # Confirm if the count is more than a threshold error value
                    if obstacle_count > 5:
                        obstacle_count = 0
                        # If stairs are detected, write "CLEAR" on the frame
                        pub_obstacle.publish("STAIRS")
                        rospy.loginfo("*** Stairs detected!")
            # else:
    else:
        rospy.loginfo("*** No obstacles detected ***")

    return image


print("running main")
main()
