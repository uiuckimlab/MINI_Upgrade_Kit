#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Point
import cv2
import numpy as np
import time

def perception():
    pub = rospy.Publisher('ball_pos', Point, queue_size=10)
    rospy.init_node('perception', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    cap = cv2.VideoCapture(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

    #start time 
    while not rospy.is_shutdown():

        _, frame = cap.read()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        #blue
        mask = cv2.inRange(hsv, (100,50,20), (130,255,255))
        
        #make it black objects on white background 
        mask = cv2.bitwise_not(mask)

        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 10
        params.maxThreshold = 200
    
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 200
        params.maxArea = 15000

        # # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.8

        # Create a detector with the parameters
        detector = cv2.SimpleBlobDetector_create(params)

        # Detect blobs.
        keypoints = detector.detect(mask)

        # Draw detected blobs as red circles.
        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures
        # the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(mask, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        if len(keypoints) > 0 :
            print( len(keypoints), keypoints[0].pt[0], keypoints[0].pt[1], keypoints[0].size)


            ball_pos = Point()
            ball_pos.x = round(keypoints[0].pt[0])
            ball_pos.y = round(keypoints[0].pt[1])
            ball_pos.z = round(keypoints[0].size) #use z coord to send size 
            # hello_str = "hello world %s" % rospy.get_time() % 
            # rospy.loginfo(hello_str)
            pub.publish(ball_pos)
        rate.sleep()

if __name__ == '__main__':
    try:
        perception()
    except rospy.ROSInterruptException:
        pass