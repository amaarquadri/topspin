#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

class BallTracker:

    def __init__(self):
        # Initialize the CvBridge class
        self.bridge = CvBridge()

        rospy.init_node('orange_detector', anonymous=True)

        rospy.Subscriber("/rgb/image_raw", Image, self.image_callback)
        rospy.Subscriber("/depth_to_rgb/image_raw",Image, self.depth_callback)

        self.pose_pub = rospy.Publisher('/ball_pose', PoseStamped, queue_size=10)

        self.ball_center = None
        self.x, self.y, self.w, self.h = None, None, None, None

    def image_callback(self, data):

        try:
            # Convert the ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Convert the image from BGR to HSV
        # hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hsv_image = cv_image

        # Define the range of orange color in HSV
        # These values can be adjusted depending on the specific shade of orange you are looking for
        lower_orange = (60, 120, 230)
        upper_orange = (80, 150, 265)

        # Threshold the HSV image to get only orange colors
        mask = cv2.inRange(hsv_image, lower_orange, upper_orange)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Find the largest contour and draw a bounding box around it
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            self.x, self.y, self.w, self.h = x, y, w, h
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            self.ball_center = (x + w / 2, y + h / 2)

        else:
            self.ball_center = None

    def depth_callback(self, data):
        if not self.ball_center:
            return
        
        cv_image = self.bridge.imgmsg_to_cv2(data, '16UC1')

        # cv2.rectangle(cv_image, (self.x, self.y), (self.x+self.w, self.y+self.h), (0, 255, 0), 2)
        # cv2.imshow('Detected Orange Region', cv_image)
        # cv2.waitKey(3)
        x, y = self.ball_center[1], self.ball_center[0]
        z = cv_image[int(x), int(y)]
        
        ball_pose = PoseStamped()

        ball_pose.header.seq = 1
        ball_pose.header.stamp = rospy.Time.now()
        # TODO find correct frame
        ball_pose.header.frame_id = "camera_body"

        ball_pose.pose.position.x = x / 1000.0
        ball_pose.pose.position.y = y / 1000.0
        ball_pose.pose.position.z = z / 1000.0

        ball_pose.pose.orientation.x = 0.0
        ball_pose.pose.orientation.y = 0.0
        ball_pose.pose.orientation.z = 0.0
        ball_pose.pose.orientation.w = 1.0

        self.pose_pub.publish(ball_pose)
        


if __name__ == '__main__':
    bt = BallTracker()

    # Spin until Ctrl+C is pressed
    rospy.spin()

    cv2.destroyAllWindows()
