#!/usr/bin/env python3

import sys
import cv2
import numpy as np  # Ensure "np" is used instead of "numpy" consistently
import rclpy
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist

class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')	
        self.bridge = CvBridge() 
        self.subscriber = self.create_subscription(Image,'/camera1/image_raw', self.process_data, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        self.velocity = Twist()
        self.empty = False
        self.error = 0 
        self.action = ""
        self.get_logger().info("Node Started!")

    def send_cmd_vel(self):
        if self.empty:
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = 0.0
            self.action = "Stop"
        else:  
            if self.error > 0:
                self.velocity.linear.x = 0.3
                self.velocity.angular.z = 0.2
                self.action = "Go Left"
            elif self.error < 0:
                self.velocity.linear.x = 0.3
                self.velocity.angular.z = -0.2
                self.action = "Go Right"
            elif self.error == 0:
                self.velocity.linear.x = 0.1
                self.velocity.angular.z = 0.0
                self.action = "Go Straight"

        self.publisher.publish(self.velocity)

    ## Subscriber Call Back
    def process_data(self, data): 
        self.get_logger().info("Image Received!")
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")  # Ensure correct format
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # Convert to HSV

        # Updated HSV values for the ball color (double-check that these values make sense)
        lower_color = np.array([115, 95, 95])  # Lower range of the ball color
        upper_color = np.array([125, 260, 260])  # Upper range of the ball color

        mask = cv2.inRange(hsv_frame, lower_color, upper_color)  # Threshold the image
        cv2.imshow('mask', mask)

        # Find contours (for detecting the ball)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # Find the largest contour (assuming it's the ball)
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)

            # Only proceed if the radius is larger than a minimum size
            if radius > 10:
                self.empty = False
                center = (int(x), int(y))
                frame_mid = center[0]
                mid_point = frame.shape[1] // 2

                # Draw the ball and its center
                cv2.circle(frame, center, int(radius), (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)

                self.error = mid_point - frame_mid  # Calculate error as the difference
            else:
                self.empty = True
        else:
            self.empty = True  # If no ball is detected

        cv2.imshow('Ball Tracking', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = BallFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
