#!/usr/bin/env python3

import sys
import cv2
import numpy as np
import rclpy
from rclpy.node import Node 
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image 
from geometry_msgs.msg import Twist

class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')	
        self.bridge = CvBridge() 
        self.subscriber = self.create_subscription(Image, '/camera1/image_raw', self.process_data, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        self.velocity = Twist()
        self.empty = True  # Start with no ball detected
        self.error = 0 
        self.action = ""
        self.radius = 0  # Ball radius
        self.get_logger().info("Node Started!")

    def send_cmd_vel(self):
        # Debugging output
        print(f'Empty: {self.empty}, Error: {self.error}, Radius: {self.radius}')  
        
        if self.empty:
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = 0.5
            self.action = "Rotate"
        else:
            # Proportional control for angular velocity based on error
            Kp = 0.009  # Proportional gain for angular velocity (adjust this value)
            self.velocity.angular.z = Kp * self.error
            
            # Adjust linear velocity based on how close the ball is (radius)
            if self.radius > 50:  # Ball is too close
                self.velocity.linear.x = 0.0  # Stop if too close to the ball
                self.action = "Stop, Close to Ball"
            elif abs(self.error) < 20:  # Ball is mostly centered
                self.velocity.linear.x = 0.8  # Slow down as the ball is centered
                self.action = "Go Straight"
            else:
                self.velocity.linear.x = 0.7  # Move forward at a slower speed
                self.action = "Moving"

        # Publish velocity
        self.publisher.publish(self.velocity)

    def process_data(self, data): 
        self.get_logger().info("Image Received!")
        frame = self.bridge.imgmsg_to_cv2(data)
        cv2.imshow("Frame", frame)
        
        # Convert the frame to HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cv2.imshow("HSVframe", hsv_frame)
        # Display the HSV image for debugging
        #cv2.imshow('HSV Frame', hsv_frame)

        # Use your specified HSV range values for ball detection
        light_ball = np.array([115, 100, 100])  # Lower HSV range for ball color
        dark_ball = np.array([125, 260, 260])  # Upper HSV range for ball color

        # Create a mask to detect the ball based on HSV values
        mask = cv2.inRange(hsv_frame, light_ball, dark_ball)

        # Debugging: Check if the mask has any white pixels (non-zero)
        if np.count_nonzero(mask) == 0:
            self.get_logger().info("Mask is empty, no ball detected.")
            self.empty = True
        else:
            self.get_logger().info("Mask created successfully, ball may be detected.")
            self.empty = False

        # Display the mask for debugging
        cv2.imshow('Mask', mask)

        # Use contours to detect the ball instead of edges
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Check if any contours were found
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)

            # Only proceed if the radius is large enough to be a valid ball
            if radius > 1:
                center = (int(x), int(y))
                frame_mid = center[0]  # x-coordinate of the ball's center
                mid_point = frame.shape[1] // 2  # Midpoint of the image width

                # Draw the detected ball
                cv2.circle(frame, center, int(radius), (0, 255, 255), 2)
                
                # Calculate the error between the ball's center and the image midpoint
                self.error = mid_point - frame_mid
                self.radius = radius  # Update radius to determine closeness to ball
                self.empty = False
            else:
                print('Ball radius too small.')
                self.empty = True
        else:
            print('No ball detected.')
            self.empty = True

        # Display the frame with ball tracking for debugging
        cv2.imshow('Ball Tracking', frame)
        cv2.waitKey(1)  # Use a small wait time to allow OpenCV to process the window

def main(args=None):
    rclpy.init(args=args)
    node = BallFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
