#!/usr/bin/env python3

# =======================================================================================
# File: my_robot_driver.py
# Description: Custom ROS 2 plugin for Webots robot control (Hybrid Node Version).
#              This class inherits from rclpy.node.Node, making it a self-sufficient
#              ROS 2 node. This provides a stable API (self.create_publisher, etc.)
#              regardless of the webots_ros2_driver version.
# =======================================================================================

# Standard library imports
import sys
import threading
import tty
import termios
import cv2
import numpy as np

# ROS and CV Bridge imports
import rclpy
from rclpy.node import Node  # <--- IMPORTANT: Import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Local application/library specific imports
# try:
#     from .my_model import predict_image
#     from .abstacle_avoidance_algorithm import getControlPoint
# except ImportError:
#     from my_model import predict_image
#     from abstacle_avoidance_algorithm import getControlPoint

# --- Constants ---
LINEAR_SPEED_FORWARD = 0.22
LINEAR_SPEED_BACKWARD = 0.15
ANGULAR_SPEED = 1.0

# The class now inherits from rclpy.node.Node
class MyRobotDriver:
    __counter = 0
    def init(self, webots_node, properties):
        """
        Initializes the robot driver plugin.
        """
        # FIRST STEP: Initialize as a ROS 2 Node. This gives us all ROS functionalities.
        # super().__init__('my_robot_driver_node')

        # We still need the webots_node to get the robot instance
        self.__robot = webots_node.robot
        self.__TIMESTEP = int(self.__robot.getBasicTimeStep())
        self.camLeft = self.__robot.getDevice("camera_right")
        self.camLeft.enable(32)
        
    #     # Now we can use the proper logger!
    #     self.get_logger().info("Initializing custom robot driver 'MyRobotDriver' (Hybrid Node)...")

    #     # --- State Variables ---
    #     self.manual_control = False
    #     self.manual_key_pressed = None
    #     self.last_direction = None
    #     self.oscillate_count = 0
    #     self.stuck_count = 0
    #     self.latest_left_image = None
    #     self.latest_right_image = None
        
    #     # --- ROS 2 Publishers and Subscribers ---
    #     # Now we create ROS objects using `self` because `self` is a Node.
    #     self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    #     self.left_cam_subscriber = self.create_subscription(
    #         Image, '/camera_left/image_raw', self.left_image_callback, 10)
        
    #     self.right_cam_subscriber = self.create_subscription(
    #         Image, '/camera_right/image_raw', self.right_image_callback, 10)

    #     self.bridge = CvBridge()

    #     self.key_thread = threading.Thread(target=self.keyboard_listener)
    #     self.key_thread.daemon = True
    #     self.key_thread.start()
        
    #     self.get_logger().info("'MyRobotDriver' initialization complete.")

    # def keyboard_listener(self):
    #     """Listens for keyboard presses."""
    #     self.get_logger().info("Keyboard listener started. Press 'm' to toggle, 'w/a/s/d' for manual.")
    #     old_settings = termios.tcgetattr(sys.stdin)
    #     try:
    #         tty.setcbreak(sys.stdin.fileno())
    #         # Use rclpy.ok() as we are a full node
    #         while rclpy.ok():
    #             key = sys.stdin.read(1)
    #             if key:
    #                 if key.lower() == 'm':
    #                     self.manual_control = not self.manual_control
    #                     mode = "MANUAL" if self.manual_control else "OBSTACLE AVOIDANCE"
    #                     self.get_logger().info(f"Switched to {mode} control mode.")
    #                 elif self.manual_control:
    #                     self.manual_key_pressed = key.lower()
    #                 if not self.manual_control:
    #                     self.manual_key_pressed = None
    #     finally:
    #         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    # def left_image_callback(self, msg: Image):
    #     try:
    #         self.latest_left_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to convert left image: {e}")

    # def right_image_callback(self, msg: Image):
    #     try:
    #         self.latest_right_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    #     except Exception as e:
    #         self.get_logger().error(f"Failed to convert right image: {e}")

    # def handle_manual_control(self) -> Twist:
    #     twist_msg = Twist()
    #     if self.manual_key_pressed == 'w': twist_msg.linear.x = LINEAR_SPEED_FORWARD
    #     elif self.manual_key_pressed == 's': twist_msg.linear.x = -LINEAR_SPEED_BACKWARD
    #     elif self.manual_key_pressed == 'a': twist_msg.angular.z = ANGULAR_SPEED
    #     elif self.manual_key_pressed == 'd': twist_msg.angular.z = -ANGULAR_SPEED
    #     self.manual_key_pressed = None
    #     return twist_msg

    # def handle_obstacle_avoidance(self) -> Twist:
    #     twist_msg = Twist()
    #     if self.latest_left_image is None or self.latest_right_image is None:
    #         self.get_logger().warn("Waiting for camera images...", throttle_duration_sec=3)
    #         return twist_msg

    #     cam_image_left_rgb = cv2.cvtColor(self.latest_left_image, cv2.COLOR_BGR2RGB)
    #     cam_image_right_rgb = cv2.cvtColor(self.latest_right_image, cv2.COLOR_BGR2RGB)

    #     seg_image_left = predict_image(None, cam_image_left_rgb, False)
    #     seg_image_right = predict_image(None, cam_image_right_rgb, False)

    #     row_line_ok = 180
    #     row_line_back = 200
    #     range_accept = 10
    #     image_middle_col = int(seg_image_left.shape[1] / 2)

    #     cp_dot_left, _ = getControlPoint(seg_image_left, row_line_ok)
    #     cp_dot_right, _ = getControlPoint(seg_image_right, row_line_ok)
        
    #     cp_dot = None
    #     if cp_dot_left:
    #         cp_dot = cp_dot_left
    #         if cp_dot_right and abs(cp_dot_left[0] - cp_dot_right[0]) < 30:
    #             cp_dot = max(cp_dot_left, cp_dot_right, key=lambda x: x[0])

    #     if cp_dot:
    #         cp_x, cp_y = cp_dot[1][0], cp_dot[1][1]
    #         if self.stuck_count > 6:
    #             self.stuck_count = 0; twist_msg.linear.x = -LINEAR_SPEED_BACKWARD
    #         elif self.oscillate_count > 4:
    #             self.oscillate_count = 0; twist_msg.angular.z = -ANGULAR_SPEED
    #         elif cp_y > row_line_back:
    #             self.last_direction = "back"; twist_msg.linear.x = -LINEAR_SPEED_BACKWARD * 0.5; twist_msg.angular.z = -ANGULAR_SPEED
    #         elif cp_x < (image_middle_col - range_accept):
    #             if self.last_direction == "right": self.oscillate_count += 1
    #             elif self.last_direction == "left": self.stuck_count += 1
    #             else: self.stuck_count = 0; self.oscillate_count = 0
    #             self.last_direction = "left"; twist_msg.angular.z = ANGULAR_SPEED
    #         elif cp_x > (image_middle_col + range_accept):
    #             if self.last_direction == "left": self.oscillate_count += 1
    #             elif self.last_direction == "right": self.stuck_count += 1
    #             else: self.stuck_count = 0; self.oscillate_count = 0
    #             self.last_direction = "right"; twist_msg.angular.z = -ANGULAR_SPEED
    #         else:
    #             self.last_direction = "front"; self.stuck_count = 0; self.oscillate_count = 0; twist_msg.linear.x = LINEAR_SPEED_FORWARD
    #     else:
    #         self.last_direction = "back"; twist_msg.angular.z = -ANGULAR_SPEED

    #     return twist_msg

    def step(self):
        print("counter:", self.__counter, flush=True)
        self.__counter+=1
        pass
        # The main logic in step remains the same.
        # if self.manual_control:
        #     twist_command = self.handle_manual_control()
        # else:
        #     twist_command = self.handle_obstacle_avoidance()
        
        # self.cmd_vel_publisher.publish(twist_command)