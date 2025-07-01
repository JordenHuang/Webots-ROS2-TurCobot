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
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node  # <--- IMPORTANT: Import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from my_create.vision_node import VisionProcessor

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
    # __counter = 0
    def init(self, webots_node, properties):
        """
        Initializes the robot driver plugin.
        """
        # FIRST STEP: Initialize as a ROS 2 Node. This gives us all ROS functionalities.
        # super().init(webots_node, properties)

        # We still need the webots_node to get the robot instance
        self.robot = webots_node.robot
        # self.node = webots_node

        # get the time step of the current world.
        self.timestep = int(self.robot.getBasicTimeStep())

        self.camLeft = self.robot.getDevice("camera_left")
        self.camLeft.enable(self.timestep)
        
        self.camRight = self.robot.getDevice("camera_right")
        self.camRight.enable(self.timestep)

        rclpy.init()
        self.node = rclpy.create_node("my_create_node")
        
        self.leftImage_pub = self.node.create_publisher(Image, "/Left_camera/image", qos_profile_sensor_data)
        self.rightImage_pub = self.node.create_publisher(Image, "/Right_camera/image", qos_profile_sensor_data)
        
        
        #建立轉換器物件
        self.bridge = CvBridge()


        self.vision = VisionProcessor()
        
    

    def step(self):
        # print("counter:", self.__counter, flush=True)
        # self.__counter+=1
        # pass

        #Left_Camera
        img1 = self.camLeft.getImage()
        if img1:
            w1, h1 = self.camLeft.getWidth(), self.camLeft.getHeight()
            np_img1 = np.frombuffer(img1, dtype=np.uint8).reshape((h1, w1, 4))
            bgr1 = cv2.cvtColor(np_img1, cv2.COLOR_BGRA2BGR)
            msg1 = self.bridge.cv2_to_imgmsg(bgr1, encoding="bgr8")
            msg1.header.stamp = self.node.get_clock().now().to_msg()
            self.leftImage_pub.publish(msg1)

        #Right_Camera
        img2 = self.camRight.getImage()
        if img2:
            w2, h2 = self.camRight.getWidth(), self.camRight.getHeight()
            np_img2 = np.frombuffer(img2, dtype=np.uint8).reshape((h2, w2, 4))
            bgr2 = cv2.cvtColor(np_img2, cv2.COLOR_BGRA2BGR)
            msg2 = self.bridge.cv2_to_imgmsg(bgr2, encoding="bgr8")
            msg2.header.stamp = self.node.get_clock().now().to_msg()
            self.rightImage_pub.publish(msg2)

        rclpy.spin_once(self.node, timeout_sec=0.001)