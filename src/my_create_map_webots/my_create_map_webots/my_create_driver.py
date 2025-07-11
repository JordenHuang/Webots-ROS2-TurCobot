#!/usr/bin/env python3

# =======================================================================================
# File: my_create_driver.py
# Description: TODO
# =======================================================================================

# Standard library imports
import cv2
import numpy as np

# ROS and CV Bridge imports
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from my_create_map_webots.vision_node import VisionProcessor


# --- Constants ---
LINEAR_SPEED_FORWARD = 0.22
LINEAR_SPEED_BACKWARD = 0.15
ANGULAR_SPEED = 1.0

class MyCreateDriver:
    def init(self, webots_node, properties):
        """
        Initializes the robot driver plugin.
        """

        # We still need the webots_node to get the robot instance
        self.robot = webots_node.robot

        # Get the time step of the current world.
        self.TIMESTEP = int(self.robot.getBasicTimeStep())

        self.camLeft = self.robot.getDevice("camera_left")
        self.camLeft.enable(self.TIMESTEP)

        self.camRight = self.robot.getDevice("camera_right")
        self.camRight.enable(self.TIMESTEP)

        rclpy.init(args=None)
        self.node = rclpy.create_node("my_create_node")

        self.leftImage_pub = self.node.create_publisher(Image, "/Left_camera/image", qos_profile_sensor_data)
        self.rightImage_pub = self.node.create_publisher(Image, "/Right_camera/image", qos_profile_sensor_data)

        #建立轉換器物件
        self.bridge = CvBridge()
        self.node.get_logger().info(f"Driver init done")
        # self.vision = VisionProcessor()

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        # Left camera
        img1 = self.camLeft.getImage()
        if img1:
            w1, h1 = self.camLeft.getWidth(), self.camLeft.getHeight()
            np_img1 = np.frombuffer(img1, dtype=np.uint8).reshape((h1, w1, 4))
            bgr1 = cv2.cvtColor(np_img1, cv2.COLOR_BGRA2BGR)
            msg1 = self.bridge.cv2_to_imgmsg(bgr1, encoding="bgr8")
            msg1.header.stamp = self.node.get_clock().now().to_msg()
            self.leftImage_pub.publish(msg1)

        # # Right camera
        img2 = self.camRight.getImage()
        if img2:
            w2, h2 = self.camRight.getWidth(), self.camRight.getHeight()
            np_img2 = np.frombuffer(img2, dtype=np.uint8).reshape((h2, w2, 4))
            bgr2 = cv2.cvtColor(np_img2, cv2.COLOR_BGRA2BGR)
            msg2 = self.bridge.cv2_to_imgmsg(bgr2, encoding="bgr8")
            msg2.header.stamp = self.node.get_clock().now().to_msg()
            self.rightImage_pub.publish(msg2)

