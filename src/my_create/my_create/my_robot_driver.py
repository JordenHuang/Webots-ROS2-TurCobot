#!/usr/bin/env python3

# ... (其他 import 保持不變) ...
import rclpy
import numpy as np
import cv2
from rclpy.executors import SingleThreadedExecutor # <--- 匯入 Executor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

from my_create.vision_node import VisionProcessor

# ... (常數保持不變) ...

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.timestep = int(self.robot.getBasicTimeStep())

        self.camLeft = self.robot.getDevice("camera_left")
        self.camLeft.enable(self.timestep)
        
        self.camRight = self.robot.getDevice("camera_right")
        self.camRight.enable(self.timestep)

        # 1. 初始化 rclpy (只需要一次)
        rclpy.init(args=None)
        
        # 2. 創建你的兩個節點
        self.my_create_node = rclpy.create_node("my_create_node")
        self.vision_node = VisionProcessor() # 這個節點內部名為 'vision_processor'

        # 3. 創建一個 Executor，並將兩個節點都加進去
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.my_create_node)
        self.executor.add_node(self.vision_node)

        # 4. 在 my_create_node 上創建發布者
        self.leftImage_pub = self.my_create_node.create_publisher(Image, "/Left_camera/image", qos_profile_sensor_data)
        self.rightImage_pub = self.my_create_node.create_publisher(Image, "/Right_camera/image", qos_profile_sensor_data)
        
        self.bridge = CvBridge()

        self.my_create_node.get_logger().info("✅ MyRobotDriver with two nodes and executor initialized.")


    def step(self):
        # 發布影像的邏輯保持不變
        #Left_Camera
        img1 = self.camLeft.getImage()
        if img1:
            w1, h1 = self.camLeft.getWidth(), self.camLeft.getHeight()
            np_img1 = np.frombuffer(img1, dtype=np.uint8).reshape((h1, w1, 4))
            bgr1 = cv2.cvtColor(np_img1, cv2.COLOR_BGRA2BGR)
            msg1 = self.bridge.cv2_to_imgmsg(bgr1, encoding="bgr8")
            msg1.header.stamp = self.my_create_node.get_clock().now().to_msg()
            self.leftImage_pub.publish(msg1)

        #Right_Camera
        img2 = self.camRight.getImage()
        if img2:
            w2, h2 = self.camRight.getWidth(), self.camRight.getHeight()
            np_img2 = np.frombuffer(img2, dtype=np.uint8).reshape((h2, w2, 4))
            bgr2 = cv2.cvtColor(np_img2, cv2.COLOR_BGRA2BGR)
            msg2 = self.bridge.cv2_to_imgmsg(bgr2, encoding="bgr8")
            msg2.header.stamp = self.my_create_node.get_clock().now().to_msg()
            self.rightImage_pub.publish(msg2)

        # 在每個 Webots 步驟中，對 Executor 進行 spin_once
        # 這會處理所有已添加節點 (my_create_node 和 vision_node) 的待辦事件
        self.executor.spin_once(timeout_sec=0.001)