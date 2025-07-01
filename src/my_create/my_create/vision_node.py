#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.qos import qos_profile_sensor_data
from my_create.my_model import predict_image  # 你的語意分割模型處理函式
from my_create.abstacle_avoidance_algorithm import getControlPoint  # 控制點演算法（若要畫點）

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')
        # rclpy.init()
        # self.node = rclpy.create_node("vision_processor_node")

        # self.node = node

        self.bridge = CvBridge()

        self.left_image = None
        self.right_image = None

        # 建立 ROS2 訂閱者
        self.create_subscription(Image, "/Left_camera/image", self.left_callback, qos_profile_sensor_data)
        self.create_subscription(Image, "/Right_camera/image", self.right_callback, qos_profile_sensor_data)
        
        self.get_logger().info("✅ VisionProcessor initialized")

    def left_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.left_image = cv_image
            self.process_images_if_ready()
            result_mask = predict_image(image_array=cv_image, show=False)

            # 若要顯示
            cv2.imshow("Left Camera - Prediction", result_mask)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ Failed to process left image: {e}")

    def right_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.right_image = cv_image
            self.process_images_if_ready()
            result_mask = predict_image(image_array=cv_image, show=False)

            # 若要顯示
            cv2.imshow("Right Camera - Prediction", result_mask)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"❌ Failed to process right image: {e}")

    def process_images_if_ready(self):
        if self.left_image is None or self.right_image is None:
            return
        
        # 語意分割
        mask_left = predict_image(image_array=self.left_image, show=False)
        mask_right = predict_image(image_array=self.right_image, show=False)

        # 計算控制點
        row_line_front = 180
        cp_dot_left, middle_dots_left = getControlPoint(mask_left, row_line_front)
        cp_dot_right, middle_dots_right = getControlPoint(mask_right, row_line_front)

        # 融合控制點，示範以距離為判斷
        cp_dot = None
        if cp_dot_left and cp_dot_right:
            if abs(cp_dot_left[1][0] - cp_dot_right[1][0]) < 30:
                cp_dot = cp_dot_left if cp_dot_left[1][0] > cp_dot_right[1][0] else cp_dot_right
            else:
                cp_dot = cp_dot_left
        elif cp_dot_left:
            cp_dot = cp_dot_left
        elif cp_dot_right:
            cp_dot = cp_dot_right

        # 視覺化左側遮罩與控制點
        vis_image = cv2.cvtColor(mask_left, cv2.COLOR_GRAY2BGR)
        for dot in middle_dots_left:
            cv2.circle(vis_image, dot, 2, (0, 255, 0), -1)
        if cp_dot:
            cv2.circle(vis_image, cp_dot[1], 4, (255, 0, 255), -1)
            self.get_logger().info(f"Control Point: {cp_dot[1]}, Distance from center: {cp_dot[0]}")
        else:
            self.get_logger().info("No control point detected")

        cv2.imshow("Control Point Visualization", vis_image)
        cv2.waitKey(1)

        # 處理完後清除
        self.left_image = None
        self.right_image = None

def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionProcessor()
    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        pass
    vision_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
