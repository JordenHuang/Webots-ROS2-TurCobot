import rclpy
from rclpy.node import Node
import threading

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.qos import qos_profile_sensor_data

# 假設這些是您的自訂模組
from my_create.my_model import predict_image
from my_create.abstacle_avoidance_algorithm import getControlPoint

class VisionProcessor(Node):
    def __init__(self):
        super().__init__('vision_processor')

        self.bridge = CvBridge()

        # 使用 None 來標記是否已收到影像
        self.left_image = None
        self.right_image = None
        
        # 使用一個鎖來防止資料競爭，雖然在單線程執行器中不是絕對必要，但這是一個好習慣
        self.lock = threading.Lock() 

        # 訂閱原始左右影像
        self.create_subscription(Image, "/Left_camera/image", self.left_callback, qos_profile_sensor_data)
        self.create_subscription(Image, "/Right_camera/image", self.right_callback, qos_profile_sensor_data)

        # 發布遮罩影像（給 rqt_image_view 用）
        self.left_mask_pub = self.create_publisher(Image, "/Left_camera/mask", qos_profile_sensor_data)
        self.right_mask_pub = self.create_publisher(Image, "/Right_camera/mask", qos_profile_sensor_data)

        self.get_logger().info("✅ VisionProcessor initialized")

    def left_callback(self, msg):
        # self.get_logger().info("✅left_callback successed")
        try:
            # 回呼函式的唯一職責：儲存影像並嘗試處理
            with self.lock:
                self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.process_images_if_ready()
        except Exception as e:
            self.get_logger().error(f"❌ Failed in left_callback: {e}")

    def right_callback(self, msg):
        try:
            # 回呼函式的唯一職責：儲存影像並嘗試處理
            with self.lock:
                self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.process_images_if_ready()
        except Exception as e:
            self.get_logger().error(f"❌ Failed in right_callback: {e}")

    def process_images_if_ready(self):
        # self.get_logger().info("--- Checking if images are ready to process ---")

        # 使用 with self.lock 來確保線程安全
        with self.lock:
            # 檢查是否收到了左右兩張影像
            if self.left_image is None or self.right_image is None:
                return # 如果不齊全，直接返回

            # --- 將所有處理和發布邏輯集中到這裡 ---

            # 複製影像，避免在處理時被新的回呼覆蓋
            left_image_copy = self.left_image.copy()
            right_image_copy = self.right_image.copy()

            # 將成員變數設為 None，表示這對影像已經被「消費」，等待下一對新影像
            self.left_image = None
            self.right_image = None
        
        # --- 在鎖之外進行耗時的計算 ---

        # 1. 產生遮罩 (Mask Generation)
        mask_left = predict_image(image_array=left_image_copy, show=False)
        mask_right = predict_image(image_array=right_image_copy, show=False)
        
        # 2. 發布遮罩影像 (Publish Masks)
        try:
            now = self.get_clock().now().to_msg()
            
            mask_msg_left = self.bridge.cv2_to_imgmsg(mask_left, encoding="mono8")
            mask_msg_left.header.stamp = now
            mask_msg_left.header.frame_id = "left_camera_link" # 建議設定 frame_id
            self.left_mask_pub.publish(mask_msg_left)

            mask_msg_right = self.bridge.cv2_to_imgmsg(mask_right, encoding="mono8")
            mask_msg_right.header.stamp = now
            mask_msg_right.header.frame_id = "right_camera_link" # 建議設定 frame_id
            self.right_mask_pub.publish(mask_msg_right)
            
            # self.get_logger().info("Published left and right masks.") # 可用於除錯

        except Exception as e:
            self.get_logger().error(f"❌ Failed to publish masks: {e}")


        # 3. 進行避障演算法計算 (Obstacle Avoidance Logic)
        row_line_front = 180
        cp_dot_left, middle_dots_left = getControlPoint(mask_left, row_line_front)
        cp_dot_right, middle_dots_right = getControlPoint(mask_right, row_line_front)

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

        # 這部分視覺化可以保留，但它不會被發布出去，只在節點運行時的日誌中顯示
        if cp_dot:
            self.get_logger().info(f"Control Point: {cp_dot[1]}, Distance from center: {cp_dot[0]}")
        else:
            self.get_logger().info("No control point detected")


def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionProcessor()
    
    # 為了處理 callback，建議使用多線程執行器
    # from rclpy.executors import MultiThreadedExecutor
    # executor = MultiThreadedExecutor()
    # executor.add_node(vision_node)
    
    try:
        # executor.spin()
        rclpy.spin(vision_node) # 單線程執行器在這種情況下通常也夠用
    except KeyboardInterrupt:
        pass
    finally:
        vision_node.destroy_node()
        rclpy.shutdown()
        # cv2.destroyAllWindows() # 如果 predict_image 或其他地方有 cv2.imshow，才需要這行

if __name__ == "__main__":
    main()