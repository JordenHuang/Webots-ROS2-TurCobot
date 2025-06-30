#!/usr/bin/env python3
import rclpy
# from webots_ros2_driver.webots_node import WebotsNode
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import sys
import threading
import tty
import termios

# Assuming these files are in the same ROS2 package or Python path
from .my_model import predict_image
from .abstacle_avoidance_algorithm import getControlPoint

# --- Constants ---
# It's good practice to use uppercase for constants
LINEAR_SPEED_FORWARD = 0.22  # m/s, TurtleBot3 Burger's max linear speed
LINEAR_SPEED_BACKWARD = 0.15 # m/s
ANGULAR_SPEED = 1.0     # rad/s, a moderate turning speed

class MyRobotDriver:
    def __init__(self, webots_node, properties):
        print("MyRobotDirver init ================================================================================", flush=True)
        # super().__init__('my_robot_driver')
        self.__robot = webots_node.robot
        self.__TIMESTEP = int(self.__robot.getBasicTimeStep())
        self.get_logger().info("Robot Driver Node has been started.")
        '''
        # --- State Variables ---
        self.manual_control = False
        self.manual_key_pressed = None
        self.last_direction = None
        self.oscillate_count = 0
        self.stuck_count = 0
        self.latest_left_image = None
        self.latest_right_image = None
        
        # Flag to prevent spamming mode switch message
        self._mode_switch_flag = False

        # --- ROS2 Publishers and Subscribers ---
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.left_cam_subscriber = self.create_subscription(
            Image,
            'camera_left/image_raw',  # Adjust topic name if necessary
            self.left_image_callback,
            10)
        
        self.right_cam_subscriber = self.create_subscription(
            Image,
            'camera_right/image_raw', # Adjust topic name if necessary
            self.right_image_callback,
            10)

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Main control loop timer
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Start keyboard listener in a separate thread
        self.key_thread = threading.Thread(target=self.keyboard_listener)
        self.key_thread.daemon = True
        self.key_thread.start()

    def keyboard_listener(self):
        """Listens for keyboard presses to toggle mode and for manual control."""
        self.get_logger().info("Keyboard listener started. Press 'm' to toggle mode, 'w/a/s/d' for manual control.")
        # Save terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        try:
            # Set terminal to raw mode
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                key = sys.stdin.read(1)
                if key:
                    if key.lower() == 'm':
                        self.manual_control = not self.manual_control
                        if self.manual_control:
                            self.get_logger().info("Switched to MANUAL control mode.")
                        else:
                            self.get_logger().info("Switched to OBSTACLE AVOIDANCE mode.")
                    elif self.manual_control:
                        self.manual_key_pressed = key.lower()
                    
                    # Reset the key after processing to avoid continuous movement
                    if not self.manual_control:
                        self.manual_key_pressed = None
        finally:
            # Restore terminal settings on exit
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def left_image_callback(self, msg):
        """Callback for the left camera image subscriber."""
        try:
            # Convert ROS Image message to OpenCV image (BGR8 format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_left_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Failed to convert left image: {e}")

    def right_image_callback(self, msg):
        """Callback for the right camera image subscriber."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_right_image = cv_image
        except Exception as e:
            self.get_logger().error(f"Failed to convert right image: {e}")

    def timer_callback(self):
        """Main control loop, executed at a fixed frequency."""
        if self.manual_control:
            self.handle_manual_control()
        else:
            self.handle_obstacle_avoidance()
        
        # After one cycle of manual key press, stop the robot if no new key is pressed
        if self.manual_key_pressed:
             self.manual_key_pressed = None

    def handle_manual_control(self):
        """Publishes Twist commands based on keyboard input."""
        twist_msg = Twist()
        if self.manual_key_pressed == 'w':
            twist_msg.linear.x = LINEAR_SPEED_FORWARD
        elif self.manual_key_pressed == 's':
            twist_msg.linear.x = -LINEAR_SPEED_BACKWARD
        elif self.manual_key_pressed == 'a':
            twist_msg.angular.z = ANGULAR_SPEED
        elif self.manual_key_pressed == 'd':
            twist_msg.angular.z = -ANGULAR_SPEED
        else:
            # Stop the robot if no key is being pressed
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(twist_msg)

    def handle_obstacle_avoidance(self):
        """Implements the vision-based obstacle avoidance logic."""
        if self.latest_left_image is None or self.latest_right_image is None:
            self.get_logger().warn("Waiting for camera images...")
            return

        cam_image_left = cv2.cvtColor(self.latest_left_image, cv2.COLOR_BGR2RGB)
        cam_image_right = cv2.cvtColor(self.latest_right_image, cv2.COLOR_BGR2RGB)

        # --- Vision Processing ---
        seg_image_left = predict_image(None, cam_image_left, False)
        seg_image_right = predict_image(None, cam_image_right, False)

        # Some parameters from the original script
        row_line_ok = 180
        row_line_back = 200
        range_accept = 10
        image_middle_col = int(seg_image_left.shape[1] / 2)

        # Find the controlling point
        cp_dot_left, _ = getControlPoint(seg_image_left, row_line_ok)
        cp_dot_right, _ = getControlPoint(seg_image_right, row_line_ok)
        
        cp_dot = None
        if cp_dot_left:
            if cp_dot_right and abs(cp_dot_left[0] - cp_dot_right[0]) < 30:
                # If both are valid and close, take the one further away
                cp_dot = max(cp_dot_left, cp_dot_right, key=lambda x: x[0])
            else:
                cp_dot = cp_dot_left # Default to left if right is noisy or absent

        # --- Control Logic ---
        twist_msg = Twist()
        if cp_dot:
            cp_x = cp_dot[1][0]
            cp_y = cp_dot[1][1]

            # Stuck detection
            if self.stuck_count > 6:
                self.get_logger().info("Stuck detected. Moving backward.")
                self.stuck_count = 0
                twist_msg.linear.x = -LINEAR_SPEED_BACKWARD
            # Oscillate detection
            elif self.oscillate_count > 4:
                self.get_logger().info("Oscillation detected. Turning right.")
                self.oscillate_count = 0
                twist_msg.angular.z = -ANGULAR_SPEED  # Turn right
            # Obstacle is too close, should go back
            elif cp_y > row_line_back:
                self.get_logger().info("Obstacle too close. Turning away.")
                self.last_direction = "back"
                twist_msg.linear.x = -LINEAR_SPEED_BACKWARD * 0.5 # Move back slowly
                twist_msg.angular.z = -ANGULAR_SPEED # And turn away (right)
            # Path is clear to the left
            elif cp_x < (image_middle_col - range_accept):
                self.get_logger().info("Path clear to the left. Turning left.")
                if self.last_direction == "right": self.oscillate_count += 1
                elif self.last_direction == "left": self.stuck_count += 1
                else: self.stuck_count = 0; self.oscillate_count = 0
                self.last_direction = "left"
                twist_msg.angular.z = ANGULAR_SPEED
            # Path is clear to the right
            elif cp_x > (image_middle_col + range_accept):
                self.get_logger().info("Path clear to the right. Turning right.")
                if self.last_direction == "left": self.oscillate_count += 1
                elif self.last_direction == "right": self.stuck_count += 1
                else: self.stuck_count = 0; self.oscillate_count = 0
                self.last_direction = "right"
                twist_msg.angular.z = -ANGULAR_SPEED
            # Path is straight ahead
            else:
                self.get_logger().info("Path is straight. Moving forward.")
                self.last_direction = "front"
                self.stuck_count = 0
                self.oscillate_count = 0
                twist_msg.linear.x = LINEAR_SPEED_FORWARD
        else:
            # No control point found, likely means no clear path. Turn to find one.
            self.get_logger().warn("No control point found. Turning right to search for a path.")
            self.last_direction = "back"
            twist_msg.angular.z = -ANGULAR_SPEED # Turn right by default

        self.cmd_vel_publisher.publish(twist_msg)
        '''
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        

# def main(args=None):
#     print("my_robot_driver started!!!")
#     rclpy.init(args=args)
#     robot_driver_node = MyRobotDriver()
#     try:
#         rclpy.spin(robot_driver_node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         # Stop the robot on shutdown
#         stop_msg = Twist()
#         robot_driver_node.cmd_vel_publisher.publish(stop_msg)
#         robot_driver_node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()