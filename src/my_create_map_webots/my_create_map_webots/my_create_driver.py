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
from sensor_msgs.msg import Image, CameraInfo
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
        self._robot = webots_node.robot

        # Get the time step of the current world.
        self._TIMESTEP = int(self._robot.getBasicTimeStep())

        self.camLeft = self._robot.getDevice("camera_left")
        self.camLeft.enable(self._TIMESTEP)

        self.camRight = self._robot.getDevice("camera_right")
        self.camRight.enable(self._TIMESTEP)

        # Both camera (should) have the same width and height
        self.camWidth = self.camLeft.getWidth()
        self.camHeight = self.camLeft.getHeight()
        self.cameraFov = self.camLeft.getFov()

        rclpy.init(args=None)
        self.node = rclpy.create_node("my_create_node")

        # Publishers
        self.camLeftImgPub = self.node.create_publisher(Image, "/camera_left/image_raw", qos_profile_sensor_data)
        self.camRightImgPub = self.node.create_publisher(Image, "/camera_right/image_raw", qos_profile_sensor_data)
        self.camLeftInfoPub = self.node.create_publisher("/camera_left/camera_info", 10)
        self.camRightInfoPub = self.node.create_publisher("/camera_left/camera_info", 10)

        #建立轉換器物件
        self.bridge = CvBridge()
        self.node.get_logger().info(f"Driver init done")

        # Calculate camera intrinsics and extrinsics (ONLY ONCE)
        self._calculate_camera_parameters()

    def step(self):
        shape = (self.camHeight, self.camWidth, 4)
        now = self.node.get_clock().now().to_msg()

        # === Publish camera image of both cameras ===
        # Get image from both camera
        imgRawLeft = self.camLeft.getImage()
        imgRawRight = self.camRight.getImage()

        # --- Convert ---
        # Left
        npImgLeft = np.frombuffer(imgRawLeft, dtype=np.uint8).reshape(shape)
        bgrImgLeft = cv2.cvtColor(npImgLeft, cv2.COLOR_BGRA2BGR)

        # Right
        npImgRight = np.frombuffer(imgRawRight, dtype=np.uint8).reshape(shape)
        bgrImgRight = cv2.cvtColor(npImgRight, cv2.COLOR_BGRA2BGR)

        # --- Create message ---
        leftImgMsg = self.bridge.cv2_to_imgmsg(bgrImgLeft, encoding="bgr8")
        rightImgMsg = self.bridge.cv2_to_imgmsg(bgrImgRight, encoding="bgr8")

        # --- Set fields ---
        leftImgMsg.header.stamp = now
        leftImgMsg.header.frame_id = "camera_left_link" # Frame ID for the left camera
        rightImgMsg.header.stamp = now
        rightImgMsg.header.frame_id = "camera_right_link" # Frame ID for the right camera

        # --- Publish ---
        self.camLeftImgPub.publish(leftImgMsg)
        self.camRightImgPub.publish(rightImgMsg)

        # === Publish camera info ===
        # --- Left CameraInfo ---
        camLeftInfoMsg = CameraInfo()
        camLeftInfoMsg.header.stamp = now
        camLeftInfoMsg.header.frame_id = 'camera_left_link' # Must match image frame_id
        camLeftInfoMsg.width = self.camWidth
        camLeftInfoMsg.height = self.camHeight
        camLeftInfoMsg.distortion_model = "plumb_bob" # Common distortion model (assuming no distortion here)
        camLeftInfoMsg.d = self.D
        camLeftInfoMsg.k = self.K.flatten().tolist() # K must be flattened to a 1D list of 9 elements
        camLeftInfoMsg.r = self.R # R must be flattened to a 1D list of 9 elements
        camLeftInfoMsg.p = self.P_left # P must be flattened to a 1D list of 12 elements

        # --- Right CameraInfo ---
        # (similar to left, but P might differ if it encodes stereo baseline)
        camera_info_right_msg = CameraInfo()
        camera_info_right_msg.header.stamp = now
        camera_info_right_msg.header.frame_id = 'camera_right_link' # Must match image frame_id
        camera_info_right_msg.width = self.camera_width
        camera_info_right_msg.height = self.camera_height
        camera_info_right_msg.distortion_model = "plumb_bob"
        camera_info_right_msg.d = self.D
        camera_info_right_msg.k = self.K.flatten().tolist()
        camera_info_right_msg.r = self.R
        camera_info_right_msg.p = self.P_right # Use P_right if it's different

        # --- Publish ---
        self.camLeftInfoPub.publish(camLeftInfoMsg)
        self.camera_info_right_publisher.publish(camera_info_right_msg)

        rclpy.spin_once(self.node, timeout_sec=0)


    def _calculate_camera_parameters(self):
        # Intrinsics (K matrix)
        focalLengthPixels = (self.camWidth / 2.0) / np.tan(self.camFov / 2.0)
        fx, fy = focalLengthPixels, focalLengthPixels
        cx = self.camWidth / 2.0
        cy = self.camHeight / 2.0

        # K matrix: [fx 0 cx; 0 fy cy; 0 0 1]
        self.K = np.array([
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ], dtype=np.float64).reshape(3, 3) # Reshape to 3x3 for CameraInfo.K

        # Distortion coefficients (D matrix): Assuming no distortion in Webots
        self.D = np.zeros(5, dtype=np.float64).tolist() # Needs to be a list for CameraInfo.D

        # Rectification matrix (R matrix): Identity for ideal cameras
        self.R = np.eye(3, dtype=np.float64).flatten().tolist() # Flatten for CameraInfo.R

        # Projection matrix (P matrix) for rectified image: [fx' 0 cx' Tx; 0 fy' cy' Ty; 0 0 1 0]
        # For a monocular camera, Tx, Ty are 0. For stereo, Tx is -fx * baseline for right camera
        # RTAB-Map expects the P matrix to be for the *rectified* camera.
        # Since Webots cameras are often ideal, and we assume stereoRectify is done internally by RTAB-Map,
        # we provide the monocular P matrix here. RTAB-Map will use the provided CameraInfo for its internal stereoRectify.

        # P matrix for left camera (P1 from stereoRectify context)
        self.P_left = np.array([
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ], dtype=np.float64).flatten().tolist() # Flatten for CameraInfo.P

        # P matrix for right camera (P2 from stereoRectify context)
        # The last column's first element is -fx * baseline for the right camera's P matrix
        # Your Webots setup is vertical stereo, but RTAB-Map (and most stereo algorithms)
        # assume horizontal disparity. RTAB-Map will rectify based on the TFs.
        # It's safest to provide the P matrix *as if* it were for a monocular camera,
        # and let RTAB-Map's internal stereoRectify handle the baseline.
        # However, if RTAB-Map's stereo processing expects a pre-rectified P matrix that includes baseline,
        # you *might* need to adjust this. For initial setup, provide standard monocular P.

        # Let's re-verify the baseline based on your Webots setup:
        # camera_left translation: 0.17 0.05 0.05
        # camera_right translation: 0.17 -0.05 0.0499997
        # The baseline is 0.05 - (-0.05) = 0.1m along the Y-axis (vertical stereo)
        self.baseline = 0.08 # meters

        # IMPORTANT for RTAB-Map:
        # If RTAB-Map's internal stereo module expects a P matrix with the baseline for the right camera,
        # the P matrix for the right camera should be:
        # [fx 0 cx -fx*B; 0 fy cy 0; 0 0 1 0] for a horizontal stereo baseline 'B'.
        # For *your vertical stereo*, it's more complex.
        # The simplest approach is to let RTAB-Map figure it out from your TF transforms (base_link to camera_left_link and base_link to camera_right_link).
        # So, provide P matrices that are just like the K matrix, with an extra column of zeros for Tx/Ty.

        self.P_right = self.P_left # For initial setup, assume they are the same in terms of projection
                                  # RTAB-Map will use TF for baseline information


