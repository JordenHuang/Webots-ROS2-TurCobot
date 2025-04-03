import rclpy
from geometry_msgs.msg import Twist
from enum import Enum
# from the_interfaces.srv import TermKeyboard
from rclpy.duration import Duration
import rclpy.node
import rclpy.time
from the_interfaces.msg import TermKeyboard

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

PI = 3.14159
FORWARD_RATIO = 0.5
WHEEL_MAX_SPEED = 6.28
TURN_RATIO = 0.85

class Direction(Enum):
    stop = 0
    forward = 1
    backward = 2
    left = 3
    right = 4
    forward_left = 5
    forward_right = 6
    backward_left = 7
    backward_right = 8

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__TIMESTEP = int(self.__robot.getBasicTimeStep())
        self.__JOYSTICK_ENABLED = False

        # Enable keyboard (Use keyboard to control the robot when focusing the webots window)
        self.__keyboard  = self.__robot.getKeyboard()
        self.__keyboard.enable(self.__TIMESTEP)

        # Enable camera
        self.__camera_left = self.__robot.getDevice("camera_left")
        self.__camera_right = self.__robot.getDevice("camera_right")
        self.__camera_left.enable(self.__TIMESTEP)
        self.__camera_right.enable(self.__TIMESTEP)

        # Enable joystick (if present)
        self.__joystick = self.__robot.getJoystick()
        self.__joystick.enable(self.__TIMESTEP)
        self.__robot.step()
        if self.__joystick.isConnected():
            self.__JOYSTICK_ENABLED = True
            print("Joystick connected")
            print(f"Joystick model: {self.__joystick.model}")

        # Get motors
        self.__left_motor = self.__robot.getDevice('left_wheel_motor')
        self.__right_motor = self.__robot.getDevice('right_wheel_motor')

        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        # Get joints (myCobot)
        self.__joints = []
        for i in range(6):
            self.__joints.append(self.__robot.getDevice(f"joint{i}_rotational_motor"))

        # Get gripper joints (myCobot-gripper)
        self.__gripper_right_base_outer = self.__robot.getDevice("gripper_right::base_outer::rotational_motor")
        self.__gripper_right_outer_paddle = self.__robot.getDevice( "gripper_right::outer_paddle::rotational_motor")
        self.__gripper_right_inner = self.__robot.getDevice("gripper_right::base_inner::rotational_motor")

        self.__gripper_left_base_outer = self.__robot.getDevice("gripper_left::base_outer::rotational_motor")
        self.__gripper_left_outer_paddle = self.__robot.getDevice( "gripper_left::outer_paddle::rotational_motor")
        self.__gripper_left_inner = self.__robot.getDevice("gripper_left::base_inner::rotational_motor")



        # self.__setup_keyboard_service()
        self.__setup_keyboard_topic_subscriber()

        # ==========
        # self.__target_twist = Twist()

        # rclpy.init(args=None)
        # self.__node = rclpy.create_node('my_robot_driver')
        # self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        # ==========

    # def __cmd_vel_callback(self, twist):
    #     self.__target_twist = twist

    def __setup_keyboard_topic_subscriber(self):
        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_keyboard_node')
        self.__last_command_time = self.__node.get_clock().now()
        self.__command_key_code = -1
        self.__node.create_subscription(TermKeyboard, 'term_keyboard', self.__my_term_keyboard_topic_subscriber_callback, 1)
    def __my_term_keyboard_topic_subscriber_callback(self, msg):
        # self.__node.get_logger().info(f"I heard: {msg.key}")
        self.__last_command_time = self.__node.get_clock().now()
        self.__command_key_code = ord(msg.key)

    def keyboard_control(self, key:int):
        if key == ord('W'):
            self.wheel_turn(Direction.forward)
        elif key == ord('S'):
            self.wheel_turn(Direction.backward)
        elif key == ord('A'):
            self.wheel_turn(Direction.left)
        elif key == ord('D'):
            self.wheel_turn(Direction.right)
        else:
            self.wheel_turn(Direction.stop)

    def joystick_control(self):
        forward = self.__joystick.getAxisValue(1)  # Y-axis for forward/backward
        turn = self.__joystick.getAxisValue(0)  # X-axis for turning
        # print(f"forward: {forward}")
        # print(f"turn: {turn}")

        if forward != -1 and forward != 0:
            # Forward
            if forward < 0:
                # Forward right
                if turn > 0:
                    self.wheel_turn(Direction.forward_right)
                # Forward left
                elif turn < 0:
                    self.wheel_turn(Direction.forward_left)
                # Forward only
                else:
                    self.wheel_turn(Direction.forward)
            # Backward
            else:
                # Backward right
                if turn > 0:
                    self.wheel_turn(Direction.backward_right)
                # Backward left
                elif turn < 0:
                    self.wheel_turn(Direction.backward_left)
                # Backward only
                else:
                    self.wheel_turn(Direction.backward)
        elif turn != 0:
            # Trun right
            if turn > 0:
                self.wheel_turn(Direction.right)
            # Trun left
            elif turn < 0:
                self.wheel_turn(Direction.left)
        else:
            self.wheel_turn(Direction.stop)

    def wheel_turn(self, dir: Direction):
        if dir == Direction.forward:
            self.__left_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
            self.__right_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
        elif dir == Direction.backward:
            self.__left_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
            self.__right_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
        elif dir == Direction.left:
            self.__left_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
            self.__right_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
        elif dir == Direction.right:
            self.__left_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
            self.__right_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
        elif dir == Direction.forward_left:
            self.__left_motor.setVelocity(WHEEL_MAX_SPEED * -TURN_RATIO)
            self.__right_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
        elif dir == Direction.forward_right:
            self.__left_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
            self.__right_motor.setVelocity(WHEEL_MAX_SPEED * -TURN_RATIO)
        elif dir == Direction.backward_left:
            self.__left_motor.setVelocity(WHEEL_MAX_SPEED * TURN_RATIO)
            self.__right_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
        elif dir == Direction.backward_right:
            self.__left_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
            self.__right_motor.setVelocity(WHEEL_MAX_SPEED * TURN_RATIO)
        else:
            self.__left_motor.setVelocity(0.0)
            self.__right_motor.setVelocity(0.0)

    def mycobot_send_angles(self, degrees: list, speed=0.05):
        rad = [d * (PI / 180) for d in degrees]  # Convert degrees to radians
        current_angles = [j.getTargetPosition() for j in self.__joints]  # Get current joint angles

        while any(abs(curr - target) > 0.01 for curr, target in zip(current_angles, rad)):  # Loop until close enough
            for i in range(len(self.__joints)):
                diff = rad[i] - current_angles[i]
                step = speed if abs(diff) > speed else abs(diff)  # Step should not exceed remaining distance
                current_angles[i] += step * (1 if diff > 0 else -1)  # Move in the correct direction
                self.__joints[i].setPosition(current_angles[i])  # Apply new position
            
            # time.sleep(0.01)  # Control loop timing
            self.__robot.step()  # Small delay to control speed

    def mycobot_gripper_send_angle(self, degree: int, speed=0.007):
        rad_right = degree * (PI / 180)  # Convert degrees to radians
        current_angle_right = self.__gripper_right_base_outer.getTargetPosition()  # Get current joint angles

        rad_left = (-degree) * (PI / 180)  # Convert degrees to radians
        current_angle_left = self.__gripper_left_base_outer.getTargetPosition()  # Get current joint angles

        flag = False
        while True:  # Loop until close enough
            flag = False
            # Right
            if abs(current_angle_right - rad_right) > 0.01:
                diff = rad_right - current_angle_right
                step = speed if abs(diff) > speed else abs(diff)  # Step should not exceed remaining distance
                current_angle_right += step * (1 if diff > 0 else -1)  # Move in the correct direction

                self.__gripper_right_base_outer.setPosition(current_angle_right)  # Apply new position
                self.__gripper_right_outer_paddle.setPosition(-current_angle_right)  # Apply new position
                self.__gripper_right_inner.setPosition(current_angle_right)  # Apply new position
            else:
                flag = True

            # Left
            if abs(current_angle_left - rad_left) > 0.01:
                diff = rad_left - current_angle_left
                step = speed if abs(diff) > speed else abs(diff)  # Step should not exceed remaining distance
                current_angle_left += step * (1 if diff > 0 else -1)  # Move in the correct direction

                self.__gripper_left_base_outer.setPosition(current_angle_left)  # Apply new position
                self.__gripper_left_outer_paddle.setPosition(-current_angle_left)  # Apply new position
                self.__gripper_left_inner.setPosition(current_angle_left)  # Apply new position
            else:
                if flag == True:
                    break
            
            self.__robot.step(2*self.__TIMESTEP)  # Small delay to control speed


    def step(self):
        # ==========
        # rclpy.spin_once(self.__node, timeout_sec=0)

        # forward_speed = self.__target_twist.linear.x
        # angular_speed = self.__target_twist.angular.z

        # command_motor_left = -(forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        # command_motor_right = -(forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        # self.__left_motor.setVelocity(command_motor_left)
        # self.__right_motor.setVelocity(command_motor_right)

        # Uncomment below will make the turtlebot keep moving forward
        # self.__left_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO / WHEEL_RADIUS)
        # self.__right_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO / WHEEL_RADIUS)
        # ==========

        # Terminal keyboard control
        rclpy.spin_once(self.__node, timeout_sec=0)

        if (self.__node.get_clock().now() - self.__last_command_time) > Duration(seconds=1):
            self.keyboard_control(None)
            self.__command_key_code = -1
        else:
            if self.__command_key_code != -1:
                self.__node.get_logger().info(f"Key read: {chr(self.__command_key_code)}")
            self.keyboard_control(self.__command_key_code)

        key = self.__keyboard.getKey()

        # Joystick control
        if self.__JOYSTICK_ENABLED:
            self.joystick_control()

        # No key recieved, return
        if key == -1:
            return

        # Keyboard control
        if chr(key) in ['W', 'A', 'S', 'D']:
            self.keyboard_control(key)

        # myCobot and gripper control
        modes = [str(c) for c in range(0, 7)]
        mode = -1
        # self.__node.get_logger().info(f"key: {chr(key)}, c: {modes[0]})")
        if chr(key) in modes:
            mode = int(chr(key))
        # Not a valid key, return
        else:
            return

        # myCobot
        if mode == 1:
            degrees = [0, 0, 0, 0, 165, 0]
            self.mycobot_send_angles(degrees)
        elif mode == 2:
            degrees = [0, -135, 150, -125, 90, 0]
            self.mycobot_send_angles(degrees)
        elif mode == 3:
            degrees = [0, 110, 5, -15, 165, 0]
            self.mycobot_send_angles(degrees, 0.02)

        # Gripper
        elif mode == 4:
            deg = 30
            self.mycobot_gripper_send_angle(deg)
            mode = 0
        elif mode == 5:
            deg = -30
            self.mycobot_gripper_send_angle(deg)
            degrees = [0, 127, -5, -25, 165, 0]
            mode = 0
        
        elif mode == 6:
            degrees = [0, 80, 15, -35, 165, 0]
            self.mycobot_send_angles(degrees, 0.02)
            mode = 0
