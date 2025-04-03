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

        t = self.__node.get_clock().now() - self.__last_command_time
        if (t) > Duration(seconds=1):
            self.keyboard_control(None)
            self.__command_key_code = -1
        else:
            if self.__command_key_code != -1:
                self.__node.get_logger().info(f"Key read: {chr(self.__command_key_code)}")
            self.keyboard_control(self.__command_key_code)

        key = None
        key = self.__keyboard.getKey()

        # Joystick control
        if self.__JOYSTICK_ENABLED:
            self.joystick_control()

        # Keyboard control
        elif self.__command_key_code == -1:
            self.keyboard_control(key)

