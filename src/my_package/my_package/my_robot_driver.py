import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

FORWARD_RATIO = 0.5
WHEEL_MAX_SPEED = 6.28
TURN_RATIO = 0.85

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

        self.__left_motor = self.__robot.getDevice('left_wheel_motor')
        self.__right_motor = self.__robot.getDevice('right_wheel_motor')
# wheel_left = turtlebot.getDevice("left_wheel_motor")
# wheel_right = turtlebot.getDevice("right_wheel_motor")


        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)

        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)

        # ==========
        # self.__target_twist = Twist()

        # rclpy.init(args=None)
        # self.__node = rclpy.create_node('my_robot_driver')
        # self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        # ==========

    # def __cmd_vel_callback(self, twist):
    #     self.__target_twist = twist

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

        # Keyboard control
        key = None
        key = self.__keyboard.getKey()

        if key == ord('W'):
            self.__left_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
            self.__right_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
        elif key == ord('S'):
            self.__left_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
            self.__right_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
        elif key == ord('A'):
            self.__left_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
            self.__right_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
        elif key == ord('D'):
            self.__left_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
            self.__right_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
        else:
            self.__left_motor.setVelocity(0.0)
            self.__right_motor.setVelocity(0.0)

        # Joystick control
        if self.__JOYSTICK_ENABLED:
            forward = self.__joystick.getAxisValue(1)  # Y-axis for forward/backward
            turn = self.__joystick.getAxisValue(0)  # X-axis for turning
            # print(f"forward: {forward}")
            # print(f"turn: {turn}")

            if forward != -1 and forward != 0:
                # Forward
                if forward < 0:
                    # Forward right
                    if turn > 0:
                        self.__left_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
                        self.__right_motor.setVelocity(WHEEL_MAX_SPEED * -TURN_RATIO)
                    # Forward left
                    elif turn < 0:
                        self.__left_motor.setVelocity(WHEEL_MAX_SPEED * -TURN_RATIO)
                        self.__right_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
                    # Forward only
                    else:
                        self.__left_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
                        self.__right_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
                # Backward
                else:
                    # Backward right
                    if turn > 0:
                        self.__left_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
                        self.__right_motor.setVelocity(WHEEL_MAX_SPEED * TURN_RATIO)
                    # Backward left
                    elif turn < 0:
                        self.__left_motor.setVelocity(WHEEL_MAX_SPEED * TURN_RATIO)
                        self.__right_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
                    # Backward only
                    else:
                        self.__left_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
                        self.__right_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
            elif turn != 0:
                # Trun right
                if turn > 0:
                    self.__left_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
                    self.__right_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
                # Trun left
                elif turn < 0:
                    self.__left_motor.setVelocity(WHEEL_MAX_SPEED * -FORWARD_RATIO)
                    self.__right_motor.setVelocity(WHEEL_MAX_SPEED * FORWARD_RATIO)
            else:
                self.__left_motor.setVelocity(0.0)
                self.__right_motor.setVelocity(0.0)
