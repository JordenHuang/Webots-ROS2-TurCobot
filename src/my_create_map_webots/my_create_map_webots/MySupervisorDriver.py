import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
import rclpy
from rclpy.time import Time
from launch_ros.actions import Node
from controller import Supervisor
from rosgraph_msgs.msg import Clock

class MySupervisorDriver(Node):
    def __init__(self):
        super().__init__('MySupervisorDriver')

        self.__robot = Supervisor()
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # /clock topic
        self.create_timer(1 / 1000, self.__supervisor_step_callback)
        self.__clock_publisher = self.create_publisher(Clock, 'clock', 10)

    def __supervisor_step_callback(self):
        if self.__robot.step(self.__timestep) < 0:
            self.get_logger().info('MySupervisor is shutting down...')
        else:
            clock_message = Clock()
            clock_message.clock = Time(seconds=self.__robot.getTime()).to_msg()
            self.__clock_publisher.publish(clock_message)


# class MySupervisor(Node):
#     def __init__(self):
#         super().__init__(
#             package='my_create_map_webots',
#             executable='world_launch.py',
#             namespace='MySupervisor',
#             remappings=[('/MySupervisor/clock', '/clock')],
#         )

#         self.__robot = Supervisor()
#         self.__timestep = int(self.__robot.getBasicTimeStep())

#         # /clock topic
#         self.create_timer(1 / 1000, self.__supervisor_step_callback)
#         self.__clock_publisher = self.create_publisher(Clock, 'clock', 10)

#         # Spawn Nodes (URDF robots or Webots objects)
#         root_node = self.__robot.getRoot()


def main(args=None):
    rclpy.init(args=args)
    ros_2_supervisor = MySupervisorDriver()
    rclpy.spin(ros_2_supervisor)
    rclpy.shutdown()


if __name__ == '__main__':
    main()