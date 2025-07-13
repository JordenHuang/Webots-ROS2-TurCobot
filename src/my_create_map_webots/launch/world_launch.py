import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
import rclpy
from launch_ros.actions import Node
from controller import Supervisor
from rosgraph_msgs.msg import Clock



# def generate_launch_description():
#     package_dir = get_package_share_directory('my_create_map_webots')
#     robot_description_path = os.path.join(package_dir, 'resource', 'MyCreate.urdf')
#     # world_path = os.path.join(package_dir, 'worlds', 'school-obstacle.wbt')
#     world_path = os.path.join(package_dir, 'worlds', 'alpha.wbt')

#     webots = WebotsLauncher(
#         world=world_path,
#         ros2_supervisor=True
#     )
#     # ros2_supervisor = Ros2SupervisorLauncher()
#     ros2_supervisor = Node(
#         robot_name='MyCreate_2',
#         parameters=[{
#                 'robot_description': robot_description_path,
#                 'use_sim_time': True,
#         }],
#         respawn=True
#     )

#     return LaunchDescription([
#         webots,
#         ros2_supervisor,

#         launch.actions.RegisterEventHandler(
#             event_handler=launch.event_handlers.OnProcessExit(
#                 target_action=webots,
#                 on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
#             )
#         )
#     ])
