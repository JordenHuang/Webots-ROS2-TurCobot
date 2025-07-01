import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('my_create')
    robot_description_path = os.path.join(package_dir, 'resource', 'MyCreate.urdf')
    world_path = os.path.join(package_dir, 'worlds', 'school-obstacle.wbt')

    webots = WebotsLauncher(
        world=world_path
    )

    my_robot_driver = WebotsController(
        robot_name='MyCreate',
        parameters=[
            {'robot_description': robot_description_path},
        ],
        respawn=True
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
