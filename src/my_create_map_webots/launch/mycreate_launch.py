import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('my_create_map_webots')
    robot_description_path = os.path.join(package_dir, 'resource', 'MyCreate.urdf')
    # world_path = os.path.join(package_dir, 'worlds', 'school-obstacle.wbt')
    world_path = os.path.join(package_dir, 'worlds', 'alpha.wbt')

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

    # 啟動 RTAB-Map（stereo 模式）
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[
            {'use_sim_time': True},
            {'frame_id': 'base_link'},
            {'subscribe_depth': False},
            {'subscribe_stereo': True},
        ],
        remappings=[
            ('/stereo_camera/left/image_rect', '/camera_left/image_raw'),
            ('/stereo_camera/right/image_rect', '/camera_right/image_raw'),
            ('/stereo_camera/left/camera_info', '/camera_left/camera_info'),
            ('/stereo_camera/right/camera_info', '/camera_right/camera_info'),
        ]
    )

    return LaunchDescription([
        webots,
        my_robot_driver,
        rtabmap,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
