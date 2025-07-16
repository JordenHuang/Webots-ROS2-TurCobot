import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.urdf_spawner import URDFSpawner
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import TimerAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationNotEquals, LaunchConfigurationEquals

def generate_launch_description():
    package_dir = get_package_share_directory('my_create_map_webots')
    robot_description_path = os.path.join(package_dir, 'resource', 'MyCreate.urdf')
    supervisor_description_path = os.path.join(package_dir, 'resource', 'MySupervisor.urdf')
    world_path = os.path.join(package_dir, 'worlds', 'school-obstacle.wbt')
    # world_path = os.path.join(package_dir, 'worlds', 'alpha.wbt')

    webots = WebotsLauncher(
        world=world_path,
        ros2_supervisor=True
    )

    # my_supervisor = Node(
    #     package='my_create_map_webots',
    #     executable='MySupervisorDriver.py',
    #     namespace='MySupervisor',
    #     remappings=[('/MySupervisor/clock', '/clock')],
    # )

    my_robot_driver = WebotsController(
        robot_name='MyCreate',
        parameters=[{
                'robot_description': robot_description_path,
                'use_sim_time': True,
        }],
        respawn=True
    )

    # stereo_image_proc = ComposableNodeContainer(
    #     name='stereo_proc_container',
    #     namespace='',
    #     package='rclcpp_components',
    #     executable='component_container_mt',
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='stereo_image_proc',
    #             plugin='stereo_image_proc::DisparityNode',
    #             name='stereo_image_proc',
    #             remappings=[
    #                 ('left/image_rect', '/camera_left/image_raw'),
    #                 ('right/image_rect', '/camera_right/image_raw'),
    #                 ('left/camera_info', '/camera_left/camera_info'),
    #                 ('right/camera_info', '/camera_right/camera_info'),
    #             ],
    #             parameters=[{
    #                 'use_sim_time': True,
    #                 'approximate_sync': True,
    #             }]
    #         )
    #     ],
    #     output='screen',
    # )

    rtabmap_odom = Node(
        package='rtabmap_odom',
        executable='stereo_odometry',
        name='stereo_odometry',
        parameters=[{
                'use_sim_time': True,
                'frame_id': 'base_link',
                # 'odom_frame_id': 'odom',
                'publish_tf': True,
                'approx_sync': True,       # <-- **SET THIS TO TRUE**
                'approx_sync_max_interval': 0.05,
                # 'Stereo/MinDisparity': str(0),
                # 'Stereo/MaxDisparity': str(320),

                # 'Odom/Strategy': str(1),
                'Vis/MinInliers': str(10),
                # 'Vis/InlierDistance': str(0.1),
            }],
        remappings=[
            ('/left/image_rect', '/camera_left/image_raw'),
            ('/right/image_rect', '/camera_right/image_raw'),
            ('/left/camera_info', '/camera_left/camera_info'),
            ('/right/camera_info', '/camera_right/camera_info'),
        ]
    )

    # 啟動 RTAB-Map（stereo 模式）
    window_size = 5
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        parameters=[{
            'use_sim_time': True,
            'frame_id': 'base_link',
            'subscribe_depth': False,
            'subscribe_stereo': True,
            'publish_tf': True,
            'subscribe_odom_info': True,
            'approx_sync': True,
            # ---- StereoSGBM Parameters ----
            # 'StereoSGBM/MinDisparity': str(0),          # 最小視差值 (通常為 0)
            # 'StereoSGBM/NumDisparities': str(320),       # 視差範圍，必須是 16 的倍數
            # 'StereoSGBM/BlockSize': str(window_size),   # 必須是奇數，通常 3-11
            # 'StereoSGBM/P1': str(8*3*window_size**2),   # 視差平滑度參數 (8 * channels * blockSize^2)
            # 'StereoSGBM/P2': str(32*3*window_size**2),  # 視差平滑度參數 (32 * channels * blockSize^2)
            # 'StereoSGBM/Disp12MaxDiff': str(1),         # 允許左右視差的最大差異
            # # 'StereoSGBM/UniquenessRatio': str(5),      # 唯一性比率，用於濾波
            # # 'StereoSGBM/SpeckleWindowSize':str(50),   # 視差斑點窗口大小，用於濾波
            # # 'StereoSGBM/SpeckleRange': str(2),         # 視差斑點範圍，用於濾波
            # 'StereoSGBM/PreFilterCap': str(63),         # 預濾波器截斷值
            # 'StereoSGBM/Mode': str(0),                  # 0: SGBM_MODE_SGBM, 1: SGBM_MODE_HH, 2: SGBM_MODE_SGBM_3WAY, 3: SGBM_MODE_HH4
'StereoSGBM/Uniquenese': str(0),            # 關閉斑點濾波

            # -------------------------------
            # 'Stereo/MinDisparity': str(0),
            # 'Stereo/MaxDisparity': str(320),

            'Vis/MinInliers': str(10),
            # 'wait_for_transform': 0.5,
        }],
        remappings=[
            ('/left/image_rect', '/camera_left/image_raw'),
            ('/right/image_rect', '/camera_right/image_raw'),
            ('/left/camera_info', '/camera_left/camera_info'),
            ('/right/camera_info', '/camera_right/camera_info'),
        ],
        # arguments=['-d', "--udebug"]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(robot_description_path).read(),
        }]
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        my_robot_driver,
        # my_supervisor,
        # stereo_image_proc,

        TimerAction(
            period=3.0,
            actions=[rtabmap, rtabmap_odom]
        ),
        TimerAction(
            period=5.0,
            actions=[robot_state_publisher]
        ),
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
