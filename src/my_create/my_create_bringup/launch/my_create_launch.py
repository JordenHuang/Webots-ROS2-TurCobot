import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():
    bringup_dir = get_package_share_directory("my_create_bringup")
    description_dir = get_package_share_directory("my_create_description")

    default_config_file = bringup_dir + "/config/default.yaml"

    arg_config = DeclareLaunchArgument("config", default_value=default_config_file)
    arg_desc = DeclareLaunchArgument("desc", default_value="true")

    print("ok")
    condition = IfCondition(PythonExpression(["'", LaunchConfiguration("desc"), "' == 'true'"]))


    return LaunchDescription([
        # args that can be set from the command line or a default will be used
        arg_config,
        arg_desc,

        # include another launch file
        IncludeLaunchDescription(
            PathJoinSubstitution([description_dir, "launch.py"]),
            condition=condition,
        ),

        # start a node
        Node(
            name="mycreate_driver",
            package="my_create_driver",
            executable="driver", # my_create_driver/my_create_driver/driver.py
            output="screen",
            parameters=[
                {
                    "config": LaunchConfiguration("config"),
                    "robot_model": "CREATE_2"
                }
            ],
        ),
    ])

