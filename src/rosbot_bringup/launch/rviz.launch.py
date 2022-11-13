from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # paths
    robot_description_path = get_package_share_directory("rosbot_description")
    rviz_config_file = join(robot_description_path, "config", "robot.rviz" )

    # build launch description
    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        ),
    ])
