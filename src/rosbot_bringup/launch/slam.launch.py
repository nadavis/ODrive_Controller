from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os.path import join
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # paths
    robot_description_path = get_package_share_directory("rosbot_description")
    slam_toolbox_config_path = join(robot_description_path, "config", "slam_toolbox_mapping.yaml")
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    return LaunchDescription([
        declare_use_sim_time_argument,

        Node(
            parameters=[
                slam_toolbox_config_path,
                {"use_sim_time": use_sim_time}
            ],
            package="slam_toolbox",
            executable="sync_slam_toolbox_node",
            name="slam_toolbox_localization",
            output="screen"
        )
    ])
