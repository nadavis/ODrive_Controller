from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # paths
    robot_description_path = get_package_share_directory("rosbot_description")
    rosbot_bringup_path = get_package_share_directory("rosbot_bringup")
    teleop_config = join(robot_description_path, "config", "teleop_bluetooth.yaml")

    # launch arguments
    launch_teleop = LaunchConfiguration("launch_teleop")
    joy_dev = LaunchConfiguration("joy_dev")

    # build launch description
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource([rosbot_bringup_path, '/launch/robot.launch.py'])),

        DeclareLaunchArgument(
            "launch_teleop",
            default_value="true",
            description="Start joystick teleop automatically with the launch file",
        ),

        DeclareLaunchArgument(
            "joy_dev",
            default_value="/dev/input/js0",
            description="Joystick device to use",
        ),

        # joystick, used by onboard teleop
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node_robot",
            parameters=[{
                "dev": joy_dev,
                "deadzone": 0.1,
                "autorepeat_rate": 20.0,
            }],
            # remap joy topic to allow to co-exist wcmd_velith another joystick
            remappings={
                ("/joy", "/joy_robot"),
                ("/cmd_vel", "/mini_robot_base_controller/cmd_vel_unstamped")
            },
            condition=IfCondition(launch_teleop),
        ),

        # onboard teleop
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_node_robot",
            parameters=[teleop_config],
            remappings={
                ("/joy", "/joy_robot"),
                ("/cmd_vel", "/mini_robot_base_controller/cmd_vel_unstamped")
            },
            condition=IfCondition(launch_teleop),
        ),
    ])
