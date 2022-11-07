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
    diff_drive_controller_config = join(robot_description_path, "config", "mini_robot_controllers.yaml")
    rviz_config_file = join(robot_description_path, "config", "mini_robot.rviz" )
    teleop_config = join(robot_description_path, "config", "teleop_bluetooth.yaml")

    # launch arguments
    launch_teleop = LaunchConfiguration("launch_teleop")
    joy_dev = LaunchConfiguration("joy_dev")

    # get robot description from urdf xacro file
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [robot_description_path, "urdf", "rosbot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # build launch description
    return LaunchDescription([

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

        # ros2 control used by differential drive
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, diff_drive_controller_config],
            output="both",
        ),

        # robot state publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
            remappings=[
                ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
            ],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
        ),

        # joint state broadcaster
        # Node(
        #     package="controller_manager",
        #     executable="spawner",
        #     arguments=["joint_state_broadcaster"],
        # ),
        #
        # # differential drive controller
        # Node(
        #     package="controller_manager",
        #     executable="spawner",
        #     arguments=["diff_drive_controller"],
        #     output="screen",
        # ),

        Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["mini_robot_base_controller", "-c", "/controller_manager"],
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
            # remap joy topic to allow to co-exist with another joystick
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
