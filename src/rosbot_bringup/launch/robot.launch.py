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
    # rplidar_path = get_package_share_directory("rplidar_ros")
    diff_drive_controller_config = join(robot_description_path, "config", "robot_controllers.yaml")
    xacro_file = join(robot_description_path, "urdf", "rosbot.urdf.xacro")


    use_gazebo = LaunchConfiguration('use_gazebo')
    robot_description_content = Command(['xacro ', xacro_file, ' use_gazebo:=', use_gazebo])
    # Create a robot_state_publisher node
    # robot_description = {'robot_description': robot_description_content, 'use_sim_time': use_sim_time}
    robot_description = {'robot_description': robot_description_content}


    # get robot description from urdf xacro file
    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             # [robot_description_path, "urdf", "odrive_diffbot.urdf.xacro"]
    #             [robot_description_path, "urdf", "rosbot.urdf.xacro"]
    #         ),
    #     ]
    # )
    # robot_description = {"robot_description": robot_description_content}

    # build launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='false',
            description='Use sim time if true'),

        # IncludeLaunchDescription(PythonLaunchDescriptionSource([rplidar_path, '/launch/rplidar.launch.py'])),

        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'base_laser',
                'inverted': False,
                'angle_compensate': True,
            }],
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
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["mini_robot_base_controller", "-c", "/controller_manager"],
        )
    ])
