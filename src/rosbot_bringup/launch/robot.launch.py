from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join
def generate_launch_description():
    # paths
    robot_description_path = get_package_share_directory("rosbot_description")
    diff_drive_controller_config = join(robot_description_path, "config", "robot_controllers.yaml")
    xacro_file = join(robot_description_path, "urdf", "rosbot.urdf.xacro")

    use_gazebo = LaunchConfiguration('use_gazebo')
    robot_description_content = Command(['xacro ', xacro_file, ' use_gazebo:=', use_gazebo])

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    declare_use_gazebo_argument = DeclareLaunchArgument(
            'use_gazebo',
            default_value='false',
            description='Use sim time if true')

    robot_description = {'robot_description': robot_description_content}
    params = {'robot_description': robot_description_content, 'use_sim_time': use_sim_time}

    # build launch description
    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_use_gazebo_argument,

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
            # parameters=[robot_description],
            parameters=[params]

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
