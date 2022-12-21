from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from os.path import join
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # list of launch arguments
    rosbot_bringup_path = get_package_share_directory("rosbot_bringup")
    launch_dir = join(rosbot_bringup_path, 'launch')

    declared_arguments = []
    use_sim_time = LaunchConfiguration('use_sim_time')
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        )
    )

    # gazebo simulation world to use
    default_gazebo_world = join(
        get_package_share_directory('rosbot_description'),
        'world',
        #'office.sdf'
        # 'simple.sdf'
        'obstacles.world'
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value=default_gazebo_world,
            description="Gazebo simulation world to use",
        )
    )

    # get robot description from urdf xacro file
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("rosbot_description"),
                    "urdf",
                    "rosbot.urdf.xacro",
                ]
            ),
        ]
    )

    # start gazebo server with simulated world
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=[
            "gzserver",
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            LaunchConfiguration("world")
        ],
        # remappings={
        #     ("/cmd_vel", "/mini_robot_base_controller/cmd_vel_unstamped")
        # },
        cwd=[launch_dir],
        output="screen"
    )

    # start gazebo client
    start_gazebo_client_cmd = ExecuteProcess(
        cmd=["gzclient"],
        cwd=[launch_dir],
        output="screen"
    )

    # spawn gazebo rosbot robot
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'rosbot',
            '-topic', 'robot_description'
        ],

        output='screen'
    )

    # setup list of nodes to launch
    nodes = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource([launch_dir, '/robot.launch.py']), launch_arguments={'use_gazebo': 'true', ' use_sim_time': 'true'}.items()),

        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        spawn_entity_node,

        IncludeLaunchDescription(PythonLaunchDescriptionSource([launch_dir, '/rviz.launch.py'])),

    ]

    return LaunchDescription(declared_arguments + nodes)