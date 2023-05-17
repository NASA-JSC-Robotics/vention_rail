import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

import xacro

from launch_ros.actions import Node
def generate_launch_description():
    declared_arguments = []
    # xacro args
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_name",
            default_value="vention_rail",
            description="name of the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ip_addr",
            default_value="192.168.7.2",
            description="IP Address for Vention Rail",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "port",
            default_value="8000",
            description="Port number for Vention Rail",
        )
    )

    # other args
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value='true',
            description="launch rviz",
        )
    )
    robot_name = LaunchConfiguration("robot_name")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    ip_addr = LaunchConfiguration("ip_addr")
    port = LaunchConfiguration("port")
    rviz = LaunchConfiguration("rviz")


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("vention_rail_description"), "urdf", 'vention_rail.urdf.xacro']),
            " ",
            "ip_addr:=",
            ip_addr,
            " ",
            "port:=",
            port,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    controller_params_file = os.path.join(get_package_share_directory("vention_rail_deploy"),'config','rail_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description,
                    controller_params_file]
    )

    
    position_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_trajectory_controller", "--controller-manager-timeout",
                "100",],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout",
                "100",],
    )

    io_and_status_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["io_and_status_controller", "--controller-manager-timeout",
                "100",],
    )

    robot_status_publisher = Node(
         package='rail_e_hardware_interface',
         executable='robot_status_publisher', 
         name='robot_status_publisher' 
       )
    
    controller_stopper = Node(
         package='rail_e_hardware_interface',
         executable='controller_stopper_node', 
         name='controller_stopper_node',
         parameters=[
            {
                "consistent_controllers": [
                    "io_and_status_controller",
                    "joint_state_broadcaster",
                ]
            },
        ],
       )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("vention_rail_deploy"), "rviz", "view_robot.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz)
    )

    nodes = [
        rsp,
        controller_manager,
        position_trajectory_controller_spawner,
        joint_state_broadcaster_spawner,
        io_and_status_controller_spawner,
        controller_stopper, 
        rviz_node
    ]

    return LaunchDescription(nodes)
