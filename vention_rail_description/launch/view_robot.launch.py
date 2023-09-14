from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition

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
            default_value="9999",
            description="Port number for Vention Rail",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "position_limit",
            default_value="2.0",
            description="Maximium position in meters for the rail",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "velocity_limit",
            default_value="0.5",
            description="Maximium velocity in meters/s for the rail",
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
    position_limit = LaunchConfiguration("position_limit")
    velocity_limit = LaunchConfiguration("velocity_limit")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("vention_rail_description"), "urdf", "vention_rail.urdf.xacro"]),
            " ", 
            "name:=",
            robot_name,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "ip_addr:=",
            ip_addr,
            " ",
            "port:=",
            port,
            " ",
            "position_limit:=",
            position_limit,
            " ",
            "velocity_limit:=",
            velocity_limit,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("vention_rail_description"), "rviz", "view_robot.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
   
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz)
    )

    nodes_to_start = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
