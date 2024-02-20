import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )    
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    controller_params_file = os.path.join(get_package_share_directory("vention_rail_deploy"),'config','rail_controllers.yaml')
    position_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rail_position_trajectory_controller", 
                   "--controller-manager-timeout","100",
                   "-c", "controller_manager",
                   "-t", "joint_trajectory_controller/JointTrajectoryController",
                #    "-p", controller_params_file
                ],
    )
    estop_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rail_estop_controller", 
                   "--controller-manager-timeout","100",
                   "-c", "controller_manager",
                   "-t", "vention_rail_controllers/EstopController ",
                #    "-p", controller_params_file
                ],
        condition=UnlessCondition(use_fake_hardware)
    )    
    rail_controller_stopper = Node(
        package='vention_rail_hardware_interface',
        executable='controller_stopper_node', 
        name='rail_controller_stopper_node',
        parameters=[
            {
                "consistent_controllers": [
                    "rail_estop_controller",
                    "joint_state_broadcaster",
                ]
            },
        ],
        condition=UnlessCondition(use_fake_hardware)
    )

    nodes = [position_trajectory_controller_spawner, estop_controller_spawner, rail_controller_stopper]    

    return LaunchDescription(declared_arguments + nodes)
