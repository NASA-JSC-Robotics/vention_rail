#!/usr/bin/env python3
#
# Copyright (c) 2025, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# This software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


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
            default_value="",
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
            description="Maximum height in meters for the lift",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "velocity_limit",
            default_value="0.15",
            description="Maximum velocity in meters/s for the rail",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "acceleration_limit",
            default_value="1.0",
            description="Maximum velocity in meters/s for the rail",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_com_port",
            default_value="/dev/ttyACM0",
            description="comport that the safety switch is being connected to",
        )
    )

    # other args
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
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
    acceleration_limit = LaunchConfiguration("acceleration_limit")
    safety_com_port = LaunchConfiguration("safety_com_port")

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
            "acceleration_limit:=",
            acceleration_limit,
            " ",
            "safety_com_port:=",
            safety_com_port,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_common_params = ParameterFile(
        PathJoinSubstitution([FindPackageShare("vention_rail_deploy"), "config", "controllers_common.yaml"]),
        allow_substs=True,
    )

    controller_rail_params = ParameterFile(
        PathJoinSubstitution([FindPackageShare("vention_rail_deploy"), "config", "rail_controllers.yaml"]),
        allow_substs=True,
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controller_common_params,
            controller_rail_params,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "controller_manager",
            "--controller-manager-timeout",
            "100",
        ],
    )

    rviz_config_file = PathJoinSubstitution([FindPackageShare("vention_rail_deploy"), "rviz", "view_robot.rviz"])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz),
    )

    nodes = [robot_state_publisher, controller_manager, joint_state_broadcaster_spawner, rviz_node]

    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("vention_rail_deploy"), "launch", "spawn_controllers.launch.py")
        ),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )

    return LaunchDescription(declared_arguments + nodes + [spawn_controllers_launch])
