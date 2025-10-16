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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
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

    # controller_params_file = os.path.join(
    #     get_package_share_directory("vention_rail_deploy"), "config", "rail_controllers.yaml"
    # )
    position_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rail_position_trajectory_controller",
            "--controller-manager-timeout",
            "100",
            "-c",
            "controller_manager",
        ],
    )
    estop_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rail_estop_controller",
            "--controller-manager-timeout",
            "100",
            "-c",
            "controller_manager",
        ],
        condition=UnlessCondition(use_fake_hardware),
    )
    rail_controller_stopper = Node(
        package="vention_rail_hardware_interface",
        executable="controller_stopper_node",
        name="rail_controller_stopper_node",
        parameters=[
            {
                "consistent_controllers": [
                    "rail_estop_controller",
                    "joint_state_broadcaster",
                ]
            },
        ],
        condition=UnlessCondition(use_fake_hardware),
    )

    nodes = [position_trajectory_controller_spawner, estop_controller_spawner, rail_controller_stopper]

    return LaunchDescription(declared_arguments + nodes)
