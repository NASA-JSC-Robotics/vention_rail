/* Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include "vention_rail_controllers/estop_controller.hpp"

#include <string>

namespace vention_rail_controllers
{
controller_interface::CallbackReturn EstopController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration EstopController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  return config;
}

controller_interface::InterfaceConfiguration
vention_rail_controllers::EstopController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.emplace_back("vention_rail_base_to_carriage/robot_ready");

  return config;
}

controller_interface::return_type vention_rail_controllers::EstopController::update(const rclcpp::Time& /*time*/,
                                                                                    const rclcpp::Duration& /*period*/)
{
  publishProgramRunning();
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
vention_rail_controllers::EstopController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void EstopController::publishProgramRunning()
{
  bool program_running = static_cast<bool>(state_interfaces_[0].get_value());
  if (program_running_msg_.data != program_running)
  {
    program_running_msg_.data = program_running;
    program_state_pub_->publish(program_running_msg_);
  }
}

controller_interface::CallbackReturn
vention_rail_controllers::EstopController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  try
  {
    auto program_state_pub_qos = rclcpp::SystemDefaultsQoS();
    program_state_pub_qos.transient_local();
    program_state_pub_ =
        get_node()->create_publisher<std_msgs::msg::Bool>("~/robot_program_running", program_state_pub_qos);
  }
  catch (...)
  {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
vention_rail_controllers::EstopController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  try
  {
    // reset publisher
    program_state_pub_.reset();
  }
  catch (...)
  {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace vention_rail_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(vention_rail_controllers::EstopController, controller_interface::ControllerInterface)
