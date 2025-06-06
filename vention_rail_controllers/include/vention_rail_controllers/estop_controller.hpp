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

#ifndef VENTION_RAIL_CONTROLLERS__ESTOP_CONTROLLER_HPP_
#define VENTION_RAIL_CONTROLLERS__ESTOP_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "std_srvs/srv/trigger.hpp"

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/bool.hpp"
#include "ur_dashboard_msgs/msg/robot_mode.hpp"
#include "ur_dashboard_msgs/msg/safety_mode.hpp"
#include "ur_msgs/msg/io_states.hpp"
#include "ur_msgs/msg/tool_data_msg.hpp"
#include "ur_msgs/srv/set_io.hpp"
#include "ur_msgs/srv/set_payload.hpp"
#include "ur_msgs/srv/set_speed_slider_fraction.hpp"

namespace vention_rail_controllers
{

class EstopController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_init() override;

private:
  void publishProgramRunning();

protected:
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> program_state_pub_;
  std_msgs::msg::Bool program_running_msg_;
};
}  // namespace vention_rail_controllers

#endif  // VENTION_RAIL_CONTROLLERS__ESTOP_CONTROLLER_HPP_
