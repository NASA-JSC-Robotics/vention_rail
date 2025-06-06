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

#ifndef VENTION_RAIL_HARDWARE_INTERFACE__VENTION_RAIL_HARDWARE_INTERFACE_HPP_
#define VENTION_RAIL_HARDWARE_INTERFACE__VENTION_RAIL_HARDWARE_INTERFACE_HPP_

#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include "control_toolbox/pid.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "serial/serial.h"
#include "visibility_control.h"

namespace vention_rail_hardware_interface
{
class RailEHardwareInterface : public hardware_interface::ActuatorInterface
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RailEHardwareInterface)

  VENTION_RAIL_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info_) override;

  VENTION_RAIL_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  VENTION_RAIL_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  VENTION_RAIL_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  VENTION_RAIL_HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  VENTION_RAIL_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  VENTION_RAIL_HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  VENTION_RAIL_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  VENTION_RAIL_HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  bool open_serial_port();

protected:
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_states_robot_ready_;

  std::vector<double> hw_commands_positions_;

  hardware_interface::HardwareInfo system_info;

  int sockfd_;                  // communication socket for the vention system
  std::string ip_addr;          // ip address of the rail
  int port;                     // port for connecting to the rail
  std::string safety_com_port;  // port for connecting to the rail

  serial::Serial serial;

  double position_limit;      // maximum position for the rail
  double velocity_limit;      // max absolute velocity allowed
  double acceleration_limit;  // max absolute acceleration allowed

  std::atomic<double> curr_position_;  // thread-safe position
  std::atomic<double> curr_velocity_;  // thread-safe velocity
  std::atomic<double> position_cmd_;   // thread-safe position setpoint
  std::thread com_thread_;             // thread which performs read and write as fast as possible

  bool hard_estopped = false;          // variable to track if the hardware returned an estop condition
  bool soft_estopped = false;          // variable to make sure we can come out of estop after an event
  double soft_estop_tolerance = 0.01;  // difference between desired and actual
                                       // position to allow us out of an estop

  control_toolbox::Pid pid_;  // pid controller to

  /// function which performs the reading and writing
  void com_thread();
};
}  // namespace vention_rail_hardware_interface

#endif  // VENTION_RAIL_HARDWARE_INTERFACE__VENTION_RAIL_HARDWARE_INTERFACE_HPP_
