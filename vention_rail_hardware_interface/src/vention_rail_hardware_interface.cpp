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

#include "vention_rail_hardware_interface/vention_rail_hardware_interface.hpp"
#include "vention_rail_hardware_interface/vention_rail_cxx_api.hpp"

#include <math.h>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

// Velocity controller proportional gain
const double Kp = 5.0;
const double Kd = 5.0;
const double Ki = 0.0;
const double Ki_min = 0.0;
const double Ki_max = 0.0;
const bool antiwindup = true;

const std::string pressed = "113";
const std::string unpressed = "86";

using namespace std;

namespace vention_rail_hardware_interface
{
std::atomic<bool> run_ = false;
std::atomic<bool> com_closed_ = false;

void signal_callback_handler(int signum)
{
  // Terminate program
  run_ = false;
  while (!com_closed_)
  {
    // wait for the thread to shut down
  }
  exit(signum);
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn RailEHardwareInterface::on_init(const hardware_interface::HardwareInfo& info_)
{
  if (hardware_interface::ActuatorInterface::on_init(info_) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  hw_states_positions_.resize(info_.joints.size(), numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(info_.joints.size(), numeric_limits<double>::quiet_NaN());
  hw_states_robot_ready_.resize(info_.joints.size(), numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), numeric_limits<double>::quiet_NaN());
  signal(SIGINT, signal_callback_handler);
  system_info = info_;
  ip_addr = system_info.hardware_parameters["ip_addr"];
  port = stoi(system_info.hardware_parameters["port"]);
  position_limit = stof(system_info.hardware_parameters["position_limit"]);
  velocity_limit = stof(system_info.hardware_parameters["velocity_limit"]);
  acceleration_limit = stof(system_info.hardware_parameters["acceleration_limit"]);
  safety_com_port = system_info.hardware_parameters["safety_com_port"];
  return CallbackReturn::SUCCESS;
}

CallbackReturn RailEHardwareInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  sockfd_ = connect_to_rail(ip_addr, port, 50.0);
  RCLCPP_INFO(rclcpp::get_logger("RailEHardwareInterface"), "Successfully configure!");
  com_closed_ = false;
  // Make sure we are not moving
  string stop_all_motion_str = sendHTTPMessage(create_stop_all_motion_command(), sockfd_);
  // set maximum velocity and acceleration values
  string max_vel_str = sendHTTPMessage(create_set_max_vel_command(velocity_limit), sockfd_);
  string max_acc_str = sendHTTPMessage(create_set_max_acc_command(acceleration_limit), sockfd_);

  RCLCPP_INFO(rclcpp::get_logger("RailEHardwareInterface"), "Successfully configure!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn RailEHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  run_ = false;
  com_thread_.join();
  close_connection_to_rail(sockfd_);
  RCLCPP_INFO(rclcpp::get_logger("RailEHardwareInterface"), "Successfully cleanup!");
  return CallbackReturn::SUCCESS;
}

vector<hardware_interface::StateInterface> RailEHardwareInterface::export_state_interfaces()
{
  vector<hardware_interface::StateInterface> state_interfaces;

  // export sensor state interface
  for (uint i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, info_.joints[i].state_interfaces[0].name, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, info_.joints[i].state_interfaces[1].name, &hw_states_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, info_.joints[i].state_interfaces[2].name, &hw_states_robot_ready_[i]));
  }

  return state_interfaces;
}

vector<hardware_interface::CommandInterface> RailEHardwareInterface::export_command_interfaces()
{
  vector<hardware_interface::CommandInterface> command_interfaces;

  // export command state interface
  for (uint i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, info_.joints[i].state_interfaces[0].name, &hw_commands_positions_[i]));
  }

  return command_interfaces;
}

CallbackReturn RailEHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  for (unsigned int i = 0; i < hw_states_positions_.size(); ++i)
  {
    hw_states_positions_[i] = 0;
    hw_states_velocities_[i] = 0;
    hw_states_robot_ready_[i] = 0;
  }
  for (unsigned int i = 0; i < hw_commands_positions_.size(); ++i)
  {
    hw_commands_positions_[i] = 0;
  }

  if (!open_serial_port())
  {
    serial.close();
    RCLCPP_FATAL(rclcpp::get_logger("RailEHardwareInterface"),
                 "Failed to open safety comport. Is the safety serial port "
                 "available on %s?",
                 safety_com_port.c_str());
    return CallbackReturn::ERROR;
  }

  // flush the buffers
  serial.flush();
  // flush them again a different way because apparently I can't get this to
  // work
  while (serial.available() > 0)
  {
    std::string s = serial.read();
  }

  bool is_safe = false;
  while (!is_safe)
  {
    // keep track of how many times we are going so we can print to the user
    // every n times at the moment, the arduino is checking for polls every
    // 250ms, so this will print every 2 seconds
    const int print_every_n_polls = 8;
    static int counter = 0;

    // ask for status of the button
    serial.write("poll");

    // read the button status, vars pressed and unpressed correspond to the nums
    // that will be sent
    auto is_safe_string = serial.readline();
    is_safe = (is_safe_string.find(pressed) != std::string::npos);  // if 113 is in the read serial message

    // every n polls, remind the person to press the button
    if (counter++ % print_every_n_polls == 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("RailEHardwareInterface"), "PLEASE PRESS THE SAFETY BUTTON TO START HOMING");
    }

    // let the user know that the rail is homing
    if (is_safe)
    {
      RCLCPP_INFO(rclcpp::get_logger("RailEHardwareInterface"), "Button pressed! Homing the rail");
    }
  }
  // close the serial port because we don't need it anymore
  serial.close();

  // Trying to instantiate the driver
  try
  {
    string response = sendHTTPMessage(create_homing_command(), sockfd_);
    if (response.find("error") != string::npos)
    {
      sendHTTPMessage(create_stop_all_motion_command(), sockfd_);
      RCLCPP_FATAL(rclcpp::get_logger("RailEHardwareInterface"), "Is the Estop Active?");
      return CallbackReturn::ERROR;
    }
    else
    {
      RCLCPP_INFO(rclcpp::get_logger("RailEHardwareInterface"), "Driver successfully created!");
      while (true)
      {
        response = sendHTTPMessage(create_motion_complete_command(), sockfd_);
        RCLCPP_INFO(rclcpp::get_logger("RailEHardwareInterface"), "Motion complete: %s", response.c_str());
        if (response.find("true") != string::npos)
        {
          // set the homed position to 0
          response = sendHTTPMessage(create_set_position_command(0.0), sockfd_);
          RCLCPP_INFO(rclcpp::get_logger("RailEHardwareInterface"), "Set position: %s", response.c_str());
          break;
        }
      }
    }
  }
  catch (boost::system::system_error& e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("RailEHardwareInterface"), "TCP error: '%s'", e.what());
    return CallbackReturn::ERROR;
  }

  // Make sure we are not moving
  sendHTTPMessage(create_stop_all_motion_command(), sockfd_);

  // start the reading and writing threads
  com_thread_ = thread(&RailEHardwareInterface::com_thread, this);

  run_ = true;

  curr_position_ = 0.0;
  curr_velocity_ = 0.0;
  position_cmd_ = 0.0;

  pid_ = control_toolbox::Pid(Kp, Kd, Ki, Ki_max, Ki_min, antiwindup);
  pid_.reset();

  RCLCPP_INFO(rclcpp::get_logger("RailEHardwareInterface"), "Successfully activated!");

  return CallbackReturn::SUCCESS;
}

CallbackReturn RailEHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // the com thread will joint after they complete one more loop, and the
  // while(run_) triggers
  run_ = false;
  com_thread_.join();

  // Make sure we are not moving
  sendHTTPMessage(create_stop_all_motion_command(), sockfd_);

  RCLCPP_INFO(rclcpp::get_logger("RailEHardwareInterface"), "Successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type RailEHardwareInterface::read(const rclcpp::Time& /*time*/,
                                                             const rclcpp::Duration& /*period*/)
{
  // get thread-safe variables
  hw_states_positions_[0] = curr_position_;
  hw_states_velocities_[0] = curr_velocity_;

  hw_states_robot_ready_[0] = static_cast<double>(!is_stopped());

  RCLCPP_DEBUG(rclcpp::get_logger("RailEHardwareInterface"), "Reading positions: %f", hw_states_positions_[0]);
  RCLCPP_DEBUG(rclcpp::get_logger("RailEHardwareInterface"), "Reading velocities: %f", hw_states_velocities_[0]);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RailEHardwareInterface::write(const rclcpp::Time& /*time*/,
                                                              const rclcpp::Duration& /*period*/)
{
  static bool warned_ = false;
  if (hw_commands_positions_[0] > position_limit && !warned_)
  {
    RCLCPP_WARN(rclcpp::get_logger("RailEHardwareInterface"),
                "Commanded Position was greater than position limit! Position "
                "being clamped.");
    warned_ = true;
  }
  else if (hw_commands_positions_[0] <= position_limit && warned_)
  {
    warned_ = false;
  }

  if (!is_stopped())
  {
    // update threadsafe position setpoint
    position_cmd_ = std::clamp(hw_commands_positions_[0], 0.0, position_limit);
  }
  // or if we are stopped, the desired position just stays the same

  return hardware_interface::return_type::OK;
}

void RailEHardwareInterface::com_thread()
{
  auto last_time = std::chrono::steady_clock::now();
  auto curr_time = std::chrono::steady_clock::now();
  while (run_)
  {
    // get period of cycle for velocity calculation
    last_time = curr_time;
    curr_time = std::chrono::steady_clock::now();
    int64_t dt_com = std::chrono::duration_cast<std::chrono::nanoseconds>(curr_time - last_time).count();

    // read position and compute velocity
    double previous_position = curr_position_;
    std::string pos_str = sendHTTPMessage(create_get_position_command(), sockfd_);
    // if the returned string contains "error", don't try to parse
    std::string estop_status_str = sendHTTPMessage(create_estop_status_command(), sockfd_);
    hard_estopped = (estop_status_str.find("true") != string::npos);
    if (hard_estopped)
    {
      RCLCPP_FATAL(rclcpp::get_logger("RailEHardwareInterface"), "Check E-Stop Status!");
    }
    else
    {
      const double parsed_position = parse_position_string(pos_str);
      // check to see if we were able to correctly parse the position
      if (!isnan(parsed_position))
      {
        // update position and velocities
        curr_position_ = parsed_position;
        curr_velocity_ = (curr_position_ - previous_position) / (static_cast<double>(dt_com) / 1.0e9);
      }
      else
      {
        // estimate our current position based on our previous velocity if we
        // received invalid data
        curr_position_ = curr_position_ + curr_velocity_ * (static_cast<double>(dt_com) / 1.0e9);
      }
    }

    // condition to come out of a soft_estop is that the difference between the
    // current position and desired position is less than the
    // soft_estop_tolerance, and we are not estopped
    if (soft_estopped)
    {
      soft_estopped = (abs(curr_position_ - hw_commands_positions_[0]) > soft_estop_tolerance) || hard_estopped;
    }

    // write new velocity based on current position, velocity, and goal

    // calculate vel command using pid controller
    double unclamped_vel_cmd_pid = pid_.computeCommand(position_cmd_ - hw_states_positions_[0], dt_com);
    double velocity_cmd = std::clamp(unclamped_vel_cmd_pid, -velocity_limit, velocity_limit);

    // clamp absolute value of velocity so it doesn't oscillate around the
    // setpoint by a single tick
    if (abs(velocity_cmd) < 0.005)
      velocity_cmd = 0.0;

    // write desired velocity to rail if not estopped
    if (!hard_estopped && !soft_estopped)
    {
      string vel_cmd_str = create_velocity_command(velocity_cmd);
      string response = sendHTTPMessage(vel_cmd_str, sockfd_);
    }
    // if estop detected. command a 0 velocity before the stop motion command so
    // that this is saved after estop is over. Somehow, it still commands some
    // residual velocity even though none is commanded after
    else
    {
      string vel_cmd_str = create_velocity_command(0.0);
      string response = sendHTTPMessage(vel_cmd_str, sockfd_);
    }

    // if we received an error, stop the rail. It is probably in estop state.
    // Also command the current position
    if (hard_estopped)
    {
      sendHTTPMessage(create_stop_all_motion_command(), sockfd_);
      RCLCPP_FATAL(rclcpp::get_logger("RailEHardwareInterface"),
                   "Error sending velocity command, Check E-Stop Status!");
      // update all desired positions available to the current position
      hw_commands_positions_[0] = hw_states_positions_[0];
      position_cmd_ = hw_states_positions_[0];
      soft_estopped = true;
    }
    else if (is_stopped())
    {
      reset_estop();
    }
  }
  // once thread is over,
  sendHTTPMessage(create_stop_all_motion_command(), sockfd_);
  close_connection_to_rail(sockfd_);
  com_closed_ = true;
}

bool RailEHardwareInterface::open_serial_port()
{
  serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);

  serial.setPort(safety_com_port);
  serial.setBaudrate(9600);
  serial.setTimeout(timeout);

  try
  {
    serial.open();
    RCLCPP_INFO(rclcpp::get_logger("RailEHardwareInterface"), "Safety comport open at %s", safety_com_port.c_str());
    return true;
  }
  catch (serial::IOException& e)
  {
    RCLCPP_INFO(rclcpp::get_logger("RailEHardwareInterface"), "Safety comport - serial::IOException: %s", e.what());
    return false;
  }

  return true;
}
}  // namespace vention_rail_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(vention_rail_hardware_interface::RailEHardwareInterface, hardware_interface::ActuatorInterface)
