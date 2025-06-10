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

#ifndef VENTION_RAIL_CXX_API_HPP_
#define VENTION_RAIL_CXX_API_HPP_

#include <math.h>
#include <netdb.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

namespace vention_rail_hardware_interface
{
/// safely exits in the case of ctrl+c events
void signal_callback_handler(int signum);

/// returns value of stopped_
bool is_stopped();

/// sets_value of stopped_ to true
void reset_estop();

// create command to check estop status
std::string create_estop_status_command();

// create command to send the rail to the homing position
std::string create_homing_command();

/// create command to get the position of the rail (returns in mm)
std::string create_get_position_command();

/// create command to set the position of the rail in meters
std::string create_set_position_command(double position);

/// create command to set the max velocity of the rail in m/s
std::string create_set_max_vel_command(double max_velocity);

/// create command to set the max acceleration of the rail in m/s2
std::string create_set_max_acc_command(double max_acceleration);

/// create command to check if motion is complete
std::string create_motion_complete_command();

/// create the stop all motion command. This also sets the stopped variable to
/// true
std::string create_stop_all_motion_command();

/// create velocity command with velocity provided in m/s and acceleration
/// provided in m/s^2
std::string create_velocity_command(double velocity, double accel = 1.0);

// takes current position returned from vention in format '(<position>)' and
// returns a double of the current position in m
double parse_position_string(std::string pos_str);

/// connect to rail with ip address, port number, and timeout in seconds
/// and returns the socketfd
int connect_to_rail(std::string ip_addr, int portno, float timeout);

/// close connection to a socket
void close_connection_to_rail(int sockfd);

/// send a message to a socket
std::string sendHTTPMessage(std::string message, int sockfd);

std::string recvHTTPMessage(int sockfd);

};  // namespace vention_rail_hardware_interface

#endif  // VENTION_RAIL_CXX_API_HPP_
