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

#include "vention_rail_hardware_interface/vention_rail_cxx_api.hpp"

using namespace std;
int main()
{
  int sockfd = connect_to_rail();
  string homing_cmd_str = create_homing_command();
  sendHTTPMessage(homing_cmd_str.c_str(), sockfd);
  close_connection_to_rail(sockfd);
  return 0;
}
