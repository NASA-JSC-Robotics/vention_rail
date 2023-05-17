#ifndef RAIL_E_CXX_API_HPP_
#define RAIL_E_CXX_API_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <cstring>
#include <math.h>
#include <iostream>
#include <signal.h>

namespace rail_e_hardware_interface
{
    void signal_callback_handler(int signum);
    double get_rail_position(int sockfd);
    double get_rail_velocity(double dt);
    bool is_stopped();
    void reset_estop();
    std::string create_homing_command();
    std::string create_motion_complete_command();
    std::string stop_all_motion();
    std::string create_velocity_command(double command);
    int connect_to_rail(std::string ip_addr, int portno);
    void close_connection_to_rail(int sockfd);
    std::string sendHTTPMessage(const char message_fmt[], int sockfd);
};

#endif // RAIL_E_CXX_API_HPP_