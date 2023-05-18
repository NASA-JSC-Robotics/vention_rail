#include "vention_rail_hardware_interface/vention_rail_cxx_api.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
using namespace std;

bool stopped = false;
double prev_position = 0.0;
double curr_position = 0.0;

namespace vention_rail_hardware_interface
{    
    int port;
    string ip_addr;
    void signal_callback_handler(int signum)
    {
        // Terminate program
        int sockfd = connect_to_rail(ip_addr,port);
        // Make sure we are not moving
        sendHTTPMessage(stop_all_motion().c_str(), sockfd);
        close_connection_to_rail(sockfd);
        exit(signum);
    }

    double get_rail_position(int sockfd)
    {
        string message_fmt = "GET /smartDrives/position HTTP/1.1\r\n\r\n";
        string pos_str = sendHTTPMessage(message_fmt.c_str(), sockfd);
        // Find the position in the reponse
        string temp = pos_str.substr(pos_str.size() - 7);
        // Tokenize position
        string token = temp.substr(temp.find(":") + 1).substr(0, token.find("}"));
        prev_position = curr_position;
        // Position provided is in milimeters, conversion to meters
        curr_position = stof(token) / 1000.0;
        return curr_position;
    }

    double get_rail_velocity(double dt)
    {
        // Velocity in m/s (requires conversion from milimeters to meters)
        return (curr_position - prev_position) / (1000.0 * dt);
    }

    bool is_stopped()
    {
        return stopped;
    }

    void reset_estop()
    {
        stopped = false;
    }

    string create_homing_command()
    {
        string homing_str = "GET /gcode?gcode=G28+X HTTP/1.1\r\n\r\n";
        return homing_str;
    }

    string create_motion_complete_command()
    {
        string motion_complete_str = "GET /gcode?gcode=V0 HTTP/1.1\r\n\r\n";
        return motion_complete_str;
    }

    string stop_all_motion()
    {
        string stop_str = "GET /gcode?gcode=M410+X HTTP/1.1\r\n\r\n";
        stopped = true;
        return stop_str;
    }

    string create_velocity_command(double command)
    {
        double vel_cmd = command;
        string command_str = "GET /gcode?gcode=V7+S";
        if (vel_cmd < 0.0)
        {
            command_str += "-";
            vel_cmd = vel_cmd * -1.0;
        }
        string vel_cmd_str = to_string(vel_cmd);
        command_str += vel_cmd_str + "+A50.0+X HTTP/1.1\r\n\r\n";
        stopped = false;
        return command_str;
    }

    int connect_to_rail(string host, int portno)
    {
        port = portno;
        ip_addr = host;
        struct hostent *server;
        struct sockaddr_in serv_addr;

        // create the socket
        int sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("RailEHardwareInterface"),
                "Error opening socket to Rail");
        }

        // lookup the ip address
        server = gethostbyname(host.c_str());
        if (server == NULL)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("RailEHardwareInterface"),
                "Error finding IP Address to Rail");
        }

        // fill in the structure
        memset(&serv_addr, 0, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(portno);
        memcpy(&serv_addr.sin_addr.s_addr, server->h_addr, server->h_length);

        // connect the socket
        if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("RailEHardwareInterface"),
                "Error connecting to Rail");
        }

        // set timeout
        struct timeval timeout;
        timeout.tv_sec = 5;
        timeout.tv_usec = 0;

        if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                       sizeof timeout) < 0)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("RailEHardwareInterface"),
                "Error setting socket timeout");
        }
        return sockfd;
    }

    void close_connection_to_rail(int sockfd)
    {
        // close the socket
        close(sockfd);
    }

    string sendHTTPMessage(const char message_fmt[], int sockfd)
    {

        int bytes, sent, received, total;
        char message[1024], response[4096];

        // fill in the message
        sprintf(message, "%s", message_fmt);

        // send the request
        total = strlen(message);
        sent = 0;
        do
        {
            bytes = write(sockfd, message + sent, total - sent);
            if (bytes < 0)
            {
            //     RCLCPP_FATAL(
            //         rclcpp::get_logger("RailEHardwareInterface"),
            //         "Error writing message to Rail");
            }
            if (bytes == 0)
                break;
            sent += bytes;
        } while (sent < total);

        memset(response, 0, sizeof(response));

        // receive the response
        total = sizeof(response) - 1;
        received = 0;

        bytes = read(sockfd, response + received, total - received);
        if (bytes < 0)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("RailEHardwareInterface"),
                "ERROR reading response from socket");
        }
        else
        {
            received += bytes;
        }

        if (received == total)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("RailEHardwareInterface"),
                "ERROR storing complete response from socket, buffer too small");
        }
        return response;
    }
}
