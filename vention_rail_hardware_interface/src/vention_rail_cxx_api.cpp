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
        return "im_home_axis_1;";
    }

    string create_motion_complete_command()
    {
        return "isMotionCompleted;";
    }

    string create_stop_all_motion_command()
    {
        stopped = true;
        return "im_stop;";
    }

    string create_get_position_command()
    {
        return "GET im_get_controller_pos_axis_1";
    }
    
    string create_set_position_command(double position)
    {
        // This is a bug in the MachineMotion firmware and should only require a value of 1000
        int position_mm = static_cast<int>(position*1000000.0);  // convert from 
        return "SET im_set_controller_pos_axis_1/" + to_string(position_mm) + "/;";
    }

    string create_set_max_vel_command(double max_velocity)
    {
        int max_vel_mm_s = static_cast<int>(max_velocity*1000.0); // convert from m/s to mm/s
        return "SET speed/" + to_string(max_vel_mm_s) + "/;";
    }

    string create_set_max_acc_command(double max_acceleration)
    {
        int max_acc_mm_s2 = static_cast<int>(max_acceleration*1000.0); // convert from m/s to mm/s
        return "SET speed/" + to_string(max_acc_mm_s2) + "/;";
    }

    string create_velocity_command(double velocity, double accel)
    {
        stopped = false;
        int vel_cmd = static_cast<int>(velocity*1000.0); // convert from m/s to mm/s
        int acc_cmd = static_cast<int>(accel*1000.0); // convert from m/s^2 to mm/s^2
        string command_str = "SET im_conv_1 S" + to_string(vel_cmd) + "A" + to_string(acc_cmd) + ";";
        return command_str;
    }

    double parse_position_string(std::string pos_str)
    {
        const auto pos_str_input = pos_str;

        // check for open parenthesis and delete it if there
        auto open_index = pos_str.find("(");
        if (open_index != std::string::npos){
            pos_str.erase(open_index,1);    
        }
        else{
            RCLCPP_ERROR(
                rclcpp::get_logger("RailEHardwareInterface"),
                "Returned position string: '%s' was not of format (<position>). Not updating position.", pos_str_input.c_str());
            return std::numeric_limits<double>::quiet_NaN();
        }

        // check for close parenthesis and delete it if there
        auto close_index = pos_str.find("(");
        if (open_index != std::string::npos){
            pos_str.erase(close_index,1);    
        }
        else{
            RCLCPP_ERROR(
                rclcpp::get_logger("RailEHardwareInterface"),
                "Returned position string: '%s' was not of format (<position>). Not updating position.", pos_str_input.c_str());
            return std::numeric_limits<double>::quiet_NaN();
        }

        // check for open parenthesis and delete it if there
        try
        {
            float position = static_cast<double>(stoi(pos_str)) / 1000.0; // convert mm to m
            return position;
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(
                rclcpp::get_logger("RailEHardwareInterface"),
                    "Was not able to parse '%s' into a float. Not updating position.", pos_str.c_str());
            return std::numeric_limits<double>::quiet_NaN();
        }
    }

    int connect_to_rail(string host, const int portno, const float timeout)
    {
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
        struct timeval timeout_;
        
        // get second and microsecond components of the timeout
        float whole, fraction;
        fraction = std::modf(timeout, &whole);
        timeout_.tv_sec = static_cast<int>(whole);
        timeout_.tv_usec = static_cast<int>(fraction)*1000000; // convert fraction into microseconds

        RCLCPP_INFO(rclcpp::get_logger("RailEHardwareInterface"),
                    "Seconds: %ld, us: %ld", timeout_.tv_sec, timeout_.tv_usec);

        if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout_,
                       sizeof timeout_) < 0)
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

    string sendHTTPMessage(string message_fmt, int sockfd)
    {

        int bytes, sent, received, total;
        char message[1024], response[4096];

        // fill in the message
        sprintf(message, "%s", message_fmt.c_str());

        // send the request
        total = strlen(message);
        sent = 0;
        do
        {
            bytes = write(sockfd, message + sent, total - sent);
            if (bytes < 0)
            {
                RCLCPP_FATAL(
                    rclcpp::get_logger("RailEHardwareInterface"),
                    "Error writing message to Rail");
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
            RCLCPP_WARN(
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
