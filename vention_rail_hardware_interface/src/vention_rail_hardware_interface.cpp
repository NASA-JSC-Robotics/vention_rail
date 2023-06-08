#include "vention_rail_hardware_interface/vention_rail_hardware_interface.hpp"
#include "vention_rail_hardware_interface/vention_rail_cxx_api.hpp"

#include <algorithm>
#include <mutex>
#include <string>
#include <iomanip>
#include <sstream>
#include <math.h>
#include <boost/shared_ptr.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

// Velocity controller proportional gain
const double Kp = 1000;
const double Kd = 1000;
const double Ki = 0.2; // need to do some more investigation. I tried this at 2500 and it didn't seem to do anything with the new pid class

using namespace std;

namespace vention_rail_hardware_interface
{
    std::atomic<bool> run_ = false;
    std::atomic<bool> read_closed_ = false;
    std::atomic<bool> write_closed_ = false;

    void signal_callback_handler(int signum)
    {
        // Terminate program
        std::cerr << "IN SIGNAL CALLBACK HANDLER" << std::endl;
        run_ = false;
        while (!read_closed_ && !write_closed_){
            // wait for the threads to shut down
        }
        exit(signum);
    }

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    CallbackReturn RailEHardwareInterface::on_init(const hardware_interface::HardwareInfo &info_)
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
        velocity_limit = stof(system_info.hardware_parameters["velocity_limit"]) * 1000.0; // Conversion from Meters to mm
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RailEHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        sockfd_read_ = connect_to_rail(ip_addr, port);
        sockfd_write_ = connect_to_rail(ip_addr, port);
        read_closed_ = false;
        write_closed_ = false;
        // Make sure we are not moving
        sendHTTPMessage(stop_all_motion().c_str(), sockfd_write_);
        // close_connection_to_rail(sockfd);

        RCLCPP_INFO(rclcpp::get_logger("RailEHardwareInterface"), "Successfully configure!");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RailEHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        run_ = false;
        read_thread_.join();
        write_thread_.join();
        // sendHTTPMessage(stop_all_motion().c_str(), sockfd_write_);
        close_connection_to_rail(sockfd_write_);
        close_connection_to_rail(sockfd_read_);
        // int sockfd = connect_to_rail(ip_addr, port);
        // Make sure we are not moving
        // close_connection_to_rail(sockfd);
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

    CallbackReturn RailEHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // int sockfd = connect_to_rail(ip_addr, port);
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

        // Trying to instantiate the driver
        try
        {
            string homing_cmd_str = create_homing_command();
            string response = sendHTTPMessage(homing_cmd_str.c_str(), sockfd_write_);
            if (response.find("error") != string::npos)
            {
                sendHTTPMessage(stop_all_motion().c_str(), sockfd_write_);
                RCLCPP_FATAL(
                    rclcpp::get_logger("RailEHardwareInterface"),
                    "Is the Estop Active?");
                return CallbackReturn::ERROR;
            }
            else
            {
                RCLCPP_INFO(
                    rclcpp::get_logger("RailEHardwareInterface"),
                    "Driver sucessfully created!");
                // close_connection_to_rail(sockfd);
                while (true) {
                    // sockfd = connect_to_rail(ip_addr, port);
                    string motion_complete_cmd_str = create_motion_complete_command();
                    response = sendHTTPMessage(motion_complete_cmd_str.c_str(), sockfd_write_);
                    // close_connection_to_rail(sockfd);
                    if (response.find("COMPLETED") != string::npos){
                        break;
                    } 
                }
 

            }
        }
        catch (boost::system::system_error &e)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("RailEHardwareInterface"),
                "TCP error: '%s'", e.what());
            return CallbackReturn::ERROR;
        }

        // Make sure we are not moving
        // sockfd = connect_to_rail(ip_addr, port);
        sendHTTPMessage(stop_all_motion().c_str(), sockfd_write_);
        // close_connection_to_rail(sockfd);
        read_thread_ = thread(&RailEHardwareInterface::readLoop, this); 
        write_thread_ = thread(&RailEHardwareInterface::writeLoop, this); 
        run_ = true;
        curr_position_ = 0.0;
        curr_velocity_ = 0.0;
        position_cmd_ = 0.0;
        pid_ = control_toolbox::Pid(Kp, Kd, Ki);
        pid_.reset();
        RCLCPP_DEBUG(rclcpp::get_logger("RailEHardwareInterface"), "Successfully activated!");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RailEHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        run_ = false;
        read_thread_.join();
        write_thread_.join();
        // Make sure we are not moving
        sendHTTPMessage(stop_all_motion().c_str(), sockfd_write_);

        RCLCPP_INFO(rclcpp::get_logger("RailEHardwareInterface"), "Successfully deactivated!");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RailEHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        hw_states_positions_[0] = curr_position_;
        hw_states_velocities_[0] = curr_velocity_;
        
        hw_states_robot_ready_[0] = double(!is_stopped());

        RCLCPP_DEBUG(
            rclcpp::get_logger("RailEHardwareInterface"),
            "Reading positions: %f",
            hw_states_positions_[0]);
        RCLCPP_DEBUG(
            rclcpp::get_logger("RailEHardwareInterface"),
            "Reading velocities: %f",
            hw_states_velocities_[0]);
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RailEHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        static bool warned_ = false;
        if (hw_commands_positions_[0] > position_limit && !warned_) {
            RCLCPP_WARN(rclcpp::get_logger("RailEHardwareInterface"), "Commanded Position was greater than position limit! Position being clamped.");
            warned_ = true;
        } else if (hw_commands_positions_[0] <= position_limit && warned_) {
            warned_ = false;
        }
        {
            std::lock_guard<std::mutex> lock(write_m_);
            position_cmd_ = clamp(hw_commands_positions_[0], 0.0, position_limit);
        }

        return hardware_interface::return_type::OK;
    }

    void RailEHardwareInterface::readLoop(){
        auto last_time = std::chrono::steady_clock::now();
        auto curr_time = std::chrono::steady_clock::now();
        while(run_){
            double previous_position = curr_position_;
            
            std::string message_fmt = "GET /smartDrives/position HTTP/1.1\r\n\r\n";
            std::string pos_str = sendHTTPMessage(message_fmt.c_str(), sockfd_read_);
            if (pos_str.find("error") != string::npos)
            {
                RCLCPP_FATAL(rclcpp::get_logger("RailEHardwareInterface"),"Error getting position, Check E-Stop Status!");
            }
            else{
                // Find the position in the reponse
                string temp = pos_str.substr(pos_str.size() - 7);
                // Tokenize position
                string token = temp.substr(temp.find(":") + 1).substr(0, token.find("}"));
                // Position provided is in milimeters, conversion to meters
                curr_position_ = stof(token) / 1000.0;
            }
            // get period of cycle for velocity calculation
            last_time = curr_time;
            curr_time = std::chrono::steady_clock::now();
            int64_t dt_read_ = std::chrono::duration_cast<std::chrono::nanoseconds> (curr_time - last_time).count();

            curr_velocity_ = (curr_position_ - previous_position) / (static_cast<double>(dt_read_)/1.0e9);
        }
        close_connection_to_rail(sockfd_read_);
        read_closed_ = true;
    }

    void RailEHardwareInterface::writeLoop(){
        auto last_time = std::chrono::steady_clock::now();
        auto curr_time = std::chrono::steady_clock::now();
        while(run_){

            last_time = curr_time;
            curr_time = std::chrono::steady_clock::now();
            int64_t dt_write_ = std::chrono::duration_cast<std::chrono::nanoseconds> (curr_time - last_time).count();
            
            double unclamped_vel_cmd_pid = pid_.computeCommand(position_cmd_ - hw_states_positions_[0], dt_write_);
            double pe, de, ie;
            pid_.getCurrentPIDErrors(pe, ie, de);

            double velocity_cmd = clamp(unclamped_vel_cmd_pid, -velocity_limit, velocity_limit);

            string vel_cmd_str = create_velocity_command(velocity_cmd);
            string response = sendHTTPMessage(vel_cmd_str.c_str(), sockfd_write_);
            if (response.find("error") != string::npos)
            {
                sendHTTPMessage(stop_all_motion().c_str(), sockfd_write_);
                RCLCPP_FATAL(
                    rclcpp::get_logger("RailEHardwareInterface"),
                    "Error sending velocity command, Check E-Stop Status!");
                hw_commands_positions_[0] = hw_states_positions_[0];
            }
            else if (is_stopped())
            {
                reset_estop();
            }

        }
        RCLCPP_INFO(rclcpp::get_logger("RailEHardwareInterface"),"exited write loop");
        sendHTTPMessage(stop_all_motion().c_str(), sockfd_write_);
        close_connection_to_rail(sockfd_write_);
        write_closed_ = true;
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(vention_rail_hardware_interface::RailEHardwareInterface, hardware_interface::ActuatorInterface)