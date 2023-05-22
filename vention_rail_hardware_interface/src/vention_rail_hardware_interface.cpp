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
const double Kp = 500;

using namespace std;

namespace vention_rail_hardware_interface
{
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
        int sockfd = connect_to_rail(ip_addr, port);
        // Make sure we are not moving
        sendHTTPMessage(stop_all_motion().c_str(), sockfd);
        close_connection_to_rail(sockfd);

        RCLCPP_INFO(rclcpp::get_logger("RailEHardwareInterface"), "Successfully configure!");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RailEHardwareInterface::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        int sockfd = connect_to_rail(ip_addr, port);
        // Make sure we are not moving
        sendHTTPMessage(stop_all_motion().c_str(), sockfd);
        close_connection_to_rail(sockfd);

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
        int sockfd = connect_to_rail(ip_addr, port);
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
            string response = sendHTTPMessage(homing_cmd_str.c_str(), sockfd);
            if (response.find("error") != string::npos)
            {
                sendHTTPMessage(stop_all_motion().c_str(), sockfd);
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
                close_connection_to_rail(sockfd);
                while (true) {
                    sockfd = connect_to_rail(ip_addr, port);
                    string motion_complete_cmd_str = create_motion_complete_command();
                    response = sendHTTPMessage(motion_complete_cmd_str.c_str(), sockfd);
                    close_connection_to_rail(sockfd);
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
        sockfd = connect_to_rail(ip_addr, port);
        sendHTTPMessage(stop_all_motion().c_str(), sockfd);
        close_connection_to_rail(sockfd);
        RCLCPP_DEBUG(rclcpp::get_logger("RailEHardwareInterface"), "Successfully activated!");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RailEHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        int sockfd = connect_to_rail(ip_addr, port);
        // Make sure we are not moving
        sendHTTPMessage(stop_all_motion().c_str(), sockfd);
        close_connection_to_rail(sockfd);
        RCLCPP_INFO(rclcpp::get_logger("RailEHardwareInterface"), "Successfully deactivated!");
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type RailEHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        int sockfd = connect_to_rail(ip_addr, port);
        double previous_position_ = hw_states_positions_[0];
        hw_states_positions_[0] = get_rail_position(sockfd);
        hw_states_velocities_[0] = (hw_states_positions_[0] - previous_position_) / period.seconds();
        hw_states_robot_ready_[0] = double(!is_stopped());

        RCLCPP_DEBUG(
            rclcpp::get_logger("RailEHardwareInterface"),
            "Reading positions: %f",
            hw_states_positions_[0]);
        RCLCPP_DEBUG(
            rclcpp::get_logger("RailEHardwareInterface"),
            "Reading velocities: %f",
            hw_states_velocities_[0]);
        close_connection_to_rail(sockfd);
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type RailEHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        int sockfd = connect_to_rail(ip_addr, port);
        static bool warned_ = false;
        if (hw_commands_positions_[0] > position_limit && !warned_) {
            RCLCPP_WARN(rclcpp::get_logger("RailEHardwareInterface"), "Commanded Position was greater than position limit! Position being clamped.");
            warned_ = true;
        } else if (hw_commands_positions_[0] <= position_limit && warned_) {
            warned_ = false;
        }
        double position_cmd = clamp(hw_commands_positions_[0], 0.0, position_limit);
        double velocity = clamp(Kp * (position_cmd - hw_states_positions_[0]), -velocity_limit, velocity_limit);
        string vel_cmd_str = create_velocity_command(velocity);
        string response = sendHTTPMessage(vel_cmd_str.c_str(), sockfd);
        if (response.find("error") != string::npos)
        {
            sendHTTPMessage(stop_all_motion().c_str(), sockfd);
            RCLCPP_FATAL(
                rclcpp::get_logger("RailEHardwareInterface"),
                "Error sending velocity command, Check E-Stop Status!");
            hw_commands_positions_[0] = hw_states_positions_[0];
        }
        else if (is_stopped())
        {
            reset_estop();
        }

        close_connection_to_rail(sockfd);
        RCLCPP_DEBUG(rclcpp::get_logger("RailEHardwareInterface"), "Writing %f", velocity);

        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(vention_rail_hardware_interface::RailEHardwareInterface, hardware_interface::ActuatorInterface)