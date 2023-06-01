#ifndef VENTION_RAIL_HARDWARE_INTERFACE__VENTION_RAIL_HARDWARE_INTERFACE_HPP_
#define VENTION_RAIL_HARDWARE_INTERFACE__VENTION_RAIL_HARDWARE_INTERFACE_HPP_

#include <thread>
#include <mutex>
#include <chrono>
#include <iostream>
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "visibility_control.h"





namespace vention_rail_hardware_interface
{
    class RailEHardwareInterface : public hardware_interface::ActuatorInterface
    {

        using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(RailEHardwareInterface)

        VENTION_RAIL_HARDWARE_INTERFACE_PUBLIC
        CallbackReturn on_init(const hardware_interface::HardwareInfo &info_) override;

        VENTION_RAIL_HARDWARE_INTERFACE_PUBLIC
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        VENTION_RAIL_HARDWARE_INTERFACE_PUBLIC
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

        VENTION_RAIL_HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        VENTION_RAIL_HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        VENTION_RAIL_HARDWARE_INTERFACE_PUBLIC
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        VENTION_RAIL_HARDWARE_INTERFACE_PUBLIC
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        VENTION_RAIL_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        VENTION_RAIL_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    
    protected:
        std::vector<double> hw_states_positions_;
        std::vector<double> hw_states_velocities_;
        std::vector<double> hw_states_robot_ready_;

        std::vector<double> hw_commands_positions_;

        hardware_interface::HardwareInfo system_info;
        std::string ip_addr;
        int port;
        double position_limit;
        double velocity_limit;
        double curr_position_;
        double curr_velocity_;
        double position_cmd_;
        bool run_;
        std::thread read_thread_;
        std::thread write_thread_;
        std::mutex read_m_;
        std::mutex write_m_;
        void readLoop();
        void writeLoop();

    };
}

#endif // VENTION_RAIL_HARDWARE_INTERFACE__VENTION_RAIL_HARDWARE_INTERFACE_HPP_