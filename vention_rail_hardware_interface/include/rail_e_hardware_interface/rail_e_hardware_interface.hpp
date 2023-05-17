#ifndef RAIL_E_HARDWARE_INTERFACE__RAIL_E_HARDWARE_INTERFACE_HPP_
#define RAIL_E_HARDWARE_INTERFACE__RAIL_E_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "visibility_control.h"

namespace rail_e_hardware_interface
{
    class RailEHardwareInterface : public hardware_interface::ActuatorInterface
    {

        using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(RailEHardwareInterface)

        RAIL_E_HARDWARE_INTERFACE_PUBLIC
        CallbackReturn on_init(const hardware_interface::HardwareInfo &info_) override;

        RAIL_E_HARDWARE_INTERFACE_PUBLIC
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        RAIL_E_HARDWARE_INTERFACE_PUBLIC
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

        RAIL_E_HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        RAIL_E_HARDWARE_INTERFACE_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        RAIL_E_HARDWARE_INTERFACE_PUBLIC
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        RAIL_E_HARDWARE_INTERFACE_PUBLIC
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        RAIL_E_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        RAIL_E_HARDWARE_INTERFACE_PUBLIC
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    
    protected:
        std::vector<double> hw_states_positions_;
        std::vector<double> hw_states_velocities_;
        std::vector<double> hw_states_robot_ready_;

        std::vector<double> hw_commands_positions_;

        hardware_interface::HardwareInfo system_info;
        std::string ip_addr;
        int port;
    };
}

#endif // RAIL_E_HARDWARE_INTERFACE__RAIL_E_HARDWARE_INTERFACE_HPP_