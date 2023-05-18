#ifndef VENTION_RAIL_CONTROLLERS__ESTOP_CONTROLLER_HPP_
#define VENTION_RAIL_CONTROLLERS__ESTOP_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "std_srvs/srv/trigger.hpp"

#include "controller_interface/controller_interface.hpp"
#include "ur_msgs/msg/io_states.hpp"
#include "ur_msgs/msg/tool_data_msg.hpp"
#include "ur_dashboard_msgs/msg/robot_mode.hpp"
#include "ur_dashboard_msgs/msg/safety_mode.hpp"
#include "ur_msgs/srv/set_io.hpp"
#include "ur_msgs/srv/set_speed_slider_fraction.hpp"
#include "ur_msgs/srv/set_payload.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"
#include "std_msgs/msg/bool.hpp"

namespace vention_rail_controllers
{

    class EstopController : public controller_interface::ControllerInterface
    {
    public:
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        CallbackReturn on_init() override;

    private:
        void publishProgramRunning();

    protected:
        std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> program_state_pub_;
        std_msgs::msg::Bool program_running_msg_;
    };
} // namespace vention_rail_controllers

#endif // VENTION_RAIL_CONTROLLERS__ESTOP_CONTROLLER_HPP_