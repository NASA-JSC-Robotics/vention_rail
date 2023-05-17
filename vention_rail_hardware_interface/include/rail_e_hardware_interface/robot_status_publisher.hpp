#ifndef RAIL_E_ROBOT_STATUS_PUBLISHER_HPP_
#define RAIL_E_ROBOT_STATUS_PUBLISHER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

class RobotStatusPublisher : public rclcpp::Node
{
public:
    RobotStatusPublisher();

private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    size_t count_;
};

#endif // RAIL_E_ROBOT_STATUS_PUBLISHER_HPP_
