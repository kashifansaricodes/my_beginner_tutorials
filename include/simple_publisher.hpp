/**
 * @file simple_publisher.hpp
 * @author Kashif Ansari
 * @brief Header file for ROS2 publisher node implementation
 * @version 0.1
 * @date 2024-02-08
 *
 * @copyright Copyright (c) 2024 Kashif Ansari
 *
 * @details This header defines a ROS2 publisher node that publishes string
 * messages with configurable frequency and provides a service to modify the
 * message
 */

#ifndef BEGINNER_TUTORIALS_SIMPLE_PUBLISHER_HPP_
#define BEGINNER_TUTORIALS_SIMPLE_PUBLISHER_HPP_

#include <example_interfaces/srv/set_bool.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SimplePublisher : public rclcpp::Node {
public:
    SimplePublisher();

private:
    void timer_callback();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

#endif  // BEGINNER_TUTORIALS_SIMPLE_PUBLISHER_HPP_
