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

#include "example_interfaces/srv/set_bool.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class SimplePublisher : public rclcpp::Node {
 public:
    // Constructor with NodeOptions
    explicit SimplePublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
    // Callback Methods
    void timer_callback();
    void change_string_callback(
        const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
        const std::shared_ptr<example_interfaces::srv::SetBool::Response> response);
    void update_timer_period();
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter>& parameters);
    
    // TF2 broadcasting method
    void broadcast_static_transform();

    // Member variables
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // TF2 broadcaster
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    
    size_t count_{0};
    std::string base_message_{"Hello"};
    double publish_frequency_{2.0};  // Hz
};

#endif  // BEGINNER_TUTORIALS_SIMPLE_PUBLISHER_HPP_