/**
 * @file simple_publisher.hpp
 * @author Kashif Ansari
 * @brief Header file for ROS2 publisher node implementation
 * @version 0.1
 * @date 2024-02-08
 *
 * @copyright Copyright (c) 2024 Kashif Ansari
 * @license MIT License
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software.
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
    /**
     * @brief Construct a new Simple Publisher object
     * @param options Node options for configuration
     */
    explicit SimplePublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
    /**
     * @brief Timer callback for periodic message publishing
     */
    void timer_callback();

    /**
     * @brief Service callback to change the base message string
     * @param request Service request containing boolean flag
     * @param response Service response with success status
     */
    void change_string_callback(
        const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
        const std::shared_ptr<example_interfaces::srv::SetBool::Response> response);

    /**
     * @brief Updates the timer period based on frequency parameter
     */
    void update_timer_period();

    /**
     * @brief Callback for parameter changes
     * @param parameters Vector of changed parameters
     * @return SetParametersResult Result of parameter update
     */
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter>& parameters);
    
    /**
     * @brief Broadcasts static transform between world and talk frames
     */
    void broadcast_static_transform();

    rclcpp::TimerBase::SharedPtr timer_;                 ///< Timer for periodic publishing
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  ///< Message publisher
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;  ///< Service to change message
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;  ///< Parameter callback handle
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;  ///< TF2 broadcaster
    size_t count_{0};                                    ///< Message counter
    std::string base_message_{"Hello"};                  ///< Base message string
    double publish_frequency_{2.0};                      ///< Publishing frequency in Hz
};

#endif  // BEGINNER_TUTORIALS_SIMPLE_PUBLISHER_HPP_