/**
 * @file simple_subscriber.hpp
 * @author Kashif Ansari
 * @brief Header file for ROS2 subscriber node implementation
 * @version 0.1
 * @date 2024-02-08
 *
 * @copyright Copyright (c) 2024 Kashif Ansari
 *
 * @details This header defines a ROS2 subscriber node that listens to string
 * messages on the /chatter topic and processes them with appropriate error
 * handling
 */

#ifndef MY_BEGINNER_TUTORIALS_SRC_BEGINNER_TUTORIALS_INCLUDE_SIMPLE_SUBSCRIBER_HPP_
#define MY_BEGINNER_TUTORIALS_SRC_BEGINNER_TUTORIALS_INCLUDE_SIMPLE_SUBSCRIBER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @brief A ROS2 subscriber node class
 *
 * @details The SimpleSubscriber class creates a ROS2 node that subscribes
 *          to string messages on the /chatter topic. It processes incoming
 *          messages and provides appropriate logging and error handling.
 */
class SimpleSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for SimpleSubscriber
   *
   * @details Initializes the ROS2 node, sets up the subscription,
   *          and configures logging for the subscriber
   */
  SimpleSubscriber();

 private:
  /**
   * @brief Callback function for processing received messages
   *
   * @param msg Shared pointer to the received string message
   * @details Handles incoming messages from the /chatter topic,
   *          includes validation and error checking
   */
  void topic_callback(const std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief Subscription object for receiving messages
   *
   * @details SharedPtr to the ROS2 subscription that listens
   *          to string messages on the /chatter topic
   */
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif  // MY_BEGINNER_TUTORIALS_SRC_BEGINNER_TUTORIALS_INCLUDE_SIMPLE_SUBSCRIBER_HPP_
