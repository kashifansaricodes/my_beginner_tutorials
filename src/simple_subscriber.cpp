/**
 * @file simple_subscriber.cpp
 * @author Kashif Ansari
 * @brief Implementation of a ROS2 subscriber node that listens to string
 * messages
 * @version 0.1
 * @date 2024-02-08
 *
 * @copyright Copyright (c) 2024 Kashif Ansari
 *
 */
#include "simple_subscriber.hpp"

/**
 * @brief Construct a new Simple Subscriber object
 *
 * @details Initializes a ROS2 subscriber node that listens to the /chatter
 * topic Sets up subscription with error checking and appropriate logging
 */
SimpleSubscriber::SimpleSubscriber() : Node("simple_subscriber") {
  // Log the initialization of the subscriber node
  RCLCPP_INFO_STREAM(this->get_logger(), "Initializing Subscriber Node");

  // Create a subscription to the /chatter topic with QoS depth of 10
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/chatter", 10,
      std::bind(&SimpleSubscriber::topic_callback, this,
                std::placeholders::_1));

  // Verify subscription creation and log appropriate message
  if (!subscription_) {
    // Log fatal error if subscription creation fails
    RCLCPP_FATAL(this->get_logger(), "Failed to create subscriber");
    return;
  }

  // Log successful creation of subscriber using debug level
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Subscriber created successfully");
}

/**
 * @brief Callback function for processing received messages
 *
 * @param msg Shared pointer to the received string message
 * @details Processes received messages from the /chatter topic
 *          Includes error checking for empty messages
 */
void SimpleSubscriber::topic_callback(
    const std_msgs::msg::String::SharedPtr msg) {
  // Check for empty messages
  if (msg->data.empty()) {
    // Log error for empty messages
    RCLCPP_ERROR(this->get_logger(), "Received empty message");
    return;
  }

  // Log the received message content
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

/**
 * @brief Main function to initialize and run the subscriber node
 *
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return int Exit status
 * @details Initializes ROS2, creates and spins the subscriber node,
 *          and handles graceful shutdown
 */
int main(int argc, char* argv[]) {
  // Initialize ROS2 system
  rclcpp::init(argc, argv);

  // Create and spin the subscriber node
  rclcpp::spin(std::make_shared<SimpleSubscriber>());

  // Perform clean shutdown
  rclcpp::shutdown();
  return 0;
}
