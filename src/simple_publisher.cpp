/**
 * @file simple_publisher.cpp
 * @author Kashif Ansari
 * @brief Implementation of a ROS2 publisher node with parameter and service
 * capabilities
 * @version 0.1
 * @date 2024-02-08
 */

#include "simple_publisher.hpp"

#include <ctime>
#include <functional>
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @brief Construct a new Simple Publisher object
 *
 * @param options Node options for ROS2 configuration
 * @details Initializes the publisher node with configurable frequency,
 *          sets up parameter callback, and creates a service for string
 * modification
 */
SimplePublisher::SimplePublisher(const rclcpp::NodeOptions& options)
    : Node("simple_publisher", options),
      count_(0),
      base_message_{"Hii"},
      publish_frequency_(2.0) {
  // Create a parameter descriptor for frequency
  rcl_interfaces::msg::ParameterDescriptor freq_desc{};
  freq_desc.description = "Publishing frequency in Hz";

  // Set the valid range for frequency parameter
  freq_desc.floating_point_range.resize(1);
  freq_desc.floating_point_range[0].from_value = 0.1;
  freq_desc.floating_point_range[0].to_value = 30.0;

  // Declare and get the frequency parameter
  this->declare_parameter("publish_frequency", 2.0, freq_desc);
  publish_frequency_ = this->get_parameter("publish_frequency").as_double();

  // Log the initial frequency value
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Starting pub: " << publish_frequency_ << " Hz");

  // Create the publisher for the chatter topic
  publisher_ = this->create_publisher<std_msgs::msg::String>("/chatter", 10);

  // Set up callback for parameter changes
  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&SimplePublisher::parameters_callback, this, _1));

  // Initialize the timer with the specified frequency
  update_timer_period();

  // Create service for changing the base string
  service_ = create_service<example_interfaces::srv::SetBool>(
      "change_string",
      std::bind(&SimplePublisher::change_string_callback, this, _1, _2));

  // Log successful initialization
  RCLCPP_DEBUG_STREAM(this->get_logger(),
                      "Publisher initialized successfully on /chatter topic");
}

/**
 * @brief Callback function for the string modification service
 */
void SimplePublisher::change_string_callback(
    const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
    std::shared_ptr<example_interfaces::srv::SetBool::Response> response) {
  // Check for null pointers in request/response
  if (request == nullptr || response == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Received null request or response");
    return;
  }

  // Change the base message based on request data
  base_message_ = request->data ? "bye" : "Hii";
  response->success = true;
  response->message = "Changed base string to '" + base_message_ + "'";

  // Log the string change
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Base string changed to: " << base_message_);
}

/**
 * @brief Timer callback function for publishing messages
 */
void SimplePublisher::timer_callback() {
  // Create and populate the message
  auto message = std_msgs::msg::String();
  message.data = base_message_ + " World! " + std::to_string(count_++);

  // Log and publish the message
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

/**
 * @brief Updates the timer period based on the publish frequency
 * 
 * @details Calculates and sets a new timer period using the current publish_frequency_
 */
void SimplePublisher::update_timer_period() {
    // Calculate the period in milliseconds from the frequency
    const int period_ms = static_cast<int>(1000.0 / publish_frequency_);
    
    // Create a new timer with the calculated period
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&SimplePublisher::timer_callback, this));
}

/**
 * @brief Callback function for parameter updates
 */
rcl_interfaces::msg::SetParametersResult SimplePublisher::parameters_callback(
    const std::vector<rclcpp::Parameter>& parameters) {
  // Initialize the result
  rcl_interfaces::msg::SetParametersResult result{};
  result.successful = true;

  // Process each parameter
  for (const auto& param : parameters) {
    if (param.get_name() == "publish_frequency") {
      const double new_freq = param.as_double();

      // Validate and apply the new frequency
      if (new_freq > 0.0) {
        publish_frequency_ = new_freq;
        update_timer_period();
        RCLCPP_INFO(this->get_logger(), "Updated frequency to: %.2f Hz",
                    new_freq);
      } else {
        // Log warning for invalid frequency
        result.successful = false;
        result.reason = "Frequency must be positive";
        RCLCPP_WARN(this->get_logger(), "Invalid frequency requested: %.2f Hz",
                    new_freq);
      }
    }
  }
  return result;
}

/**
 * @brief Main function to initialize and run the publisher node
 */
int main(int argc, char* argv[]) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create and spin the node
  auto node = std::make_shared<SimplePublisher>();
  rclcpp::spin(node);

  // Clean shutdown
  rclcpp::shutdown();
  return 0;
}
