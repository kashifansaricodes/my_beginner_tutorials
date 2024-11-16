/**
 * @file simple_publisher.cpp
 * @author Kashif Ansari
 * @brief Implementation of a ROS2 publisher node with TF2 broadcasting capabilities
 * @version 0.2
 * @date 2024-02-08
 */

#include "simple_publisher.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

SimplePublisher::SimplePublisher(const rclcpp::NodeOptions& options)
    : Node("simple_publisher", options),
      count_(0),
      base_message_{"Hii"},
      publish_frequency_(2.0) {
      
  // Initialize the static transform broadcaster
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  
  // Set up and broadcast the static transform
  broadcast_static_transform();

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

void SimplePublisher::broadcast_static_transform() {
  geometry_msgs::msg::TransformStamped t;

  // Set header
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";  // parent frame
  t.child_frame_id = "talk";    // child frame

  // Set translation (non-zero values)
  t.transform.translation.x = 1.0;  // 1 meter in x
  t.transform.translation.y = 0.5;  // 0.5 meters in y
  t.transform.translation.z = 0.3;  // 0.3 meters in z

  // Set rotation (45 degrees around Z-axis)
  tf2::Quaternion q;
  q.setRPY(0.1,  // roll (rotation around X)
           0.2,  // pitch (rotation around Y)
           0.785);  // yaw (rotation around Z) - 45 degrees in radians

  // Convert quaternion to transform
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  // Send the transform
  tf_static_broadcaster_->sendTransform(t);
  
  RCLCPP_INFO(this->get_logger(), "Published static transform from 'world' to 'talk'");
}

rcl_interfaces::msg::SetParametersResult SimplePublisher::parameters_callback(
    const std::vector<rclcpp::Parameter>& parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto& param : parameters) {
        if (param.get_name() == "publish_frequency") {
            update_timer_period();
        }
    }
    
    return result;
}

void SimplePublisher::update_timer_period() {
    auto period = this->get_parameter("publish_frequency").as_double();
    timer_->cancel();
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / period),
        std::bind(&SimplePublisher::timer_callback, this));
}

void SimplePublisher::change_string_callback(
    const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
    std::shared_ptr<example_interfaces::srv::SetBool::Response> response) {
    if (request->data) {
        base_message_ = "Changed Hello";
        response->success = true;
        response->message = "Message changed successfully";
    } else {
        base_message_ = "Hello";
        response->success = true;
        response->message = "Message reset to default";
    }
}

void SimplePublisher::timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = base_message_ + " " + std::to_string(count_++);
    
    // Log the message being published
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    
    // Publish the message
    publisher_->publish(message);
}

// ... [rest of the existing methods remain the same] ...