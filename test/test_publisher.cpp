/**
 * @file test_publisher.cpp
 * @author Kashif Ansari
 * @brief Integration tests for the publisher node
 * @version 0.1
 * @date 2024-02-08
 * 
 * @copyright Copyright (c) 2024
 */

#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>

using namespace std::chrono_literals;
using std_msgs::msg::String;

////////////////////////////////////////////////
// Define Fixture
////////////////////////////////////////////////

auto logger = rclcpp::get_logger("");

/**
 * @brief Test fixture class for publisher integration tests
 * @details Sets up a test environment with a publisher node and necessary ROS2 infrastructure
 */
class MyTestsFixture {
 public:
  /**
   * @brief Construct a new test fixture
   * @details Initializes ROS2, creates test and publisher nodes, and sets up parameters
   */
  MyTestsFixture() {
    // Initialize ROS 2
    rclcpp::init(0, nullptr);
    
    // Create the test node
    tester_node_ = rclcpp::Node::make_shared("IntegrationTestNode1");
    logger = tester_node_->get_logger();

    // Create and start the talker node
    talker_node_ = std::make_shared<rclcpp::Node>("talker");
    publisher_ = talker_node_->create_publisher<String>("chatter", 10);

    // Start publisher timer
    auto publish_message = [this]() {
        auto message = String();
        message.data = "Test message";
        publisher_->publish(message);
    };
    timer_ = talker_node_->create_wall_timer(500ms, publish_message);

    // Declare a parameter for the duration of the test
    tester_node_->declare_parameter<double>("test_duration", 10.0);
    test_duration_ = tester_node_->get_parameter("test_duration").get_parameter_value().get<double>();
    RCLCPP_INFO_STREAM(logger, "Got test_duration =" << test_duration_);
  }

  /**
   * @brief Destroy the test fixture
   * @details Performs ROS2 shutdown and cleanup
   */
  ~MyTestsFixture() {
    rclcpp::shutdown();
  }

 protected:
  double test_duration_;                                  ///< Duration for test execution
  rclcpp::Node::SharedPtr tester_node_;                  ///< Test node instance
  rclcpp::Node::SharedPtr talker_node_;                  ///< Publisher node instance
  rclcpp::Publisher<String>::SharedPtr publisher_;       ///< Message publisher
  rclcpp::TimerBase::SharedPtr timer_;                   ///< Timer for publishing messages
};

////////////////////////////////////////////////
// Test Cases
////////////////////////////////////////////////

/**
 * @brief Test case for verifying publisher functionality
 * @details Tests if the publisher successfully publishes messages that can be received
 */
TEST_CASE_METHOD(MyTestsFixture, "test talker", "[topic]") {
  bool got_topic = false;

  struct ListenerCallback {
    explicit ListenerCallback(bool &got_topic) : got_topic_(got_topic) {}
    
    void operator()(const String msg) const {
      RCLCPP_INFO_STREAM(logger, "I heard: " << msg.data.c_str());
      got_topic_ = true;
    }

    bool &got_topic_;
  };

  auto subscriber = tester_node_->create_subscription<String>(
      "chatter", 10, ListenerCallback(got_topic));

  rclcpp::Rate rate(10.0);
  auto start_time = rclcpp::Clock().now();
  auto duration = rclcpp::Clock().now() - start_time;
  auto timeout = rclcpp::Duration::from_seconds(test_duration_);

  RCLCPP_INFO_STREAM(logger, "duration = " << duration.seconds() 
                     << " timeout=" << timeout.seconds());

  while (!got_topic && (duration < timeout)) {
    rclcpp::spin_some(tester_node_);
    rclcpp::spin_some(talker_node_);  // Make sure to spin the talker node too
    rate.sleep();
    duration = (rclcpp::Clock().now() - start_time);
  }

  RCLCPP_INFO_STREAM(logger, "duration = " << duration.seconds() 
                     << " got_topic=" << got_topic);
  CHECK(got_topic);
} 