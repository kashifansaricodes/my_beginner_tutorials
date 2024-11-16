/**
 * @file main.cpp
 * @author Kashif Ansari
 * @brief Main entry point for the publisher node
 * @version 0.1
 * @date 2024-02-08
 * 
 * @copyright Copyright (c) 2024
 */

#include "simple_publisher.hpp"

/**
 * @brief Main function to initialize and run the publisher node
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return int Exit status
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::shutdown();
    return 0;
} 