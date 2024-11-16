/**
 * @file main.cpp
 * @author Kashif Ansari
 * @brief Main entry point for the publisher node
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