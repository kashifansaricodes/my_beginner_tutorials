set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)  # Added this line

# Include directories
include_directories(include)

# Create the publisher executable
add_executable(talker src/simple_publisher.cpp)
ament_target_dependencies(talker 
  rclcpp 
  std_msgs
  example_interfaces  # Added this line
)

# Add explicit linking for example_interfaces
target_link_libraries(talker
  ${example_interfaces_LIBRARIES}  # Added this line
)

add_executable(listener src/simple_subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
  
# Install include directories
install(DIRECTORY 
  include/
  DESTINATION include/${PROJECT_NAME}/
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export dependencies
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  example_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
