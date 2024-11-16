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
find_package(example_interfaces REQUIRED)  
find_package(tf2 REQUIRED)           
find_package(tf2_ros REQUIRED)       
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(catch_ros2 REQUIRED)

# Include directories
include_directories(include)

# Create the publisher executable
add_executable(talker 
    src/main.cpp
    src/simple_publisher.cpp
)
ament_target_dependencies(
  talker 
  rclcpp 
  std_msgs
  example_interfaces
  geometry_msgs
  tf2
  tf2_ros
)

# Add explicit linking for example_interfaces
target_link_libraries(talker
  ${example_interfaces_LIBRARIES}  
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
  tf2              
  tf2_ros          
  geometry_msgs    
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  # Add the test executable
  add_executable(test_publisher
    test/test_publisher.cpp
  )
  
  # Update include directories to find all headers
  target_include_directories(test_publisher PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${catch_ros2_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${std_srvs_INCLUDE_DIRS}
  )
  
  ament_target_dependencies(test_publisher
    rclcpp
    std_msgs
    std_srvs
    catch_ros2
  )

  # Install the test executable
  install(TARGETS
    test_publisher
    DESTINATION lib/${PROJECT_NAME}
  )

  # Register the test
  add_test(
    NAME test_publisher_test
    COMMAND test_publisher
  )
endif()

ament_package()
