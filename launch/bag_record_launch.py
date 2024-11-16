#!/usr/bin/env python3

import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Get the current timestamp for bag file naming
    timestamp = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    
    # Create the results directory in your package
    results_dir = 'results'
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
    
    # Define the bag file path
    bag_path = os.path.join(results_dir, f'rosbag2_{timestamp}')

    # Declare the record_bag argument
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='true',
        description='Whether to record all topics to a bag file'
    )

    # Create the talker node
    talker_node = Node(
        package='beginner_tutorials',
        executable='talker',
        name='talker'
    )

    # Create the bag recording process
    bag_record_process = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record_bag')),
        cmd=['ros2', 'bag', 'record', '-a', '-o', bag_path],
        shell=True,
        output='screen'
    )

    return LaunchDescription([
        record_bag_arg,
        talker_node,
        bag_record_process
    ]) 