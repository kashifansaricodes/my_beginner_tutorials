# launch/tutorial_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    publish_freq_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='2.0',
        description='Publishing frequency in Hz for /chatter topic'
    )

    # Create the talker node with the parameter
    talker_node = Node(
        package='beginner_tutorials',
        executable='talker',
        name='talker',
        parameters=[{
            'publish_frequency': LaunchConfiguration('publish_frequency')
        }],
        output='screen'
    )

    # Create the listener node
    listener_node = Node(
        package='beginner_tutorials',
        executable='listener',
        name='listener',
        output='screen'
    )

    # Return the launch description
    return LaunchDescription([
        publish_freq_arg,
        talker_node,
        listener_node
    ])
