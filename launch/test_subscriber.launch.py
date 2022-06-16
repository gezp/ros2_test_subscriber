from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # param
    use_dedicated_thread = LaunchConfiguration('use_dedicated_thread')

    # declare
    declare_use_dedicated_thread = DeclareLaunchArgument(
        'use_dedicated_thread', default_value="False")

    # launch composed node
    composed_node = Node(
        package='ros2_test_subscriber',
        executable='test_subscriber',
        parameters=[
            {'use_dedicated_thread': use_dedicated_thread}, 
        ],
        output="screen")

    ld = LaunchDescription()
    ld.add_action(declare_use_dedicated_thread)
    ld.add_action(composed_node)
    return ld