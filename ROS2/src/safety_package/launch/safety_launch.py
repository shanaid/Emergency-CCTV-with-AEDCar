from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='safety_package',
            node_executable='odom',
            node_name='odom'
        ),
        Node(
            package='safety_package',
            node_executable='ftc',
            node_name='follow_the_carrot'
        ),
        Node(
            package='safety_package',
            node_executable='load_map',
            node_name='load_map'
        ),
        Node(
            package='safety_package',
            node_executable='a_star_global',
            node_name='a_star_global'
        ),
        Node(
            package='safety_package',
            node_executable='a_star_local',
            node_name='a_star_local'
        ),
        Node(
            package='safety_package',
            node_executable='socket',
            node_name='data_socket'
        ),

    ])