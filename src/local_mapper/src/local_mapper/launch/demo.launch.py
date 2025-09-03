from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='local_mapper', executable='local_mapper_node',
             name='local_mapper', output='screen'),
        Node(package='rviz2', executable='rviz2',
             name='rviz2', output='screen'),
    ])
