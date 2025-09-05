from launch import LaunchDescription
from launch_ros.actions import Node

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
import os

def generate_launch_description():
    return LaunchDescription([
        # 设置环境变量，确保没有界面问题（特别是在 VM 中运行时）
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        
        # 启动 local_mapper
        Node(
            package='local_mapper',
            executable='local_mapper_node',
            name='local_mapper',
            output='screen',
            parameters=[{
                'grid_topic': '/local_grid',
                'in_cmd': '/cmd_vel',
                'out_cmd': '/cmd_vel_safe',
                'stop_distance': 0.8,
                'lateral_half': 0.3,
                'occupied_threshold': 50,
                'verbose': True
            }],
            remappings=[('/cmd_vel', '/cmd_vel')]  # 如果需要重命名话题
        ),
        
        # 启动 safety_shield
        Node(
            package='safety_shield',
            executable='safety_shield_node',
            name='safety_shield',
            output='screen',
            parameters=[{
                'grid_topic': '/local_grid',
                'in_cmd': '/cmd_vel',
                'out_cmd': '/cmd_vel_safe',
                'stop_distance': 0.8,
                'lateral_half': 0.3,
                'occupied_threshold': 50,
                'verbose': True
            }],
        ),
        
        # 启动 rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }],
            arguments=['-d', os.path.join(os.getenv('HOME'), 'ros2_ws/src/local_mapper/config/demo.rviz')]  # 你需要提前保存一个 RViz 配置文件
        ),
    ])

