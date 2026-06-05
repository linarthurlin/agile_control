import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. PACMod2 Driver Node
    pacmod2_driver_node = Node(
        package='pacmod2',
        executable='pacmod2_driver',
        name='pacmod2_driver',
        output='screen',
        parameters=[{
            'use_socketcan': True,
            'socketcan_device': 'can0'
        }]
    )

    # 2. PACMod Twist Bridge Node
    twist_bridge_node = Node(
        package='pacmod_twist_bridge',
        executable='cmd_vel_to_pacmod2.py',
        name='cmd_vel_to_pacmod2',
        output='screen',
        parameters=[{
            'cmd_vel_topic': '/cmd_vel',
            'speed_rpt_topic': '/pacmod/vehicle_speed_rpt'
        }]
    )

    return LaunchDescription([
        pacmod2_driver_node,
        twist_bridge_node
    ])
