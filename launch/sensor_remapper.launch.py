#!/usr/bin/env python
"""
Launch file for sensor/pose imu remapper nodes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    # micro_ros_agent = ExecuteProcess(
    #     cmd=[[
    #         'micro-ros-agent udp4 --port 8888 -v '
    #     ]],
    #     shell=True
    # )

    sensor_imu_remapper_node = Node(
        package='ros_com_util',
        executable='sensor_imu_remapper',
        output='screen',
        shell=True,
    )

    pose_imu_remapper_node = Node(
        package='ros_com_util',
        executable='pose_imu_remapper',
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        #micro_ros_agent,
        sensor_imu_remapper_node,
        pose_imu_remapper_node
    ])
