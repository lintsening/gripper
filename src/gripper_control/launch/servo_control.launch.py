#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package = 'gripper_control',
            executable = 'servo_control_node',
            name = 'servo_control_node',
            output = 'screen',
            )
        ])
