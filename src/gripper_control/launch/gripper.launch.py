#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # rotate 腳的伺服 pin = BCM.18 輸出
        Node(
            package='gripper_control',
            executable='servo_rotate_node',
            name='servo_rotate_controller',
            output='screen',
            parameters=[{
                'gpio_pin': 18,  # 輸出腳位
                'topic_name': '/servo_rotate_angle',  # 目標角度 topic
                'min_pulse_us': 500,
                'max_pulse_us': 2500
            }]
        ),
        # press 腳的伺服 pin = BCM.19 輸出
        Node(
            package='gripper_control',
            executable='servo_press_node',
            name='servo_press_controller',
            output='screen',
            parameters=[{
                'gpio_pin': 19,  # 輸出腳位
                'topic_name': '/servo_press_angle',  # 目標角度 topic
                'min_pulse_us': 500,
                'max_pulse_us': 2500
            }]
        ),
        # rotate 腳的監測 pin = BCM.20 輸入
        Node(
            package='gripper_control',
            executable='monitor_rotate_node',
            name='servo_rotate_monitor',
            output='screen',
            parameters=[{
                'gpio_pin': 20,  # 監測腳位
                'topic_name': '/servo_rotate_angle/actual',  # 實際角度 topic
                'min_pulse_us': 500,
                'max_pulse_us': 2500
            }]
        ),
        # press 腳的監測 pin = BCM.20 輸入
        Node(
            package='gripper_control',
            executable='monitor_press_node',
            name='servo_press_monitor',
            output='screen',
            parameters=[{
                'gpio_pin': 21,  # 監測腳位
                'topic_name': '/servo_press_angle/actual',  # 實際角度 topic
                'min_pulse_us': 500,
                'max_pulse_us': 2500
            }]
        ),
    ])