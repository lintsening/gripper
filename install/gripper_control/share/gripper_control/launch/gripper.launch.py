from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gripper_control',
            executable='servo_controller',
            name='servo_rotate_controller',
            output='screen',
            parameters=[{'gpio_pin': 18, 'topic_name': '/servo_rotate_angle'}]
        ),
        Node(
            package='gripper_control',
            executable='servo_controller',
            name='servo_press_controller',
            output='screen',
            parameters=[{'gpio_pin': 19, 'topic_name': '/servo_press_angle'}]
        ),
    ])