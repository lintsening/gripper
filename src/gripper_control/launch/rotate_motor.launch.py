from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gripper_control',
            executable='servo_control_node',
            name='rotate_motor_node',
            parameters=[
                {'gpio_pin': 19},
                {'topic_name': '/gripper/rotate_motor_angle'}
            ]
        )
    ])