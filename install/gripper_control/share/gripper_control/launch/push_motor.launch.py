from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gripper_control',
            executable='servo_control_node',
            name='push_motor_node',
            parameters=[
                {'gpio_pin': 18},
                {'topic_name': '/gripper/push_motor_angle'}
            ]
        )
    ])