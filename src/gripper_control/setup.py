from setuptools import setup

package_name = 'gripper_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
    'launch/push_motor.launch.py',
    'launch/rotate_motor.launch.py',
    'launch/gripper.launch.py'
]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Servo motor controller for gas pump gripper using Raspberry Pi',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_control_node = gripper_control.servo_control_node:main',
            'servo_controller = gripper_control.servo_controller:main', 
        ],
    },
)