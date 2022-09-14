from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='eua_control',
            namespace='/a/tool',
            executable='controller',
            name='eua_control_a',
            parameters=[
                {'prefix': 'a_tool_'},
                {'simulated': False},
                {'type': 2},
                {'port': '/dev/ttyUSB0'},
                {'baud_rate': 2000000},
                {'joint_device_mapping': {'roll': 23, 'pitch': 21, 'yaw1': 22, 'yaw2': 20}},
                {'homing_load_thresholds': [0.13, 0.18, 0.13, 0.13]},
            ],
            output='screen',
        ),
        Node(
            package='eua_control',
            namespace='/b/tool',
            executable='controller',
            name='eua_control_b',
            parameters=[
                {'prefix': 'b_tool_'},
                {'simulated': False},
                {'type': 1},
                {'port': '/dev/ttyACM0'},
                {'baud_rate': 1000000},
                {'joint_device_mapping': {'roll': 1, 'pitch': 2, 'yaw1': 4, 'yaw2': 3}},
                {'homing_load_thresholds': [0.16, 0.16, 0.13, 0.13]},
            ],
            output='screen',
        ),
    ])