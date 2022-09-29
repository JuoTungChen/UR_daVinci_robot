from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
import xacro


def generate_mops_urdf():
    xacro_path = get_package_share_path('mops_description') / 'urdf' / 'mops.urdf.xacro'
    # bringup_config_path = get_package_share_path('mops_bringup') / 'config'
    # mappings = {
    #     'world_calib_file': str(bringup_config_path / 'world_calib.yaml'),
    # }
    doc = xacro.process_file(xacro_path)  # mappings=mappings
    robot_description = doc.toxml()  # doc.toprettyxml(indent='  ')
    return robot_description


def generate_launch_description():
    robot_description = generate_mops_urdf()

    return LaunchDescription([
        Node(package='mops_state_publisher',
            executable='mops_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        #  Arm A: UR5e + EUA2 (X/Xi tool)
        Node(package='ur_rtde_ros',
            executable='ur_control',
            namespace='/a/ur',
            parameters=[{
                'hostname': 'ur-20185500393',
                'servo_rate_hz': 125.0,
                'prefix': 'a_ur_',
            }],
            remappings=[
                ('teach_mode_enable', '/pedal/left'),
            ],
        ),
        Node(package='eua_control',
            executable='controller',
            namespace='/a/tool',
            parameters=[{
                'type': 2,
                'port': '/dev/ftdi_FT4NNYP8',
                'baud_rate': 2000000,
                'prefix': 'a_tool_',
                'joint_device_mapping': {'roll': 23, 'pitch': 21, 'yaw1': 22, 'yaw2': 20},
                'homing_load_thresholds': [0.13, 0.18, 0.13, 0.13],
            }],
        ),
        Node(package='mops_control',
            executable='mops_control',
            namespace='/a',
            parameters=[{
                'robot_description': robot_description,
                'ur_prefix': 'a_ur_',
                'tool_prefix': 'a_tool_',
                'weights_joint_space': [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1, 1, 1],
            }],
        ),
        Node(package='mops_teleop',
            executable='teleop',
            namespace='/a',
            parameters=[{
                'publish_rate': 125.0,
                'ur_prefix': 'a_ur_',
                'haptic_prefix': 'touch_right_',
                'grasp_rate': 1.5708,
            }],
            remappings=[
                ('haptic_pose', '/touch/left/pose_stylus_current'),
                ('haptic_buttons', '/touch/left/button_event'),
                ('clutch_engaged', '/pedal/right'),
                ('ee_state_desired', 'servo_joint_ik'),
            ],
        ),
        # Arm B: UR5 + EUA1 (S/Si tool)
        Node(package='ur_rtde_ros',
            executable='ur_control',
            namespace='/b/ur',
            parameters=[{
                'hostname': 'ur-2017356413',
                'servo_rate_hz': 125.0,
                'prefix': 'b_ur_',
            }],
            remappings=[
                ('teach_mode_enable', '/pedal/left'),
            ],
        ),
        Node(package='eua_control',
            executable='controller',
            namespace='/b/tool',
            parameters=[{
                'type': 1,
                'port': '/dev/ttyACM0',
                'baud_rate': 1000000,
                'prefix': 'b_tool_',
                'joint_device_mapping': {'roll': 1, 'pitch': 2, 'yaw1': 4, 'yaw2': 3},
                'homing_load_thresholds': [0.16, 0.16, 0.13, 0.13],
            }],
        ),
        Node(package='mops_control',
            executable='mops_control',
            namespace='/b',
            parameters=[{
                'robot_description': robot_description,
                'ur_prefix': 'b_ur_',
                'tool_prefix': 'b_tool_',
                'weights_joint_space': [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1, 1, 1],
            }],
        ),
        Node(package='mops_teleop',
            executable='teleop',
            namespace='/b',
            parameters=[{
                'publish_rate': 125.0,
                'ur_prefix': 'b_ur_',
                'haptic_prefix': 'touch_right_',
                'grasp_rate': 1.5708,
            }],
            remappings=[
                ('haptic_pose', '/touch/right/pose_stylus_current'),
                ('haptic_buttons', '/touch/right/button_event'),
                ('clutch_engaged', '/pedal/right'),
                ('ee_state_desired', 'servo_joint_ik'),
            ],
        ),
        # HID devices
        Node(package='mops_teleop',
            executable='foot_control',
            namespace='/pedal',
        ),
        Node(package='touch_control',
            executable='touch_control',
            namespace='/touch/left',
            parameters=[{
                'device_name': 'left',
                'prefix': 'touch_left_',
            }],
        ),
        Node(package='touch_control',
            executable='touch_control',
            namespace='/touch/right',
            parameters=[{
                'device_name': 'right',
                'prefix': 'touch_right_',
            }],
        ),
    ])
