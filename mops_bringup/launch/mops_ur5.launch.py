from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_path
import xacro

def generate_mops_urdf():
    xacro_path = get_package_share_path('mops_description') / 'urdf' / 'mops_sdu_ur5.urdf.xacro'
    bringup_config_path = get_package_share_path('mops_bringup') / 'config'
    mappings = {
        # 'a_kinematics_params_file': str(bringup_config_path / 'ur-20225502173.yaml'),
        'b_kinematics_params_file': str(bringup_config_path / 'ur-2017356413.yaml'),
    }
    doc = xacro.process_file(xacro_path, mappings=mappings)
    robot_description = doc.toxml()  # doc.toprettyxml(indent='  ')
    return robot_description


def generate_launch_description():
    
    robot_description = generate_mops_urdf()
    return LaunchDescription([
        Node(package='mops_state_publisher',
            executable='mops_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(package='eua_control',
            executable='controller',
            namespace='/b/tool',
            parameters=[{
                'type': 2,
                'port': '/dev/ttyUSB0',
                'baud_rate': 2000000,
                'prefix': 'b_tool_',
                'joint_device_mapping': {'roll': 20, 'pitch': 23, 'yaw1': 22, 'yaw2': 21},
                'homing_load_thresholds': [0.08, 0.07, 0.07, 0.06],
            }],
        ),
        # Node(package='teleop_trigger',
        #     executable='trigger_node',
        #     namespace='/b',
        #     parameters=[{
        #         'port_name': '/dev/ttyUSB2',
        #         'trigger_motor_id': 10,
        #         'grasp_angle_ratio': 1.5,
        #     }],
        # ),             
        Node(package='mops_teleop',
            executable='foot_control',
            namespace='/pedal',
        ),
        ComposableNodeContainer(
            name='mops_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            # prefix=['gdbserver localhost:3000'],
            output='screen',
            composable_node_descriptions=[

                ComposableNode(
                    package='ur_rtde_ros',
                    plugin='ur_rtde_ros::URReceiverNode',
                    name='ur_receiver',
                    namespace='/b/ur',
                    parameters=[{
                        'hostname': '192.168.1.51',
                        # 'hostname': 'ur-2017357220',
                        'prefix': 'b_ur_',
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='ur_rtde_ros',
                    plugin='ur_rtde_ros::URControllerNode',
                    name='ur_controller',
                    namespace='/b/ur',
                    parameters=[{
                        # 'hostname': 'ur-2017357220',
                        'hostname': '192.168.1.51',
                        'prefix': 'b_ur_',
                        'servo_rate_hz': 125.0,
                    }],
                    remappings=[
                        ('teach_mode_enable', '/pedal/left'),
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                
                ComposableNode(
                    package='mops_control',
                    plugin='mops_control::MopsControlNode',
                    name='mops_control',
                    namespace='/b',
                    parameters=[{
                        'robot_description': robot_description,
                        'ur_prefix': 'b_ur_',
                        'tool_prefix': 'b_tool_',
                        'enable_rcm': True,
                        'weights_joint_space': [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1, 1, 1],
                        'k_task': 1.5,
                        'k_rcm': 5.0,
                        # 'task_null':,
                        'k_p': 1.0,
                        # 'k_i': ,
                        # 'k_d': ,
                        # 'position_only': ,
                        'k_pos': 1.5,
                        'k_ori': 1.5,
                        'dt': 0.2,
                        'thres': 1e-3,
                        'max_iter': 10,
                    }],                    
                    # remappings=[
                    #     ('/move_joint', '/move_joint_default'),
                    #     ('/servo_joint', '/servo_joint_default'),
                    # ],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                
                # ComposableNode(
                #     package='mops_teleop',
                #     plugin='mops_teleop::TeleopNode',
                #     name='teleop',
                #     namespace='/b',
                #     parameters=[{
                #         'publish_rate': 125.0,
                #         'ur_prefix': 'b_ur_',
                #         'haptic_prefix': 'touch_',
                #         'grasp_rate': 1.5708,
                #     }],
                #     remappings=[
                #         ('haptic_pose', '/touch/pose_stylus_current'),
                #         ('haptic_buttons', '/touch/button_event'),
                #         ('clutch_engaged', '/pedal/right'),
                #         ('rotation_tracking_engaged', '/pedal/middle'),
                #         ('ee_state_desired', 'servo_joint_ik'),
                #     ],
                #     extra_arguments=[{'use_intra_process_comms': True}],
                # ),
            ],
        )
    ])
