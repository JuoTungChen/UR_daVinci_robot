from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_path
import xacro


def generate_mops_urdf():
    xacro_path = get_package_share_path('mops_description') / 'urdf' / 'mops.urdf.xacro'
    doc = xacro.process_file(xacro_path)
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
            namespace='/a/tool',
            parameters=[{
                'type': 2,
                'prefix': 'a_tool_',
                'joint_device_mapping': {'roll': 13, 'pitch': 10, 'yaw1': 12, 'yaw2': 11},
                'simulated': True,
            }],
        ),
        Node(package='eua_control',
            executable='controller',
            namespace='/b/tool',
            parameters=[{
                'type': 1,
                'prefix': 'b_tool_',
                'joint_device_mapping': {'roll': 1, 'pitch': 2, 'yaw1': 4, 'yaw2': 3},
                'simulated': True,
            }],
        ),
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
                # ComposableNode(
                #     package='robot_state_publisher',
                #     plugin='robot_state_publisher::RobotStatePublisher',
                #     name='robot_state_publisher',
                #     parameters=[{'robot_description': robot_description}],
                #     # extra_arguments=[{'use_intra_process_comms': True}],
                # ),
                ComposableNode(
                    package='ur_rtde_ros',
                    plugin='ur_rtde_ros::URReceiverNode',
                    name='ur_receiver',
                    namespace='/a/ur',
                    parameters=[{
                        'hostname': 'ursim-1',
                        'prefix': 'a_ur_',
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='ur_rtde_ros',
                    plugin='ur_rtde_ros::URControllerNode',
                    name='ur_controller',
                    namespace='/a/ur',
                    parameters=[{
                        'hostname': 'ursim-1',
                        'prefix': 'a_ur_',
                        'servo_rate_hz': 125.0,
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='ur_rtde_ros',
                    plugin='ur_rtde_ros::URReceiverNode',
                    name='ur_receiver',
                    namespace='/b/ur',
                    parameters=[{
                        'hostname': 'ursim-2',
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
                        'hostname': 'ursim-2',
                        'prefix': 'b_ur_',
                        'servo_rate_hz': 125.0,
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='mops_control',
                    plugin='mops_control::MopsControlNode',
                    name='mops_control',
                    namespace='/a',
                    parameters=[{
                        'robot_description': robot_description,
                        'ur_prefix': 'a_ur_',
                        'tool_prefix': 'a_tool_',
                        'weights_joint_space': [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1, 1, 1],
                    }],
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
                        'weights_joint_space': [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 1, 1, 1],
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='mops_teleop',
                    plugin='mops_teleop::TeleopNode',
                    name='teleop',
                    namespace='/a',
                    parameters=[{
                        'publish_rate': 125.0,
                        'ur_prefix': 'a_ur_',
                        'haptic_prefix': 'touch_right_',
                        'grasp_rate': 1.5708,
                    }],
                    remappings=[
                        ('haptic_pose', '/touch/right/pose_stylus_current'),
                        ('haptic_buttons', '/touch/right/button_event'),
                        ('clutch_engaged', '/pedal/right'),
                        ('ee_state_desired', 'servo_joint_ik'),
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
                ComposableNode(
                    package='mops_teleop',
                    plugin='mops_teleop::TeleopNode',
                    name='teleop',
                    namespace='/b',
                    parameters=[{
                        'publish_rate': 125.0,
                        'ur_prefix': 'b_ur_',
                        'haptic_prefix': 'touch_left_',
                        'grasp_rate': 1.5708,
                    }],
                    remappings=[
                        ('haptic_pose', '/touch/left/pose_stylus_current'),
                        ('haptic_buttons', '/touch/left/button_event'),
                        ('clutch_engaged', '/pedal/right'),
                        ('ee_state_desired', 'servo_joint_ik'),
                    ],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
        ),
    ])
