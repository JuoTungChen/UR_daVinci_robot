from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_path
import xacro


def generate_mops_urdf():
    xacro_path = get_package_share_path('mops_description') / 'urdf' / 'mops_sdu.urdf.xacro'
    bringup_config_path = get_package_share_path('mops_bringup') / 'config'
    mappings = {
        'a_kinematics_params_file': str(bringup_config_path / 'ur-20225502173.yaml'),
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
        )
    ])
