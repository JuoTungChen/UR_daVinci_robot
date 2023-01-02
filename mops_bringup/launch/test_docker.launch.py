from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
import os


def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                FindExecutable(name='docker'),
                'run',
                '--network host',
                '-v ' + os.environ['HOME'] + '/.3dsystems:/root/.3dsystems',
                'touch-focal',
                'ros2 launch touch_control single_device.launch.py',
            ],
            output='both',
            shell=True,
        ),
        # ExecuteProcess(
        #     cmd=[
        #         FindExecutable(name='echo'),
        #         EnvironmentVariable('HOME'),
        #     ],
        #     output='both',
        # ),
    ])
