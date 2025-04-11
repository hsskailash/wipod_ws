from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ekf_localizer',
            executable='ekf_localizer_node',
            name='ekf_localizer',
            output='screen',
            parameters=['/home/bunty/wipod_ws/src/localization/ekf_localizer/params/ekf_localizer_params.yaml']
            

        )
    ])