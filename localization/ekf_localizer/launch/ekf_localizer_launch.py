from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ekf_localizer',
            executable='ekf_localizer_node',
            name='ekf_localizer',
            output='screen',
            parameters=['/home/wipod_orin/wipod_ws/src/wipod_ws/localization/ekf_localizer/params/ekf_localizer_params.yaml']
            

        )
    ])