from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='Pose',
            executable='spot_controller',
            name = 'spot_controller',
            output = 'screen'
        ),
        Node(
            package='Pose',
            executable='spot_show',
            name = 'spot_show',
            output = 'screen'
        ),
        Node(
            package='Pose',
            executable='spot_show3d',
            name = 'spot_show3d',
            output = 'screen'
        ),
        # Node(
        #     package='joy',
        #     executable='joy_node',
        #     name = 'joy',
        #     output = 'screen'
        # ),
    ]
    )
