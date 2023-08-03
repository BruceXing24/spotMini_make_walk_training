from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spot_controller',
            executable='spot_controller',
            name = 'spot_controller',
            output = 'screen'
        ),
        Node(
            package='spot_controller',
            executable='spot_showpb',
            name = 'spot_showpb',
            output = 'screen'
        ),
        Node(
            package='spot_controller',
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
