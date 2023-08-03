from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state',
            executable='sensor_data',
            name = 'sensor_data',
            output = 'screen'
        ),
        Node(
            package='robot_state',
            executable='motors_start',
            name = 'motor_board',
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
