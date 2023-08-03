from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state',
            executable='imu_data',
            name = 'imu_data',
            output = 'screen'
        ),
        Node(
            package='robot_state',
            executable='motors_start',
            name = 'motor_board',
            output = 'screen'
        ),

        Node(
             package='robot_state',
             executable='PPO_controller',
             name = 'PPO_controller',
             output = 'screen'
         ),
    ]
    )
