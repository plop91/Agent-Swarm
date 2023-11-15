from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agent_swarm',
            executable='agent',
            name='agent_1',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'agent_name': 'agent_1',
                    'listening_agent': 'agent_2'
                }
            ]
        ),
        
        Node(
            package='agent_swarm',
            executable='agent',
            name='agent_2',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'agent_name': 'agent_2',
                    'listening_agent': 'agent_1'
                }
            ]
        )
    ])
