from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace='test_swarm',
            package='agent_swarm',
            executable='agent',
            name='test_agent_creator',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'agent_name': 'test_agent_creator',
                    'listening_agent': 'input'
                }
            ]
        ),
        Node(
            namespace='test_swarm',
            package='agent_swarm',
            executable='agent',
            name='correctness_test_agent',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'agent_name': 'correctness_test_agent',
                    'listening_agent': 'test_agent_creator'
                }
            ]
        )
    ])
