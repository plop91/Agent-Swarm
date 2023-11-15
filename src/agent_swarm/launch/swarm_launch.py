from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution


def generate_launch_description():

    top_level_council = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('agent_swarm'),
                'top_level_council_launch.py'
            ])
        ]),
        launch_arguments={
            'stack': 'top_level_council',
        }.items()
    )

    correctness_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('agent_swarm'),
                'correctness_stack_launch.py'
            ])
        ]),
        launch_arguments={
            'stack': 'correctness_stack',
        }.items()
    )

    return LaunchDescription([
        correctness_stack
    ])
