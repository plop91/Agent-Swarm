from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

default_stack = 'correctness_stack'
package_name = 'agent_swarm'
default_executable = 'agent'
default_output = 'screen'
default_emulate_tty = True


def generate_launch_description():

    stack = LaunchConfiguration('stack')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')

    stack_launch_arg = DeclareLaunchArgument(
        'stack',
        default_value=default_stack
    )
    input_topic_launch_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='input'
    )
    output_topic_launch_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='output'
    )

    correctness_verifier_node = Node(
        namespace=stack,
        package=package_name,
        executable=default_executable,
        name='correctness_verifier',
        output=default_output,
        emulate_tty=default_emulate_tty,
        parameters=[
            {
                'agent_name': 'correctness_verifier',
                'input_topic': 'correctness_test_agent',
                'output_topic': output_topic
            }
        ]
    )

    correctness_test_node = Node(
        namespace=stack,
        package=package_name,
        executable=default_executable,
        name='correctness_test_agent',
        output=default_output,
        emulate_tty=default_emulate_tty,
        parameters=[
            {
                'agent_name': 'correctness_test_agent',
                'input_topic': input_topic,
                'output_topic': 'correctness_test_agent'
            }
        ]
    )

    return LaunchDescription([
        stack_launch_arg,
        input_topic_launch_arg,
        output_topic_launch_arg,
        correctness_verifier_node,
        correctness_test_node
    ])
