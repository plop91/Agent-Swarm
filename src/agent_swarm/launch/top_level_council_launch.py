from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

default_stack = 'top_level_council'
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

    launch_list = [stack_launch_arg,
                   input_topic_launch_arg, output_topic_launch_arg]

    top_level_condenser = Node(
        namespace=stack,
        package=package_name,
        executable=default_executable,
        name='top_level_condenser',
        output=default_output,
        emulate_tty=default_emulate_tty,
        parameters=[
            {
                'agent_category': 'top_level_council',
                'agent_name': 'top_level_condenser',
                'input_topic': input_topic,
                'output_topic': output_topic
            }
        ]
    )
    launch_list.append(top_level_condenser)

    agent_creator = Node(
        namespace=stack,
        package=package_name,
        executable=default_executable,
        name='agent_creator',
        output=default_output,
        emulate_tty=default_emulate_tty,
        parameters=[
            {
                'agent_category': 'top_level_council',
                'agent_name': 'agent_creator',
                'input_topic': input_topic,
                'output_topic': 'top_level_condenser'
            }
        ]
    )
    launch_list.append(agent_creator)

    agent_evaluator = Node(
        namespace=stack,
        package=package_name,
        executable=default_executable,
        name='agent_evaluator',
        output=default_output,
        emulate_tty=default_emulate_tty,
        parameters=[
            {
                'agent_category': 'top_level_council',
                'agent_name': 'agent_evaluator',
                'input_topic': input_topic,
                'output_topic': 'top_level_condenser'
            }
        ]
    )
    launch_list.append(agent_evaluator)

    agent_deployer = Node(
        namespace=stack,
        package=package_name,
        executable=default_executable,
        name='agent_deployer',
        output=default_output,
        emulate_tty=default_emulate_tty,
        parameters=[
            {
                'agent_category': 'top_level_council',
                'agent_name': 'agent_deployer',
                'input_topic': input_topic,
                'output_topic': 'top_level_condenser'
            }
        ]
    )
    launch_list.append(agent_deployer)

    agent_terminator = Node(
        namespace=stack,
        package=package_name,
        executable=default_executable,
        name='agent_terminator',
        output=default_output,
        emulate_tty=default_emulate_tty,
        parameters=[
            {
                'agent_category': 'top_level_council',
                'agent_name': 'agent_terminator',
                'input_topic': input_topic,
                'output_topic': 'top_level_condenser'
            }
        ]
    )
    launch_list.append(agent_terminator)

    agent_monitor = Node(
        namespace=stack,
        package=package_name,
        executable=default_executable,
        name='agent_monitor',
        output=default_output,
        emulate_tty=default_emulate_tty,
        parameters=[
            {
                'agent_category': 'top_level_council',
                'agent_name': 'agent_monitor',
                'input_topic': input_topic,
                'output_topic': 'top_level_condenser'
            }
        ]
    )
    launch_list.append(agent_monitor)

    role_manager = Node(
        namespace=stack,
        package=package_name,
        executable=default_executable,
        name='role_manager',
        output=default_output,
        emulate_tty=default_emulate_tty,
        parameters=[
            {
                'agent_category': 'top_level_council',
                'agent_name': 'role_manager',
                'input_topic': input_topic,
                'output_topic': 'top_level_condenser'
            }
        ]
    )
    launch_list.append(role_manager)

    permission_manager = Node(
        namespace=stack,
        package=package_name,
        executable=default_executable,
        name='permission_manager',
        output=default_output,
        emulate_tty=default_emulate_tty,
        parameters=[
            {
                'agent_category': 'top_level_council',
                'agent_name': 'permission_manager',
                'input_topic': input_topic,
                'output_topic': 'top_level_condenser'
            }
        ]
    )
    launch_list.append(permission_manager)

    security_manager = Node(
        namespace=stack,
        package=package_name,
        executable=default_executable,
        name='security_manager',
        output=default_output,
        emulate_tty=default_emulate_tty,
        parameters=[
            {
                'agent_category': 'top_level_council',
                'agent_name': 'security_manager',
                'input_topic': input_topic,
                'output_topic': 'top_level_condenser'
            }
        ]
    )
    launch_list.append(security_manager)

    data_manager = Node(
        namespace=stack,
        package=package_name,
        executable=default_executable,
        name='data_manager',
        output=default_output,
        emulate_tty=default_emulate_tty,
        parameters=[
            {
                'agent_category': 'top_level_council',
                'agent_name': 'data_manager',
                'input_topic': input_topic,
                'output_topic': 'top_level_condenser'
            }
        ]
    )
    launch_list.append(data_manager)

    data_analyzer = Node(
        namespace=stack,
        package=package_name,
        executable=default_executable,
        name='data_analyzer',
        output=default_output,
        emulate_tty=default_emulate_tty,
        parameters=[
            {
                'agent_category': 'top_level_council',
                'agent_name': 'data_analyzer',
                'input_topic': input_topic,
                'output_topic': 'top_level_condenser'
            }
        ]
    )
    launch_list.append(data_analyzer)

    return LaunchDescription(launch_list)
