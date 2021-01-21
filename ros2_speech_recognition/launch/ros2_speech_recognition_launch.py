from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import PushRosNamespace
import ament_index_python


def generate_launch_description():
    pkg_name = 'ros2_speech_recognition'
    namespace = 'speech_recognition'

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    namespace_cmd = PushRosNamespace(namespace=namespace)

    #
    # ARGS
    #

    stt_grammar = LaunchConfiguration('stt_grammar')
    declare_stt_grammar_cmd = DeclareLaunchArgument(
        'stt_grammar',
        default_value=ament_index_python.get_package_share_directory(
            pkg_name) + '/grammars/example.gram',
        description='Stt node gram file')

    stt_service = LaunchConfiguration('stt_service')
    declare_stt_service_cmd = DeclareLaunchArgument(
        'stt_service',
        default_value='sphinx',
        description='Speech recognition service')

    stt_started = LaunchConfiguration('stt_started')
    declare_stt_started_cmd = DeclareLaunchArgument(
        'stt_started',
        default_value='false',
        description='Is stt started?')

    parser_grammar = LaunchConfiguration('parser_grammar')
    declare_parser_grammar_cmd = DeclareLaunchArgument(
        'parser_grammar',
        default_value=ament_index_python.get_package_share_directory(
            pkg_name) + '/grammars/example.gram',
        description='Parser node gram file')

    #
    # NODES
    #

    stt_node_cmd = Node(
        package=pkg_name,
        executable='stt_node',
        name='stt_node',
        parameters=[{'grammar': stt_grammar},
                    {'service': stt_service},
                    {'started': stt_started}]
    )

    nlp_node_cmd = Node(
        package=pkg_name,
        executable='nlp_node',
        name='nlp_node',
    )

    parser_node_cmd = Node(
        package=pkg_name,
        executable='parser_node',
        name='parser_node',
        parameters=[{'grammar': parser_grammar}]
    )

    dialog_manager_node_cmd = Node(
        package=pkg_name,
        executable='dialog_manager_node',
        name='dialog_manager_node',
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(namespace_cmd)

    ld.add_action(declare_stt_grammar_cmd)
    ld.add_action(declare_stt_service_cmd)
    ld.add_action(declare_stt_started_cmd)
    ld.add_action(declare_parser_grammar_cmd)

    ld.add_action(stt_node_cmd)
    ld.add_action(nlp_node_cmd)
    ld.add_action(parser_node_cmd)
    ld.add_action(dialog_manager_node_cmd)

    return ld
