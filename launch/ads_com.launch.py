from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    package_path = get_package_share_directory('ros2_pyads')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('sim', default_value='true', description='Simulation flag'),
        DeclareLaunchArgument('pub_rate', default_value='10', description='Rate of topic publisher (Hz)'),
        DeclareLaunchArgument('config', default_value='example/eip_config_2.yaml', description='Path to EIP config file'),
        DeclareLaunchArgument('ip', default_value='192.168.0.1', description='IP address of PLC'),

        # EIP bridge simulator node (if simulation is true)
        GroupAction([
            Node(
                package='eip_bridge',
                executable='eip_bridge_simulator_node',
                # name='eip_bridge_simulator',
                output='screen',
                parameters=[
                    {'pub_rate': LaunchConfiguration('pub_rate')},
                    {'config': PathJoinSubstitution([package_path, LaunchConfiguration('config')])}
                ],
                condition=IfCondition(LaunchConfiguration('sim'))
            )
        ]),

        # EIP bridge node (if simulation is false)
        GroupAction([
            Node(
                package='eip_bridge',
                executable='eip_bridge_node',
                # name='eip_bridge',
                output='screen',
                parameters=[
                    {'ip': LaunchConfiguration('ip')},
                    {'pub_rate': LaunchConfiguration('pub_rate')},
                    {'config': PathJoinSubstitution([package_path, LaunchConfiguration('config')])}
                ],
                condition=UnlessCondition(LaunchConfiguration('sim'))
            )
        ])
    ])