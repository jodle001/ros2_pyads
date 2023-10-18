from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory('ros2_pyads')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            name='com_config',
            default_value='config/com_config.yaml',
            description='Path to ADS Configuration file'),
        DeclareLaunchArgument(
            name='plc_admin',
            default_value='config/plc_admin.yaml',
            description='Path to Admin Configuration file'),

        # EIP bridge simulator node (if simulation is true)
        GroupAction([
            Node(
                package='ros2_pyads',
                executable='ads_com_bool_test_node',
                output='screen',
                parameters=[
                    {'com_config': PathJoinSubstitution([package_path, LaunchConfiguration('com_config')])},
                    {'plc_admin': PathJoinSubstitution([package_path, LaunchConfiguration('plc_admin')])}
                ],
            )
        ]),
    ])
