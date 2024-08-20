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
            name='ads_config_topic',
            default_value='',
            description='Topic name for ADS Configuration. If empty then yaml files will be used'),
        DeclareLaunchArgument(
            "robot_station",
            default_value="",
            choices=["", "ob00", "ob03", "ob05", "ob09"],
            description="This is the robot station desired",
        ),
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
                executable='ads_com_node',
                output='screen',
                parameters=[
                    {'ads_config_topic': LaunchConfiguration('ads_config_topic')},
                    {'robot_station': LaunchConfiguration('robot_station')},
                    {'com_config': PathJoinSubstitution([package_path, LaunchConfiguration('com_config')])},
                    {'plc_admin': PathJoinSubstitution([package_path, LaunchConfiguration('plc_admin')])}
                ],
            )
        ]),
    ])
