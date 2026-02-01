from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument 
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:
    # # Match the "mindset" by using LaunchConfiguration for the config path
    # convoy_config_val = LaunchConfiguration('convoy_config')
    # default_dwm_config_path = PathJoinSubstitution([
    #     FindPackageShare('dwm1001_launch'),
    #     'config',
    #     'convoy_params.yaml'
    # ])

    # convoy_config_launch_arg = DeclareLaunchArgument(
    #     'convoy_config',
    #     default_value=default_dwm_config_path,
    #     description='Configuration file for the convoy logic node'
    # )

    # convoy_logic_node = Node(
    #     package="dwm1001_driver",
    #     executable="convoy_range",
    #     #name="convoy_range_finder",
    #     parameters=[convoy_config_val],
    #     #output="screen"
    # )

    return LaunchDescription([
        Node(
            package='dwm1001_driver',
            executable='convoy_range',
            name='convoy_range_finder',
            output='screen',
            # We hardcode parameters here for now to avoid YAML debugging
            parameters=[{
                'd1': 0.5,
                'd2': 0.5
            }]
        )
    ])