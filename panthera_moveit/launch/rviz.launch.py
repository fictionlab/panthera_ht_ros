from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    panthera_moveit_path = FindPackageShare('panthera_moveit')

    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([
            panthera_moveit_path,
            'config',
            'moveit.rviz'
        ]),
        description='Path to the RViz config file'
    )

    moveit_config = MoveItConfigsBuilder(
        'panthera_ht',
        package_name='panthera_moveit'
    ).to_moveit_configs()

    rviz_params = [
        moveit_config.to_dict(),
        {'use_sim_time': False}
    ]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        parameters=rviz_params,
    )

    return LaunchDescription([
        rviz_config_file_arg,
        rviz_node,
    ])
