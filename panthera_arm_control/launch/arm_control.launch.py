from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config_file = PathJoinSubstitution([
        FindPackageShare('panthera_arm_control'),
        'config',
        '6dof_arm_params.yaml',
    ])
    default_urdf_file = PathJoinSubstitution([
        FindPackageShare('panthera_description'),
        'urdf',
        'panthera_ht_gripper.urdf',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config_file,
                              description='Path to robot config YAML.'),
        DeclareLaunchArgument('urdf_file', default_value=default_urdf_file,
                              description='Path to robot URDF file.'),
        DeclareLaunchArgument('status_publish_rate', default_value='20.0',
                              description='Status publish rate in Hz'),
        DeclareLaunchArgument('max_velocity', default_value='0.5',
                              description='Max joint velocity in rad/s'),
        DeclareLaunchArgument('launch_joy_teleop', default_value='false',
                      description='Include joy_teleop.launch.py alongside arm_control_node'),

        Node(
            package='panthera_arm_control',
            executable='arm_control_node',
            name='arm_control_node',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'urdf_file': LaunchConfiguration('urdf_file'),
                'status_publish_rate': LaunchConfiguration('status_publish_rate'),
                'max_velocity': LaunchConfiguration('max_velocity'),
                'max_torque': [21.0, 36.0, 36.0, 21.0, 10.0, 10.0],
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('panthera_arm_control'),
                    'launch',
                    'joy_teleop.launch.py',
                ])
            ),
            condition=IfCondition(LaunchConfiguration('launch_joy_teleop')),
        ),
    ])
