from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0',
                              description='Joystick device path'),
        DeclareLaunchArgument('command_rate_hz', default_value='60.0',
                              description='IK command update rate in Hz'),
        DeclareLaunchArgument('linear_speed_mps', default_value='0.045',
                              description='Cartesian linear speed in m/s at full stick'),
        DeclareLaunchArgument('angular_speed_rps', default_value='0.2',
                              description='Angular speed in rad/s at full stick'),
        DeclareLaunchArgument('deadzone', default_value='0.1',
                              description='Joystick deadzone'),
        DeclareLaunchArgument('axis_filter_alpha', default_value='0.25',
                      description='Low-pass filter alpha for joystick axes (0..1)'),
        DeclareLaunchArgument('trigger_deadzone', default_value='0.08',
                  description='Deadzone applied to each trigger after normalization'),
        DeclareLaunchArgument('motion_epsilon', default_value='0.002',
                  description='Minimum filtered axis magnitude to command motion'),
        DeclareLaunchArgument('trigger_mode', default_value='signed',
                  description='Trigger axis convention: signed|unsigned'),

        Node(
            package='panthera_arm_control',
            executable='arm_joy_teleop_node',
            name='arm_joy_teleop_node',
            output='screen',
            parameters=[{
                'joy_topic': '/joy',
                'pose_topic': 'end_pose_euler',
                'pos_cmd_topic': '/pos_cmd',
                'gripper_service': 'gripper_control',
                'go_zero_service': 'go_zero_srv',

                'command_rate_hz': LaunchConfiguration('command_rate_hz'),
                'linear_speed_mps': LaunchConfiguration('linear_speed_mps'),
                'angular_speed_rps': LaunchConfiguration('angular_speed_rps'),
                'deadzone': LaunchConfiguration('deadzone'),
                'axis_filter_alpha': LaunchConfiguration('axis_filter_alpha'),
                'trigger_deadzone': LaunchConfiguration('trigger_deadzone'),
                'motion_epsilon': LaunchConfiguration('motion_epsilon'),
                'trigger_mode': LaunchConfiguration('trigger_mode'),
                'pos_cmd_mode1': 0,
                'pos_cmd_mode2': 0,

                # Default Xbox-like mapping.
                'axis_x': 1,
                'axis_y': 0,
                'axis_roll': 3,
                'axis_pitch': 4,
                'axis_yaw': 6,
                'axis_z_up': 5,
                'axis_z_down': 2,
                'deadman_button': 5,
                'local_deadman_button': 4,
                'gripper_open_button': 3,
                'gripper_close_button': 0,
                'home_button': 1,
            }],
        ),
    ])