from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Get package paths
    panthera_moveit_path = FindPackageShare('panthera_moveit')

    # Construct relative path to config file
    default_config_file = PathJoinSubstitution([
        panthera_moveit_path,
        'config',
        '6dof_arm_params.yaml'
    ])

    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to robot configuration YAML file'
    )

    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='position_velocity',
        description='Control mode: position_velocity, pd_control, or full_control'
    )

    # ============================================
    # 1. Hardware Launch (robot_state_publisher, controller_manager, controllers)
    # ============================================
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                panthera_moveit_path,
                'launch',
                'hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'config_file': LaunchConfiguration('config_file'),
            'control_mode': LaunchConfiguration('control_mode'),
        }.items()
    )

    # ============================================
    # 2. MoveIt Config
    # ============================================
    moveit_config = MoveItConfigsBuilder("panthera_ht", package_name="panthera_moveit").to_moveit_configs()

    # For real hardware, use system time, not simulation time
    move_group_configuration = {
        "publish_robot_description_semantic": False,
        "allow_trajectory_execution": True,
        "publish_robot_description": False,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
        "use_sim_time": False,  # CRITICAL: Use system time for real hardware
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    # ============================================
    # 3. Move Group Node
    # ============================================
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    return LaunchDescription([
        config_file_arg,
        control_mode_arg,
        hardware_launch,
        move_group_node,
    ])
