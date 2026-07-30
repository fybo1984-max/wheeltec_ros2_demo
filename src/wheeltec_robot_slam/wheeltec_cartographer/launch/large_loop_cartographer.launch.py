import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    bringup_launch_dir = os.path.join(bringup_dir, 'launch')
    cartographer_dir = get_package_share_directory('wheeltec_cartographer')
    slam_toolbox_dir = get_package_share_directory('wheeltec_slam_toolbox')
    rviz_dir = get_package_share_directory('wheeltec_rviz2')

    use_rviz = LaunchConfiguration('use_rviz')
    load_state_filename = LaunchConfiguration('load_state_filename')
    yaw_scale_positive = LaunchConfiguration('yaw_scale_positive')
    yaw_scale_negative = LaunchConfiguration('yaw_scale_negative')

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, 'base_serial.launch.py')
        )
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bringup_launch_dir,
                'robot_mode_description.launch.py',
            )
        )
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, 'wheeltec_lidar.launch.py')
        )
    )

    odom_mapping_corrector = Node(
        package='wheeltec_slam_toolbox',
        executable='odom_mapping_corrector.py',
        name='odom_mapping_corrector',
        output='screen',
        parameters=[{
            'yaw_scale_positive': ParameterValue(
                yaw_scale_positive,
                value_type=float,
            ),
            'yaw_scale_negative': ParameterValue(
                yaw_scale_negative,
                value_type=float,
            ),
            'linear_velocity_variance': 0.01,
            'yaw_velocity_variance': 0.005,
        }],
    )

    mapping_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='mapping_ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(
                slam_toolbox_dir,
                'config',
                'ekf_large_loop.yaml',
            )
        ],
        remappings=[
            ('/odometry/filtered', 'odom_combined'),
        ],
    )

    cartographer = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory',
            os.path.join(cartographer_dir, 'config'),
            '-configuration_basename',
            'cartographer_large_loop.lua',
            '-load_state_filename',
            load_state_filename,
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom_combined'),
        ],
    )

    occupancy_grid = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=[
            '-resolution',
            '0.05',
            '-publish_period_sec',
            '1.0',
        ],
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_dir, 'launch', 'wheeltec_rviz.launch.py')
        ),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Open RViz while mapping',
        ),
        DeclareLaunchArgument(
            'load_state_filename',
            default_value='',
            description='Optional frozen Cartographer state to continue mapping from',
        ),
        DeclareLaunchArgument(
            'yaw_scale_positive',
            default_value='0.9735',
            description='Calibrated chassis yaw-rate scale for left turns',
        ),
        DeclareLaunchArgument(
            'yaw_scale_negative',
            default_value='0.9130',
            description='Calibrated chassis yaw-rate scale for right turns',
        ),
        base,
        robot_description,
        joint_state_publisher,
        lidar,
        odom_mapping_corrector,
        TimerAction(period=2.0, actions=[mapping_ekf]),
        TimerAction(period=4.0, actions=[cartographer]),
        TimerAction(period=6.0, actions=[occupancy_grid]),
        rviz,
    ])
