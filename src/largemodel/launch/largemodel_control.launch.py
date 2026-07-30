import os
import yaml
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
def generate_launch_description():
    # 获取包的共享目录
    use_nav = LaunchConfiguration('use_nav')
    use_nav_arg = DeclareLaunchArgument(
        'use_nav',
        default_value='false',
        description='是否同时启动Nav2定位导航，用于记录点位和语音导航'
    )
    use_camera = LaunchConfiguration('use_camera')
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='false',
        description='是否启动并监测相机；普通语音点位导航不需要相机'
    )
    vad_threshold = LaunchConfiguration('vad_threshold')
    vad_threshold_arg = DeclareLaunchArgument(
        'vad_threshold',
        default_value='0.7',
        description='AIUI VAD灵敏度阈值，越低越容易检测到较小声音'
    )
    fixed_command_mode = LaunchConfiguration('fixed_command_mode')
    fixed_command_mode_arg = DeclareLaunchArgument(
        'fixed_command_mode',
        default_value='true',
        description='固定语音口令使用本地白名单直接执行，避免大模型误解释'
    )
    nav_controller_frequency = LaunchConfiguration('nav_controller_frequency')
    nav_model_dt = LaunchConfiguration('nav_model_dt')
    nav_time_steps = LaunchConfiguration('nav_time_steps')
    nav_batch_size = LaunchConfiguration('nav_batch_size')
    nav_progress_radius = LaunchConfiguration('nav_progress_radius')
    nav_progress_timeout = LaunchConfiguration('nav_progress_timeout')
    semantic_costmap_enabled = LaunchConfiguration('semantic_costmap_enabled')
    semantic_mask_source = LaunchConfiguration('semantic_mask_source')
    semantic_task_urgency = LaunchConfiguration('semantic_task_urgency')
    semantic_avoidance_level = LaunchConfiguration('semantic_avoidance_level')
    nav_controller_frequency_arg = DeclareLaunchArgument(
        'nav_controller_frequency',
        default_value='10.0',
        description='大模型联合导航的控制频率，降低该值可减少计算超时'
    )
    nav_model_dt_arg = DeclareLaunchArgument(
        'nav_model_dt',
        default_value='0.1',
        description='大模型联合导航的MPPI时间步长'
    )
    nav_time_steps_arg = DeclareLaunchArgument(
        'nav_time_steps',
        default_value='28',
        description='大模型联合导航的MPPI预测步数'
    )
    nav_batch_size_arg = DeclareLaunchArgument(
        'nav_batch_size',
        default_value='1000',
        description='大模型联合导航的MPPI采样数'
    )
    nav_progress_radius_arg = DeclareLaunchArgument(
        'nav_progress_radius',
        default_value='0.2',
        description='大模型联合导航判定有进展所需的最小移动距离（米）'
    )
    nav_progress_timeout_arg = DeclareLaunchArgument(
        'nav_progress_timeout',
        default_value='20.0',
        description='大模型联合导航允许未达到最小移动距离的时间（秒）'
    )
    semantic_costmap_enabled_arg = DeclareLaunchArgument(
        'semantic_costmap_enabled',
        default_value='false',
        description='是否启用全局语义软成本层；默认关闭以保持稳定演示行为'
    )
    semantic_mask_source_arg = DeclareLaunchArgument(
        'semantic_mask_source',
        default_value='file',
        choices=['file', 'topic'],
        description='语义mask来源：静态文件file或动态OccupancyGrid话题topic'
    )
    semantic_task_urgency_arg = DeclareLaunchArgument(
        'semantic_task_urgency',
        default_value='0',
        description='任务紧急度，范围0到10；越高越允许穿越语义区域'
    )
    semantic_avoidance_level_arg = DeclareLaunchArgument(
        'semantic_avoidance_level',
        default_value='70.0',
        description='语义区域避让级别，范围0到100；越高绕行倾向越强'
    )
    params_file=os.path.join(get_package_share_directory('largemodel'), "config", "param.yaml")
    wheeltec_robot_dir = get_package_share_directory('turn_on_wheeltec_robot')
    wheeltec_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheeltec_robot_dir,'launch','wheeltec_sensors.launch.py')
            ),
        launch_arguments={
            'robot_nav': use_nav,
            'use_camera': use_camera,
        }.items(),
        )
    odom_mapping_corrector = Node(
        package='wheeltec_slam_toolbox',
        executable='odom_mapping_corrector.py',
        name='odom_mapping_corrector',
        output='screen',
        parameters=[{
            'yaw_scale_positive': 0.9735,
            'yaw_scale_negative': 0.9130,
            'linear_velocity_variance': 0.01,
            'yaw_velocity_variance': 0.005,
        }],
        condition=IfCondition(use_nav),
    )
    wheeltec_nav_dir = get_package_share_directory('wheeltec_nav2')
    wheeltec_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        # 保留用户当前RViz布局，并固定显示Nav2全局与局部规划路径。
        arguments=[
            '-d', os.path.join(wheeltec_nav_dir, 'rviz', 'largemodel_nav.rviz'),
            '-f', 'map',
        ],
        output='screen',
        condition=IfCondition(use_nav),
    )
    wheeltec_config = os.path.join(
        get_package_share_directory('turn_on_wheeltec_robot'),
        'config',
        'wheeltec_param.yaml'
    )
    with open(wheeltec_config, 'r', encoding='utf-8') as config_file:
        car_mode = yaml.safe_load(config_file)['car_mode']
    nav_params_file = os.path.join(
        wheeltec_nav_dir,
        'param',
        'wheeltec_params',
        f'param_{car_mode}.yaml'
    )
    ai_nav_params = RewrittenYaml(
        source_file=nav_params_file,
        root_key='',
        param_rewrites={
            'controller_frequency': nav_controller_frequency,
            'model_dt': nav_model_dt,
            'time_steps': nav_time_steps,
            'batch_size': nav_batch_size,
            'required_movement_radius': nav_progress_radius,
            'movement_time_allowance': nav_progress_timeout,
            'global_costmap.global_costmap.ros__parameters.mask_layer.enabled':
                semantic_costmap_enabled,
            'global_costmap.global_costmap.ros__parameters.mask_layer.mask_source':
                semantic_mask_source,
            'global_costmap.global_costmap.ros__parameters.mask_layer.task_urgency':
                semantic_task_urgency,
            'global_costmap.global_costmap.ros__parameters.mask_layer.avoidance_level':
                semantic_avoidance_level,
        },
        convert_types=True,
    )
    wheeltec_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheeltec_nav_dir, 'launch', 'wheeltec_nav2_model.launch.py')
            ),
        launch_arguments={
            'params': ai_nav_params,
            # Standalone processes avoid stale same-name component containers
            # intercepting Nav2 component load requests after an abnormal stop.
            # The bundled Humble Nav2 launch evaluates these strings as a
            # Python expression, so use Python boolean spelling.
            'use_composition': 'False',
            'use_respawn': 'True',
        }.items(),
        condition=IfCondition(use_nav),
        )
    
    wheeltec_mic = Node(
        package="wheeltec_mic_aiui",
        executable="wheeltec_mic",
        output='screen',
        parameters=[{"usart_port_name": "/dev/wheeltec_mic",
                    "serial_baud_rate": 115200}]
    )

    wheeltec_mic_aiui = Node(
        package="wheeltec_mic_aiui",
        executable="wheeltec_mic_aiui",
        output='screen',
        parameters=[{
            "record_device_name": "hw:CARD=XFMDPV0018,DEV=0",
            "vad_threshold": vad_threshold
        }],
    )

    # 定义节点
    model_server = Node(
        package='largemodel',
        executable='model_service',
        name='model_service',
        parameters=[params_file, {
            'fixed_command_mode': fixed_command_mode,
            'integrated_nav_mode': use_nav,
        }],
        output='screen'
    )
    action_server = Node(
        package='largemodel',
        executable='action_service',
        name='action_service',
        parameters=[params_file, {'monitor_camera': use_camera}],
        output='screen'
    )
    lasertracker = Node(
        package="simple_follower_ros2", 
        executable="lasertracker", 
        name='lasertracker'
    )
    return LaunchDescription([
        use_nav_arg,
        use_camera_arg,
        vad_threshold_arg,
        fixed_command_mode_arg,
        nav_controller_frequency_arg,
        nav_model_dt_arg,
        nav_time_steps_arg,
        nav_batch_size_arg,
        nav_progress_radius_arg,
        nav_progress_timeout_arg,
        semantic_costmap_enabled_arg,
        semantic_mask_source_arg,
        semantic_task_urgency_arg,
        semantic_avoidance_level_arg,
        model_server,          #启动模型服务节点
        action_server,         #启动动作服务节点
        wheeltec_mic,
        wheeltec_mic_aiui,
        lasertracker,
        wheeltec_nav,
        TimerAction(period=12.0, actions=[wheeltec_rviz]),
        odom_mapping_corrector,
        wheeltec_sensors
    ])
