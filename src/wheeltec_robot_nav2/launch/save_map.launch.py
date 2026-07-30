#ros2 run nav2_map_server map_saver_cli -f ~/map
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    package_share_dir = Path(get_package_share_directory('wheeltec_nav2'))
    share_text = str(package_share_dir)
    if '/install/' in share_text:
        detected_workspace = Path(share_text.split('/install/', 1)[0])
    else:
        detected_workspace = Path.cwd()

    workspace_dir = Path(
        os.environ.get('WHEELTEC_WORKSPACE', str(detected_workspace))
    )
    installed_map = package_share_dir / 'map' / 'WHEELTEC'
    source_map = (
        workspace_dir / 'src' / 'wheeltec_robot_nav2' / 'map' / 'WHEELTEC'
    )

    map_saver = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        output='screen',
        arguments=['-f', str(installed_map)],
        
        parameters=[{'save_map_timeout': 20000.0},
                    {'free_thresh_default': 0.196}]

        )
    map_backup = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_backup',
        output='screen',
        arguments=['-f', str(source_map)],
        
        parameters=[{'save_map_timeout': 20000.0},
                    {'free_thresh_default': 0.196}]

        )
    ld = LaunchDescription()

    ld.add_action(map_saver)
    ld.add_action(map_backup)
    return ld
 
