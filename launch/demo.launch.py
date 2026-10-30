import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('negative_obstacle_detector')
    config_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'default.rviz')

    return LaunchDescription([
        # 1. Terepgenerátor és kék autó modell
        Node(
            package='negative_obstacle_detector',
            executable='mock_terrain_publisher',
            name='mock_terrain_node',
            output='screen',
            parameters=[config_file]
        ),
        # 2. Negatív akadály detektor és HUD szöveges kijelző
        Node(
            package='negative_obstacle_detector',
            executable='detector_node',
            name='detector_node',
            output='screen',
            parameters=[config_file]
        ),
        # 3. RViz2 automatikus indítása a kész felülettel
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
