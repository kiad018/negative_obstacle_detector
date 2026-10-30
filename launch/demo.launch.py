from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='negative_obstacle_detector',
            executable='mock_terrain_publisher',
            name='mock_terrain_node',
            output='screen'
        ),
        Node(
            package='negative_obstacle_detector',
            executable='detector_node',
            name='detector_node',
            output='screen',
            parameters=[{
                'sensor_height': 1.5,
                'drop_threshold': 0.35,
                'radial_gap_threshold': 1.0,
                'min_range': 1.0,
                'max_range': 15.0,
                'num_azimuth_bins': 72
            }]
        )
    ])
