from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('pointcloud_bbox_filter'),
        'config',
        'box_filter_params.yaml'
    )

    return LaunchDescription([

        Node(
            package='pointcloud_bbox_filter',
            executable='bbox_filter_node',
            name='bbox_filter_node',
            parameters=[config_path],
            output='screen'
            ),
    ])