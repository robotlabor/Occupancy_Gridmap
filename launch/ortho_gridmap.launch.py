#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tif_path = LaunchConfiguration('tif_path')
    grid_resolution = LaunchConfiguration('grid_resolution')
    map_topic = LaunchConfiguration('map_topic')
    map_frame = LaunchConfiguration('map_frame')
    publish_period = LaunchConfiguration('publish_period')

    return LaunchDescription([
        DeclareLaunchArgument(
            'tif_path',
            description='Absolute path to the input GeoTIFF file.',
        ),
        DeclareLaunchArgument(
            'grid_resolution',
            default_value='0.3',
            description='OccupancyGrid resolution in metres per cell.',
        ),
        DeclareLaunchArgument(
            'map_topic',
            default_value='/map',
            description='Output OccupancyGrid topic.',
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Frame ID of the generated map.',
        ),
        DeclareLaunchArgument(
            'publish_period',
            default_value='1.0',
            description='Map publication period in seconds.',
        ),
        Node(
            package='occupancy_gridmap',
            executable='ortho_gridmap_node',
            name='ortho_gridmap_node',
            output='screen',
            parameters=[{
                'tif_path': tif_path,
                'grid_resolution': grid_resolution,
                'map_topic': map_topic,
                'map_frame': map_frame,
                'publish_period': publish_period,
            }],
        ),
    ])
