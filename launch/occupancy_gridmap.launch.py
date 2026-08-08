#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tif_path = LaunchConfiguration('tif_path')
    grid_resolution = LaunchConfiguration('grid_resolution')
    waypoint_topic = LaunchConfiguration('waypoint_topic')
    initial_pose_topic = LaunchConfiguration('initial_pose_topic')
    base_frame = LaunchConfiguration('base_frame')
    publish_static_tf_default = LaunchConfiguration(
        'publish_static_tf_default'
    )

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
            'waypoint_topic',
            default_value='/waypoints',
            description='PoseArray topic used to publish graphical cones.',
        ),
        DeclareLaunchArgument(
            'initial_pose_topic',
            default_value='/initialpose',
            description='Topic used to publish the graphical robot start pose.',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Child frame of the optional graphical test TF.',
        ),
        DeclareLaunchArgument(
            'publish_static_tf_default',
            default_value='true',
            choices=['true', 'false'],
            description='Default state of the editor static-TF checkbox.',
        ),
        Node(
            package='occupancy_gridmap',
            executable='ortho_gridmap_node',
            name='ortho_gridmap_node',
            output='screen',
            parameters=[{
                'tif_path': tif_path,
                'grid_resolution': grid_resolution,
                'map_topic': '/map',
            }],
        ),
        Node(
            package='occupancy_gridmap',
            executable='map_editor',
            name='map_editor',
            output='screen',
            parameters=[{
                'input_map_topic': '/map',
                'output_map_topic': '/edited_map',
                'waypoint_topic': waypoint_topic,
                'initial_pose_topic': initial_pose_topic,
                'base_frame': base_frame,
                'publish_static_tf_default': publish_static_tf_default,
            }],
        ),
    ])
