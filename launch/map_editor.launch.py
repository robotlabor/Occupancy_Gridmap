#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_map_topic = LaunchConfiguration('input_map_topic')
    output_map_topic = LaunchConfiguration('output_map_topic')
    waypoint_topic = LaunchConfiguration('waypoint_topic')
    initial_pose_topic = LaunchConfiguration('initial_pose_topic')
    base_frame = LaunchConfiguration('base_frame')
    publish_static_tf_default = LaunchConfiguration(
        'publish_static_tf_default'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_map_topic',
            default_value='/map',
            description='OccupancyGrid topic captured by the editor.',
        ),
        DeclareLaunchArgument(
            'output_map_topic',
            default_value='/edited_map',
            description='Topic used to publish the edited OccupancyGrid.',
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
            executable='map_editor',
            name='map_editor',
            output='screen',
            parameters=[{
                'input_map_topic': input_map_topic,
                'output_map_topic': output_map_topic,
                'waypoint_topic': waypoint_topic,
                'initial_pose_topic': initial_pose_topic,
                'base_frame': base_frame,
                'publish_static_tf_default': publish_static_tf_default,
            }],
        ),
    ])
