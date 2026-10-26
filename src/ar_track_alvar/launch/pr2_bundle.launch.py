"""ROS 2 port of launch/pr2_bundle.launch (Kinect bundle tracking)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bundle_dir = os.path.join(get_package_share_directory('ar_track_alvar'),
                              'bundles')
    default_bundles = ' '.join([
        os.path.join(bundle_dir, 'truthTableLeg.xml'),
        os.path.join(bundle_dir, 'table_8_9_10.xml'),
    ])

    cam_image_topic = LaunchConfiguration('cam_image_topic')
    cam_info_topic = LaunchConfiguration('cam_info_topic')

    return LaunchDescription([
        DeclareLaunchArgument('marker_size', default_value='4.4'),
        DeclareLaunchArgument('max_new_marker_error', default_value='0.08'),
        DeclareLaunchArgument('max_track_error', default_value='0.0'),
        DeclareLaunchArgument(
            'cam_image_topic',
            default_value='/kinect_head/depth_registered/points'),
        DeclareLaunchArgument('cam_info_topic',
                              default_value='/kinect_head/rgb/camera_info'),
        # ROS 2 frame ids must not start with '/'.
        DeclareLaunchArgument('output_frame', default_value='torso_lift_link'),
        DeclareLaunchArgument('med_filt_size', default_value='10'),
        DeclareLaunchArgument('bundle_files', default_value=default_bundles),

        Node(
            package='ar_track_alvar',
            executable='findMarkerBundles',
            name='ar_track_alvar',
            output='screen',
            parameters=[{
                'marker_size': ParameterValue(
                    LaunchConfiguration('marker_size'), value_type=float),
                'max_new_marker_error': ParameterValue(
                    LaunchConfiguration('max_new_marker_error'),
                    value_type=float),
                'max_track_error': ParameterValue(
                    LaunchConfiguration('max_track_error'), value_type=float),
                'output_frame': ParameterValue(
                    LaunchConfiguration('output_frame'), value_type=str),
                'med_filt_size': ParameterValue(
                    LaunchConfiguration('med_filt_size'), value_type=int),
                'bundle_files': ParameterValue(
                    LaunchConfiguration('bundle_files'), value_type=str),
            }],
            remappings=[
                ('camera_image', cam_image_topic),
                ('camera_info', cam_info_topic),
            ],
        ),
    ])
