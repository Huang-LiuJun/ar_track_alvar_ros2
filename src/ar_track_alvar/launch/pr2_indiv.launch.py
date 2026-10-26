"""ROS 2 port of launch/pr2_indiv.launch (Kinect point cloud tracking)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cam_image_topic = LaunchConfiguration('cam_image_topic')
    cam_info_topic = LaunchConfiguration('cam_info_topic')

    return LaunchDescription([
        DeclareLaunchArgument('marker_size', default_value='4.4'),
        DeclareLaunchArgument('max_new_marker_error', default_value='0.08'),
        DeclareLaunchArgument('max_track_error', default_value='0.2'),
        DeclareLaunchArgument(
            'cam_image_topic',
            default_value='/kinect_head/depth_registered/points'),
        DeclareLaunchArgument('cam_info_topic',
                              default_value='/kinect_head/rgb/camera_info'),
        # ROS 2 frame ids must not start with '/'.
        DeclareLaunchArgument('output_frame', default_value='torso_lift_link'),

        Node(
            package='ar_track_alvar',
            executable='individualMarkers',
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
            }],
            remappings=[
                ('camera_image', cam_image_topic),
                ('camera_info', cam_info_topic),
            ],
        ),
    ])
