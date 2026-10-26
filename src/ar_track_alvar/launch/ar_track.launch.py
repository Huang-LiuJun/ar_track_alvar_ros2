"""ROS 2 port of launch/ar_track.launch (single marker tracking, no Kinect)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cam_image_topic = LaunchConfiguration('cam_image_topic')
    cam_info_topic = LaunchConfiguration('cam_info_topic')
    output_frame = LaunchConfiguration('output_frame')

    return LaunchDescription([
        DeclareLaunchArgument('marker_size', default_value='9.0'),
        DeclareLaunchArgument('max_new_marker_error', default_value='0.08'),
        DeclareLaunchArgument('max_track_error', default_value='0.2'),
        DeclareLaunchArgument('cam_image_topic',
                              default_value='/usb_cam/image_raw'),
        DeclareLaunchArgument('cam_info_topic',
                              default_value='/usb_cam/camera_info'),
        # ROS 2 frame ids must not start with '/'.
        DeclareLaunchArgument('output_frame', default_value='usb_cam'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map',
            arguments=['--x', '0', '--y', '0', '--z', '0.0',
                       '--yaw', '0', '--pitch', '0', '--roll', '0',
                       '--frame-id', 'world', '--child-frame-id', 'map'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_cam',
            arguments=['--x', '0', '--y', '0', '--z', '0.3',
                       '--yaw', '0', '--pitch', '1.57', '--roll', '-1.57',
                       '--frame-id', 'base_link', '--child-frame-id', 'usb_cam'],
        ),

        Node(
            package='ar_track_alvar',
            executable='individualMarkersNoKinect',
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
                'output_frame': ParameterValue(output_frame,
                                               value_type=str),
            }],
            remappings=[
                ('camera_image', cam_image_topic),
                ('camera_info', cam_info_topic),
            ],
        ),
    ])
