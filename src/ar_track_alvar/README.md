# ar_track_alvar (ROS 2 Humble port)

ROS 2 port of the ROS 1 `ar_track_alvar` package (noetic-devel branch,
commit `a857aab`). It contains the complete ALVAR 2.0.0 C++ library, all five
tracker/trainer nodes, the `createMarker` example, the message package, the
original bundle files and marker images.

This is an **unofficial** port, not affiliated with or endorsed by the original
authors, VTT Technical Research Centre of Finland, Willow Garage or Open
Robotics. The ALVAR core is LGPL-2.1; the ROS node and filter code is
BSD-3-Clause; the original copyright headers are retained. For full provenance,
licensing details, verification status and troubleshooting, see the
[repository root README](../../README.md).

## Packages

| Package | Description |
| --- | --- |
| `ar_track_alvar_msgs` | `AlvarMarker` / `AlvarMarkers` messages |
| `ar_track_alvar` | ALVAR library, tracker nodes, launch files, assets |

## Build

```bash
mkdir -p ~/ar_ws/src
git clone https://github.com/Huang-LiuJun/ar_track_alvar_ros2.git \
  ~/ar_ws/src/ar_track_alvar_ros2
cd ~/ar_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select ar_track_alvar_msgs ar_track_alvar
source install/setup.bash
```

Build dependencies: `rclcpp`, `cv_bridge`, `image_transport`, `tf2`,
`tf2_ros`, `tf2_geometry_msgs`, `pcl_conversions`, `visualization_msgs`,
OpenCV, Eigen3, PCL, `tinyxml` (via pkg-config).

## Nodes

All nodes subscribe to `camera_info` (only the first message is used) and read
ROS 2 parameters instead of the deprecated ROS 1 command-line arguments.

| Executable | Input on `camera_image` | Description |
| --- | --- | --- |
| `individualMarkersNoKinect` | `sensor_msgs/Image` | Detect and track single markers |
| `individualMarkers` | `sensor_msgs/PointCloud2` | Single markers refined with PCL plane fitting |
| `findMarkerBundlesNoKinect` | `sensor_msgs/Image` | Multi-marker bundle pose |
| `findMarkerBundles` | `sensor_msgs/PointCloud2` | Bundle pose refined with PCL |
| `trainMarkerBundle` | `sensor_msgs/Image` | Interactive bundle training (OpenCV window) |

Published topics:

- `ar_pose_marker` (`ar_track_alvar_msgs/AlvarMarkers`) — marker poses in
  `output_frame`.
- `visualization_marker` (`visualization_msgs/Marker`) — RViz cubes.
- `ARmarker_points` (`visualization_msgs/Marker`) — PCL debug spheres/arrows,
  Kinect nodes only.
- TF `ar_marker_<id>` — camera/`output_frame` to marker transforms. The
  NoKinect bundle node publishes only the bundle master; the Kinect bundle node
  publishes every detected marker.

`~/enable_detection` (`std_msgs/Bool`) toggles detection at runtime in
`individualMarkersNoKinect` and `findMarkerBundlesNoKinect` only. The other
nodes use the `enabled` parameter.

### Parameters

Common to every node: `marker_size`, `max_new_marker_error`, `max_track_error`,
`output_frame` (required unless `output_frame_from_msg` is true).

Tracker nodes: `max_frequency` (default 8.0), `marker_resolution` (5),
`marker_margin` (2), `enabled` (true).

`individualMarkers`: `output_frame_from_msg` (false) — use the point cloud
header frame instead of `output_frame`.

`findMarkerBundles`: `med_filt_size` (10), `bundle_files` (space separated
list of XML files).

`trainMarkerBundle`: `nof_markers` (1). Keyboard shortcuts: `l` load,
`s` save (`mmarker.xml`), `r` reset, `p` add measurement, `a` auto add
measurements, `o` optimize, `q` quit.

The ROS 1 `dynamic_reconfigure` parameters are ordinary ROS 2 parameters and
can be changed while running:

```bash
ros2 param set /ar_track_alvar marker_size 4.4
```

`marker_size`, `max_new_marker_error`, `max_track_error`, `max_frequency` and
`enabled` are re-applied by every tracker. `marker_resolution` and
`marker_margin` are re-applied only by `individualMarkersNoKinect`; the other
nodes read them at startup.

## Generating markers

`createMarker` writes PNG files into the current directory:

```bash
ros2 run ar_track_alvar createMarker -ucm -s 9 0        # one 9 cm marker, ID 0
ros2 run ar_track_alvar createMarker -ucm -s 5 -xy 0 0 7 -xy 6 0 8
```

Several bare IDs in one call are all drawn at the same position and overlap;
use `-xy` before each ID, use `-ar` (interactive array mode), or run the command
once per marker. `-r` does not force the grid size of `MarkerData` markers:
IDs 0-255 use 5x5, IDs 256-32767 use 7x7, and the node's `marker_resolution`
must match.

The XML that `createMarker` writes for multi-marker sheets is **not** a valid
bundle file: an upstream ALVAR defect puts the first corner of each marker at
the marker origin. Use the example XML files in `bundles/`, or inspect and
repair the output of `trainMarkerBundle`, whose master marker is initialised
through the same function. See the
[root README section 7.4](../../README.md#74-output-files).

## Launch

```bash
ros2 launch ar_track_alvar ar_track.launch.py               # single marker, USB camera
ros2 launch ar_track_alvar pr2_indiv.launch.py              # single marker, Kinect cloud
ros2 launch ar_track_alvar pr2_indiv_no_kinect.launch.py    # single marker, stereo image
ros2 launch ar_track_alvar pr2_bundle.launch.py             # bundle, Kinect cloud
ros2 launch ar_track_alvar pr2_bundle_no_kinect.launch.py   # bundle, stereo image
ros2 launch ar_track_alvar pr2_train.launch.py              # train a bundle
```

Every launch file exposes the ROS 1 arguments (`marker_size`,
`max_new_marker_error`, `max_track_error`, `cam_image_topic`,
`cam_info_topic`, `output_frame`, plus `bundle_files`, `med_filt_size` or
`nof_markers`). `config/params.yaml` holds the common defaults and can be
passed with `--params-file`.

Frame ids must not start with `/` in ROS 2, so the ported launch defaults use
`usb_cam` and `torso_lift_link` instead of `/usb_cam` and `/torso_lift_link`.

## Assets

- `bundles/` — example multi-marker bundle XML files used by the PR2 launch
  files.
- `markers/` — printable marker sheets (`Markers_0_to_8.png`,
  `Markers_9_to_17.png`, `table_8_9_10.png`).
- `createMarker` — interactive generator for marker PNG/XML files.

## Differences from the ROS 1 package

- `dynamic_reconfigure` / `Params.cfg` replaced by node parameters
  (`config/params.yaml` documents the defaults).
- Deprecated ROS 1 command-line argument parsing removed; all values are ROS 2
  parameters. Topics are remapped as usual.
- `ros::Rate` processing loops replaced by a per-message timestamp gate at
  `1 / max_frequency`.
- ROS 1 `tf` replaced by `tf2` / `tf2_ros`; `tf::Vector3` in the bundle
  library is `cv::Point3d`, `tf::Matrix3x3` in `kinect_filtering` is
  `tf2::Matrix3x3`.
- `sensor_msgs/CameraInfo` is handed to `alvar::Camera` from each node; the
  library no longer owns a `ros::NodeHandle` or a camera-info subscriber.
- The ALVAR `Rotation` copy constructor now keeps the raw `quaternion[4]`
  array in sync with `quaternion_mat`. Upstream left the array uninitialized
  when copying a `Pose`, which made the Kinect nodes publish NaN quaternions.
- The ROS 1 `test/` directory is not ported: its C++ files are manual
  diagnostic programs with no assertions, and the `.test` launch files need a
  ROS 1 bag and rostest (upstream never compiled them either). End-to-end
  detection is verified separately on ROS 2 Humble with image and point-cloud
  inputs.
