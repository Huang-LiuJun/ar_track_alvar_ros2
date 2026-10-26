# ar_track_alvar for ROS 2

ROS 2 port of the ROS 1 [`ar_track_alvar`](https://github.com/ros-perception/ar_track_alvar)
package. It wraps the ALVAR 2.0.0 augmented-reality tag library and provides
five nodes for detecting single markers or multi-marker bundles from images or
RGB-D point clouds, plus `createMarker` for generating printable markers.

* Target platform: **ROS 2 Humble on Ubuntu 22.04**.
* The detection algorithm, marker encoding, bundle file format and message
  layout are unchanged. This is a build-system and API port, not a rewrite.
* Verified in simulation with synthetic images and organized point clouds.
  It has **not** been tested with a physical camera by the porter. See
  [Verification status](#16-verification-status).

---

## Table of contents

1. [Provenance, attribution and license](#1-provenance-attribution-and-license)
2. [What the port changed](#2-what-the-port-changed)
3. [Repository layout](#3-repository-layout)
4. [Requirements](#4-requirements)
5. [Build](#5-build)
6. [Quick start](#6-quick-start)
7. [Generating markers with `createMarker`](#7-generating-markers-with-createmarker)
8. [Marker IDs, resolution and printing](#8-marker-ids-resolution-and-printing)
9. [Nodes](#9-nodes)
10. [Parameters](#10-parameters)
11. [Topics and TF](#11-topics-and-tf)
12. [Launch files](#12-launch-files)
13. [Bundle files and bundle training](#13-bundle-files-and-bundle-training)
14. [Units and coordinate conventions](#14-units-and-coordinate-conventions)
15. [Troubleshooting](#15-troubleshooting)
16. [Verification status](#16-verification-status)
17. [Known limitations](#17-known-limitations)
18. [Credits](#18-credits)

---

## 1. Provenance, attribution and license

This repository is an **unofficial ROS 2 port** of the ROS 1 package
`ar_track_alvar`.

| Item | Details |
| --- | --- |
| Upstream repository | <https://github.com/ros-perception/ar_track_alvar> |
| Upstream branch | `noetic-devel` |
| Upstream commit used | `a857aab4d6ac144478fb92f6624e0088473f4b33` (2021-07-08) |
| Original ROS wrapper | Scott Niekum and contributors; see `CHANGELOG.rst` and the headers of the original files |
| ALVAR library | ALVAR 2.0.0, VTT Technical Research Centre of Finland, 2007-2012 |
| Kinect plane-fitting code | Willow Garage, Inc., 2008 |
| ROS 2 port | The contributors of this repository |

**This project is not affiliated with, sponsored by, or endorsed by** the
original `ar_track_alvar` authors, VTT Technical Research Centre of Finland,
Willow Garage, Open Robotics, or the ROS project. The names of those projects
and organisations are used only to describe the origin of the code.

The port keeps the original copyright and license headers wherever upstream
provides them. Some upstream ALVAR sample sources (for example
`SampleMarkerCreator.cpp`, which builds `createMarker`) have no header in the
original package; they are left unchanged and are distributed as part of ALVAR
under the same LGPL-2.1 terms. Files that were modified for ROS 2 still carry
their original notices, and the port's changes to those files are distributed
under the same license as the file they modify. The marker images in
`markers/`, the bundle XML files in `bundles/` and the launch scenarios come
from the upstream package.

### Licensing

The repository contains code under **two different licenses**, because the
upstream package itself is mixed:

| Path | License | Copyright |
| --- | --- | --- |
| `src/ar_track_alvar/src/` and `src/ar_track_alvar/include/ar_track_alvar/` (ALVAR core, except the BSD files listed below) | LGPL-2.1 | VTT Technical Research Centre of Finland, 2007-2012 |
| `src/ar_track_alvar/nodes/` | BSD-3-Clause | Scott Niekum, 2012 |
| `src/ar_track_alvar/include/ar_track_alvar/filter/`, `src/kinect_filtering.cpp`, `src/medianFilter.cpp` | BSD-3-Clause | Willow Garage, Inc., 2008 |
| `src/ar_track_alvar_msgs/` | BSD (as declared in its `package.xml`) | upstream message package |
| `bundles/`, `markers/`, launch files, `config/` | distributed as part of the upstream package | see the upstream package |

`src/ar_track_alvar/package.xml` declares `LGPL-2.1`, matching the upstream
package declaration. Where a file header states a different license, **the file
header governs that file**.

The full text of the GNU Lesser General Public License 2.1 is included in
[`LICENSES/LGPL-2.1.txt`](LICENSES/LGPL-2.1.txt). [`LICENSE`](LICENSE) is a
repository-wide index of the licenses by path; it does not relicense any file.
The BSD-3-Clause text is reproduced in full at the top of each BSD-licensed
source file.

If you redistribute binaries of this package, remember that LGPL-2.1 requires
you to make the corresponding source available and to retain the license
notices. Keep the original copyright headers when you modify or redistribute
the code.

---

## 2. What the port changed

The following changes were necessary to build and run on ROS 2. They do not
change the marker format or the pose-estimation algorithm.

| Area | ROS 1 | ROS 2 port |
| --- | --- | --- |
| Build | `catkin` | `ament_cmake` + `colcon` |
| Messages | `message_generation` | `rosidl_generate_interfaces` |
| Client library | `roscpp` | `rclcpp` |
| Configuration | `dynamic_reconfigure` (`cfg/Params.cfg`) | ordinary ROS 2 parameters (`config/params.yaml` documents the defaults) |
| Node arguments | ROS 1 command-line remapping/app arguments | `--ros-args -p name:=value` and standard ROS 2 remapping |
| Transforms | `tf` | `tf2`, `tf2_ros`, `tf2_geometry_msgs` |
| Image transport | ROS 1 `image_transport` API | ROS 2 `image_transport` API, `raw` transport only |
| Point clouds | `pcl_ros` | `pcl_conversions`, `pcl::make_shared` |
| Camera info | owned by the ALVAR `Camera` helper | each node subscribes to `camera_info` and passes it to the library |
| Launch files | XML `.launch` | Python `.launch.py` (six files, including one convenience file for a USB camera) |
| Tests | `rostest`, ROS 1 bag | not ported; see below |

Additional fixes and changes:

* **`Rotation` copy-constructor fix.** Upstream ALVAR allocated a new
  `quaternion_mat` in the copy constructor but never copied the raw
  `quaternion[4]` array into it. Copying a `Pose` (for example
  `Pose p = marker.pose;` in the Kinect nodes) therefore read uninitialised
  memory and could publish NaN quaternions. The port makes `quaternion[]`
  alias the matrix and copies the values, so copied poses are valid.
* **Runtime parameter handling.** Tracker nodes re-apply `marker_size`,
  `max_new_marker_error`, `max_track_error`, `max_frequency` and `enabled`
  while running. `individualMarkersNoKinect` additionally re-applies
  `marker_resolution` and `marker_margin`. Other nodes need a restart for
  those two parameters.
* **No ROS 1 command-line parsing.** Values such as the marker size are ROS 2
  parameters now. Topic remapping is unchanged in spirit but uses ROS 2
  syntax.
* **Frame IDs without a leading slash.** ROS 2 frame names must not start with
  `/`; the launch files use `usb_cam` and `torso_lift_link` instead of
  `/usb_cam` and `/torso_lift_link`.
* **ROS 1 `test/` directory not ported.** Its C++ files are manual diagnostic
  programs without assertions, and its `.test` files require a ROS 1 bag and
  `rostest`; upstream never compiled them either. End-to-end detection is
  verified separately on ROS 2 Humble (see
  [Verification status](#16-verification-status)).

What did **not** change:

* ALVAR's marker encoding, decoding, tracking and pose estimation.
* The multi-marker bundle XML format and the example bundle files.
* The printable marker images under `markers/`.
* The `ar_track_alvar_msgs` message fields.
* Topic names (`ar_pose_marker`, `visualization_marker`, `camera_image`,
  `camera_info`) and TF child-frame naming (`ar_marker_<id>`).

---

## 3. Repository layout

```text
ar_track_alvar_ros2/
├── README.md                  this file
├── LICENSE                    license index (mixed LGPL-2.1 / BSD-3-Clause)
├── LICENSES/
│   └── LGPL-2.1.txt           full LGPL-2.1 text
└── src/
    ├── ar_track_alvar/
    │   ├── include/ar_track_alvar/   ALVAR headers
    │   ├── src/                      ALVAR library and sample programs
    │   ├── nodes/                    the five ROS 2 nodes
    │   ├── launch/                   six Python launch files
    │   ├── config/params.yaml        common parameter defaults
    │   ├── bundles/                  example bundle XML files
    │   └── markers/                  example printable marker sheets
    └── ar_track_alvar_msgs/
        ├── msg/AlvarMarker.msg
        └── msg/AlvarMarkers.msg
```

Two ROS 2 packages are provided:

| Package | Contents |
| --- | --- |
| `ar_track_alvar` | ALVAR library, five nodes, `createMarker`, launch/config/assets |
| `ar_track_alvar_msgs` | `AlvarMarker` / `AlvarMarkers` messages |

---

## 4. Requirements

* Ubuntu 22.04 with ROS 2 Humble installed.
* A C++17 compiler (the default Humble toolchain works).
* OpenCV, PCL, Eigen3, `tinyxml` and `pkg-config` development files.
* A camera driver that publishes `sensor_msgs/Image` (or an organized
  `sensor_msgs/PointCloud2`) and a matching `sensor_msgs/CameraInfo`.
  For the USB-camera example below, install `ros-humble-usb-cam`.

Install the ROS dependencies with `rosdep` (recommended), or install the
packages directly:

```bash
sudo apt install \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-pcl-conversions \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-visualization-msgs \
  ros-humble-std-msgs \
  ros-humble-geometry-msgs \
  ros-humble-sensor-msgs \
  libopencv-dev libpcl-dev libeigen3-dev libtinyxml-dev pkg-config
```

---

## 5. Build

```bash
mkdir -p ~/ar_ws/src
cd ~/ar_ws/src
git clone https://github.com/Huang-LiuJun/ar_track_alvar_ros2.git ar_track_alvar_ros2
cd ~/ar_ws

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select ar_track_alvar_msgs ar_track_alvar
source install/setup.bash
```

Check the result:

```bash
ros2 pkg executables ar_track_alvar
ros2 interface show ar_track_alvar_msgs/msg/AlvarMarkers
```

`ros2 pkg executables` should list:

```text
ar_track_alvar createMarker
ar_track_alvar findMarkerBundles
ar_track_alvar findMarkerBundlesNoKinect
ar_track_alvar individualMarkers
ar_track_alvar individualMarkersNoKinect
ar_track_alvar trainMarkerBundle
```

The package installs its launch files, `bundles/`, `markers/` and
`config/params.yaml` into the package share directory. The default bundle
paths used by the PR2 launch files are resolved from that share directory, so
they work from any current working directory.

---

## 6. Quick start

The following example uses a USB camera. Any driver that publishes an image
and camera-info topic works; replace the topic names as needed.

### 6.1 Generate and print a marker

```bash
mkdir -p ~/ar_markers
cd ~/ar_markers
ros2 run ar_track_alvar createMarker -ucm -s 9 0
```

This writes `MarkerData_0.png`: a 9 cm marker with numeric ID 0. Print it at
100% ("actual size", not "fit to page"), measure the black square to confirm
that it is 9 cm wide, and leave white paper around the black square. See
[section 8](#8-marker-ids-resolution-and-printing) for details.

### 6.2 Start a camera

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -p video_device:=/dev/video0 \
  -p image_width:=640 \
  -p image_height:=480 \
  -p framerate:=30.0 \
  -p frame_id:=usb_cam
```

This typically publishes `/image_raw` and `/camera_info`. The `usb_cam` ROS 2
driver defaults to `frame_id:=default_cam`; the example sets it to `usb_cam` so
that it matches the `output_frame` used below. Check the actual value with

```bash
ros2 topic echo /image_raw --once | grep frame_id
```

and use that frame name for `output_frame`.

If your camera publishes compressed images only, republish them as raw images
first; the nodes subscribe with the `raw` image transport.

### 6.3 Start the tracker

```bash
ros2 run ar_track_alvar individualMarkersNoKinect --ros-args \
  -p marker_size:=9.0 \
  -p output_frame:=usb_cam \
  -r camera_image:=/image_raw \
  -r camera_info:=/camera_info
```

Notes:

* The default ROS 2 node name is `marker_detect`. The launch files rename it
  to `ar_track_alvar`.
* `output_frame` must be a TF frame connected to the camera frame. Setting it
  to the camera frame itself (the image header frame, `usb_cam` here) avoids a
  TF lookup. If it does not match the image header frame, the node still
  publishes, but it needs a valid TF chain between the two.
* `marker_size` must match the printed black square in centimetres.

### 6.4 Inspect the result

```bash
ros2 topic echo /ar_pose_marker --once
ros2 run tf2_ros tf2_echo usb_cam ar_marker_0
```

In RViz2, set the fixed frame to `usb_cam`, add a **Marker** display on
`/visualization_marker`, and add a **TF** display. The marker pose is also
available on `/ar_pose_marker` in `output_frame`.

---

## 7. Generating markers with `createMarker`

`createMarker` is the ROS 2 build of ALVAR's `SampleMarkerCreator`. It writes
PNG images (and, for multi-marker layouts, an XML placement file) into the
**current working directory**.

### 7.1 Basic usage

```bash
ros2 run ar_track_alvar createMarker [options] <id-or-content> ...
```

With no arguments it starts an interactive placement prompt and prints a short
help message.

Examples:

```bash
# One 9 cm marker with ID 0 (default units: cm, default size: 9 cm)
ros2 run ar_track_alvar createMarker 0

# One 5 cm marker with ID 42
ros2 run ar_track_alvar createMarker -ucm -s 5 42

# Two markers on one sheet: ID 7 at (0, 0), ID 8 at (6, 0) cm
ros2 run ar_track_alvar createMarker -ucm -s 5 -xy 0 0 7 -xy 6 0 8

# A 2x2 array of 5 cm markers with 6 cm pitch, IDs 0..3 (see 7.3 for the input)
printf '2\n2\n6\n6\n0\n0\n' | \
  ros2 run ar_track_alvar createMarker -ar -ucm -s 5
```

**Passing several bare IDs in one call does not lay them out.** All of them are
drawn at the current position (`0,0` unless `-xy` or `-ar` moves it), so the
markers overlap and only the last one is visible in the PNG. To make a
multi-marker sheet, use `-xy` before each ID, use `-ar`, or run `createMarker`
once per marker.

### 7.2 Options

| Option | Meaning |
| --- | --- |
| `<number>` | Add a numeric marker with that ID. May be repeated. |
| `-f` | Force the stronger Hamming(8,4) code. Put the ID after it, e.g. `-f 65535`. |
| `-1 "<text>"` | Marker whose payload is a string. |
| `-2 <file>` | Marker whose payload is a file reference. |
| `-3 <url>` | Marker whose payload is a URL. |
| `-u <pixels>` | Units: 1 unit = `<pixels>` pixels at 96 dpi. |
| `-uin` | Units are inches (96 pixels per inch). |
| `-ucm` | Units are centimetres (96/2.54 pixels per cm). This is the default. |
| `-s <size>` | Marker side length in the current units (default 9.0). |
| `-r <res>` | Accepted for compatibility. For `MarkerData` markers ALVAR chooses the smallest content resolution that fits the payload; see the caveat below. |
| `-m <cells>` | Black border width in cells (default 2). `0` also means 2. |
| `-a` | Generate an ArToolkit-style marker instead of an ALVAR `MarkerData` marker. |
| `-p` | Prompt interactively for marker ID and x/y placement. |
| `-xy <x> <y>` | Place the next marker at the given position, in current units. |
| `-ar` | Interactive array mode; see below. |

### 7.3 Array mode (`-ar`)

`-ar` asks six questions on standard input:

1. number of rows,
2. number of columns,
3. row distance,
4. column distance,
5. column-major (`1`) or row-major (`0`),
6. first marker ID.

The row/column distances use the current units. The IDs are assigned
consecutively from the first ID.

**Important:** do not append marker IDs after `-ar`. Any bare argument is
treated as an extra marker and is added *before* the array, which produces
duplicated IDs (for example `MarkerData_0_1_0_1.png`). Use either
`createMarker -ar ...` with piped answers, or explicit IDs, not both.

### 7.4 Output files

* Numeric markers: `MarkerData_<id>.png`; multiple markers are combined into
  one sheet such as `MarkerData_0_1_2_3.png`.
* More than one marker also produces `MarkerData_<ids>.xml`, which contains
  corner coordinates for each marker. Read the warning below before using it.
* String/file/URL markers use a normalised version of the text in the file
  name.
* ArToolkit markers are named `MarkerArtoolkit_<id>.png`.

**Do not use the XML written by `createMarker` as a `bundle_files` input.**
The XML is produced through the unmodified ALVAR core function
`MultiMarker::PointCloudCorners3d()`, which contains an upstream defect: for
corner index 0 it leaves the template point at `(0, 0, 0, 1)` (the
`-0.5 * edge_length` assignments are commented out in the upstream source), so
the first `<corner>` of every marker is written at the marker **origin** instead
of `(-size/2, -size/2, 0)`. The other three corners are correct. A bundle file
with that corner layout does not describe the printed sheet.

Use one of the following instead:

* the example bundles installed in `share/ar_track_alvar/bundles/`;
* an XML produced by `trainMarkerBundle` (then inspect it, because that path
  initialises the master marker through the same function);
* a hand-authored XML whose corners follow the pattern in
  [section 13.1](#131-bundle-xml-format).

Always check that the four corners of each marker form a square whose side
length equals `marker_size` and whose centre matches the marker's position on
the board.

### 7.5 `-r` caveat

For `MarkerData` markers the content resolution is selected automatically by
`MarkerData::SetContent()`: it starts at 5x5 and increases while the payload
does not fit. The `-r` value passed to the `MarkerData` constructor is
overwritten by that selection, so `createMarker -r 7 0` still produces a 5x5
marker. To obtain a 7x7 marker, use a payload that needs it (for example an ID
of 256 or more), or use a string payload. `-r` is still useful for
`MarkerArtoolkit` markers (`-a`).

### 7.6 Content types

The ROS 2 nodes detect `MarkerData` markers and publish a numeric `id`.
Numeric markers are the intended input. String/file/URL payloads are
generated by ALVAR but the ROS nodes are not designed around them. ArToolkit
markers (`-a`) belong to a different ALVAR marker class and are not detected
by these ROS 2 nodes.

---

## 8. Marker IDs, resolution and printing

### 8.1 ID ranges

ALVAR `MarkerData` uses square content grids with a mandatory Hamming code.
The practical ranges with the default settings are:

| Content grid | IDs | Node setting |
| --- | --- | --- |
| 5x5 | 0-255 | `marker_resolution:=5` (default) |
| 7x7 | 256-32767 | `marker_resolution:=7` |

`createMarker` chooses 5x5 for IDs 0-255 and 7x7 for larger IDs. The node must
be told the same resolution:

```bash
ros2 run ar_track_alvar individualMarkersNoKinect --ros-args \
  -p marker_size:=9.0 -p marker_resolution:=7 -p output_frame:=usb_cam \
  -r camera_image:=/image_raw -r camera_info:=/camera_info
```

Do not run one detector with mixed 5x5 and 7x7 markers in view. If you need
both, use two detector instances or regenerate all markers at the same
resolution. `marker_margin` must also match the `-m` value used by
`createMarker` (default 2).

### 8.2 What `marker_size` means

`marker_size` is the side length of the **outer black square**, in
centimetres. It is not the width of the white paper and it is not the width of
the data area. The ALVAR "margin" is the black border between the data cells
and the outer edge; it is part of the marker.

The PNG written by `createMarker -ucm -s 9` is 340x340 pixels (9 cm at the
assumed 96 dpi) and contains only the black square; the white quiet zone around
it is not part of the file. When printing:

* print at 100% / "actual size", not "fit to page";
* keep the aspect ratio;
* measure the black square and use that measured value as `marker_size`;
* leave white paper around the black square (at least a few millimetres, more
  if the background is dark);
* use matte paper and avoid glare if possible.

If the printed marker is 4.4 cm wide, pass `marker_size:=4.4`, not 9.0.

### 8.3 Example marker assets

The package installs three example sheets from upstream:

| File | Contents |
| --- | --- |
| `markers/Markers_0_to_8.png` | markers 0-8 |
| `markers/Markers_9_to_17.png` | markers 9-17 |
| `markers/table_8_9_10.png` | markers 8, 9 and 10 as used by `table_8_9_10.xml` |

They can be located with:

```bash
ros2 pkg prefix ar_track_alvar
# then look in share/ar_track_alvar/markers/
```

These assets are examples from the upstream package. Their printed size and
layout must match the `marker_size` and bundle XML used by your setup.

---

## 9. Nodes

All five nodes subscribe to `camera_info` (only the **first** message is used)
and to `camera_image` (remapped to the actual input topic). The Kinect variants
expect a point cloud on `camera_image`; the NoKinect variants expect an image.

| Executable | `camera_image` type | Purpose |
| --- | --- | --- |
| `individualMarkersNoKinect` | `sensor_msgs/Image` | Detect and track individual markers |
| `individualMarkers` | `sensor_msgs/PointCloud2` | Individual markers, pose refined with PCL plane fitting |
| `findMarkerBundlesNoKinect` | `sensor_msgs/Image` | Estimate the pose of multi-marker bundles |
| `findMarkerBundles` | `sensor_msgs/PointCloud2` | Bundle pose refined with PCL plane fitting |
| `trainMarkerBundle` | `sensor_msgs/Image` | Interactive bundle training (OpenCV window) |

The default node name for all five is `marker_detect`. Use
`-r __node:=<name>` to rename, or use the launch files, which set the name to
`ar_track_alvar`.

### 9.1 `individualMarkersNoKinect`

```bash
ros2 run ar_track_alvar individualMarkersNoKinect --ros-args \
  -p marker_size:=9.0 \
  -p output_frame:=camera \
  -r camera_image:=/camera/image_raw \
  -r camera_info:=/camera/camera_info
```

* Publishes every detected marker on `ar_pose_marker`, with the pose expressed
  in `output_frame`.
* Publishes one `visualization_marker` cube per marker in the input image
  frame.
* Broadcasts TF `camera_frame -> ar_marker_<id>` for each detected marker.
* Subscribes to `~/enable_detection` (`std_msgs/Bool`) to toggle detection at
  runtime. The `enabled` parameter does the same.
* Supports runtime changes of `marker_size`, `marker_resolution`,
  `marker_margin`, `max_new_marker_error`, `max_track_error`, `max_frequency`
  and `enabled`.

### 9.2 `individualMarkers`

```bash
ros2 run ar_track_alvar individualMarkers --ros-args \
  -p marker_size:=4.4 \
  -p output_frame:=camera \
  -r camera_image:=/camera/depth_registered/points \
  -r camera_info:=/camera/rgb/camera_info
```

* Input must be an **organized** (`height > 1`) `sensor_msgs/PointCloud2` with
  XYZ and RGB fields, with depth in metres and registered to the RGB camera
  whose `camera_info` is supplied. The node converts the cloud to both an
  image and a `pcl::PointXYZRGB` cloud. Unorganized or colourless clouds do
  not work.
* Pose is refined by fitting a plane to the 3D points inside the marker.
* Also publishes `ARmarker_points` (`visualization_msgs/Marker`) with debug
  spheres and arrows.
* Broadcasts TF `cloud_frame -> ar_marker_<id>`.
* `output_frame_from_msg` (default `false`): when `true`, use the point
  cloud's header frame as the output frame instead of `output_frame`.
* Does **not** subscribe to `~/enable_detection`; use the `enabled` parameter.

### 9.3 `findMarkerBundlesNoKinect`

```bash
ros2 run ar_track_alvar findMarkerBundlesNoKinect --ros-args \
  -p marker_size:=9.0 \
  -p output_frame:=camera \
  -p bundle_files:="$HOME/ar_bundles/mmker.xml" \
  -r camera_image:=/camera/image_raw \
  -r camera_info:=/camera/camera_info
```

* Loads one or more bundle XML files at startup and estimates each bundle's
  master pose from all visible member markers.
* `ar_pose_marker` contains **only the master marker of each bundle**, not the
  non-master members. The `confidence` field is left at 0 by this node.
* Broadcasts TF `output_frame -> ar_marker_<master_id>` only. It does not
  broadcast TF for the non-master members.
* Subscribes to `~/enable_detection`.
* Supports runtime changes of `marker_size`, `max_new_marker_error`,
  `max_track_error`, `max_frequency` and `enabled`. `marker_resolution` and
  `marker_margin` require a restart in this node.

### 9.4 `findMarkerBundles`

```bash
ros2 run ar_track_alvar findMarkerBundles --ros-args \
  -p marker_size:=4.4 \
  -p output_frame:=camera \
  -p bundle_files:="$HOME/ar_bundles/mmker.xml" \
  -p med_filt_size:=10 \
  -r camera_image:=/camera/depth_registered/points \
  -r camera_info:=/camera/rgb/camera_info
```

* Same bundle logic as above, but takes an organized RGB point cloud and
  refines the master pose with PCL plane fitting.
* `ar_pose_marker` contains only the master marker of each bundle.
  `confidence` is set to the number of visible markers in that bundle.
* Broadcasts TF `cloud_frame -> ar_marker_<id>` for every detected marker and
  for each bundle master.
* Publishes `ARmarker_points` for debugging.
* `med_filt_size` (default 10) is the window size of the median filter applied
  to the bundle pose; `0` disables the filter.
* `max_track_error` defaults to 0.2 here, but the example PR2 launch file uses
  0.0. Use the value that matches your setup.

### 9.5 `trainMarkerBundle`

```bash
ros2 run ar_track_alvar trainMarkerBundle --ros-args \
  -p nof_markers:=8 \
  -p marker_size:=4.4 \
  -p output_frame:=camera \
  -r camera_image:=/camera/image_raw \
  -r camera_info:=/camera/camera_info
```

* Opens an OpenCV window. The terminal must have a display; for a headless
  machine run it under Xvfb and forward the display, or use a desktop session.
* `nof_markers` is the number of markers in the bundle, and the markers must
  have IDs **0 .. nof_markers-1**.
* Keyboard shortcuts (select the OpenCV window first):

| Key | Action |
| --- | --- |
| `l` | Load `mmarker.xml` from the current directory |
| `s` | Save `mmarker.xml` to the current directory |
| `r` | Reset all measurements and the bundle |
| `p` | Add the current detection as a measurement |
| `a` | Auto-add one measurement per second for five seconds |
| `o` | Optimize the bundle |
| `q` | Quit |

The full training workflow is in
[section 13.2](#132-training-a-bundle-with-trainmarkerbundle).

---

## 10. Parameters

### 10.1 Common parameters

| Parameter | Type | Default | Nodes | Meaning |
| --- | --- | --- | --- | --- |
| `marker_size` | double | 10.0 | all | Outer black square side length in **centimetres** |
| `max_new_marker_error` | double | 0.08 | all | ALVAR detector tolerance; keep the default unless tuning |
| `max_track_error` | double | 0.2 | all | ALVAR tracking tolerance; keep the default unless tuning |
| `output_frame` | string | `""` | all | Frame in which marker poses are reported; must be set (except when `output_frame_from_msg` is true) |
| `max_frequency` | double | 8.0 | trackers | Maximum processing rate in Hz; `0` disables the gate |
| `marker_resolution` | int | 5 | trackers | Content grid size; must match the marker |
| `marker_margin` | int | 2 | trackers | Black border width in cells; must match the marker |
| `enabled` | bool | true | trackers | Initial detection state |
| `output_frame_from_msg` | bool | false | `individualMarkers` only | Use the point cloud header frame as `output_frame` |
| `bundle_files` | string | `""` | bundle nodes | Space-separated list of bundle XML files |
| `med_filt_size` | int | 10 | `findMarkerBundles` only | Median-filter window for the bundle pose; `0` disables |
| `nof_markers` | int | 1 | `trainMarkerBundle` | Number of markers in the bundle (IDs 0..N-1) |

`max_new_marker_error` is the largest accepted content-decoding and margin
error ratio (default 0.08), and `max_track_error` is the largest accepted
corner-tracking error, measured as a ratio of corner displacement to the marker
diagonal in the image (default 0.2). Both are dimensionless error thresholds,
**not** lengths in centimetres. Leave them at their defaults unless you have a
specific reason to tune them.

### 10.2 Runtime changes

The tracker nodes accept parameter updates while running:

```bash
# If started with ros2 run (default node name is marker_detect):
ros2 param set /marker_detect marker_size 4.4

# If started with one of the launch files (node name is ar_track_alvar):
ros2 param set /ar_track_alvar marker_size 4.4

ros2 param list /ar_track_alvar
```

Applied at runtime by all tracker nodes: `marker_size`,
`max_new_marker_error`, `max_track_error`, `max_frequency`, `enabled`.

Applied at runtime by `individualMarkersNoKinect` only: `marker_resolution`,
`marker_margin`. Other nodes read those two at startup.

Not applied at runtime: `output_frame`, `output_frame_from_msg`,
`bundle_files`, `med_filt_size`, `nof_markers`. Restart the node after
changing them.

### 10.3 Example parameter file

`config/params.yaml` contains the common defaults:

```yaml
/**:
  ros__parameters:
    marker_size: 10.0
    max_new_marker_error: 0.08
    max_track_error: 0.2
```

Pass it in addition to the node-specific parameters:

```bash
ros2 run ar_track_alvar individualMarkersNoKinect --ros-args \
  --params-file $(ros2 pkg prefix ar_track_alvar)/share/ar_track_alvar/config/params.yaml \
  -p output_frame:=camera \
  -r camera_image:=/camera/image_raw \
  -r camera_info:=/camera/camera_info
```

---

## 11. Topics and TF

### 11.1 Topics

| Topic | Type | Direction | Nodes |
| --- | --- | --- | --- |
| `camera_image` | `sensor_msgs/Image` or `sensor_msgs/PointCloud2` | subscribe | all |
| `camera_info` | `sensor_msgs/CameraInfo` | subscribe | all |
| `~/enable_detection` | `std_msgs/Bool` | subscribe | `individualMarkersNoKinect`, `findMarkerBundlesNoKinect` |
| `ar_pose_marker` | `ar_track_alvar_msgs/AlvarMarkers` | publish | all trackers and `trainMarkerBundle` |
| `visualization_marker` | `visualization_msgs/Marker` | publish | all trackers and `trainMarkerBundle` |
| `ARmarker_points` | `visualization_msgs/Marker` | publish | `individualMarkers`, `findMarkerBundles` |
| `/tf` | `tf2_msgs/TFMessage` | publish | all trackers and `trainMarkerBundle` |

The camera inputs use the sensor-data QoS profile (best effort). The image
input is subscribed through `image_transport` with the **`raw`** transport;
compressed transports are not enabled in these nodes.

`~/enable_detection` is a private topic. With the default node name it is
`/marker_detect/enable_detection`; with the launch files it is
`/ar_track_alvar/enable_detection`:

```bash
ros2 topic pub --once /ar_track_alvar/enable_detection std_msgs/msg/Bool "{data: false}"
```

### 11.2 Message layout

`ar_track_alvar_msgs/msg/AlvarMarker`:

```text
std_msgs/Header header
uint32 id
uint32 confidence
geometry_msgs/PoseStamped pose
```

`ar_track_alvar_msgs/msg/AlvarMarkers`:

```text
std_msgs/Header header
AlvarMarker[] markers
```

* `AlvarMarkers.header` and each `AlvarMarker.header` have
  `frame_id = output_frame` and the input image/cloud timestamp.
* The nested `pose.header` is left **empty** by the nodes. Read the frame and
  time from the outer `AlvarMarker.header` (or `AlvarMarkers.header`), not from
  `pose.header`.
* Translations are in **metres** (ALVAR works in centimetres internally; the
  nodes divide by 100 before publishing).
* `confidence` is only filled by `findMarkerBundles` (Kinect bundle node) with
  the number of visible markers; other nodes leave it at 0.

### 11.3 TF

TF child frames are always named `ar_marker_<id>`.

| Node | Broadcast |
| --- | --- |
| `individualMarkersNoKinect` | `camera_frame -> ar_marker_<id>` for each detected marker |
| `individualMarkers` | `cloud_frame -> ar_marker_<id>` for each detected marker |
| `findMarkerBundlesNoKinect` | `output_frame -> ar_marker_<master_id>` for each seen bundle |
| `findMarkerBundles` | `cloud_frame -> ar_marker_<id>` for each detected marker and each bundle master |
| `trainMarkerBundle` | `camera_frame -> ar_marker_0` (the master) |

The asymmetry between `findMarkerBundles` and `findMarkerBundlesNoKinect` is
inherited from the ROS 1 package: the NoKinect bundle node broadcasts only the
master pose, in `output_frame`, while the Kinect bundle node broadcasts every
detected marker in the cloud frame. Keep this in mind when composing TF trees.

If `output_frame` is not connected to the camera frame in TF, the nodes log an
error and fall back to the identity transform for the `ar_pose_marker` output.
They still publish, but the pose is then effectively in the camera frame.

---

## 12. Launch files

| Launch file | Node | `marker_size` | Input topic | `output_frame` | Notes |
| --- | --- | --- | --- | --- | --- |
| `ar_track.launch.py` | `individualMarkersNoKinect` | 9.0 | `/usb_cam/image_raw` | `usb_cam` | Adds example static TFs `world->map` and `base_link->usb_cam` |
| `pr2_indiv.launch.py` | `individualMarkers` | 4.4 | `/kinect_head/depth_registered/points` | `torso_lift_link` | Kinect point cloud |
| `pr2_indiv_no_kinect.launch.py` | `individualMarkersNoKinect` | 4.4 | `/wide_stereo/left/image_color` | `torso_lift_link` | Stereo image |
| `pr2_bundle.launch.py` | `findMarkerBundles` | 4.4 | `/kinect_head/depth_registered/points` | `torso_lift_link` | `max_track_error=0.0`, `med_filt_size=10`, example bundles |
| `pr2_bundle_no_kinect.launch.py` | `findMarkerBundlesNoKinect` | 4.4 | `/wide_stereo/left/image_color` | `torso_lift_link` | Example bundles |
| `pr2_train.launch.py` | `trainMarkerBundle` | 4.4 | `/wide_stereo/left/image_color` | `torso_lift_link` | `nof_markers=8` |

All launch files accept the ROS 1 argument names:

* `marker_size`
* `max_new_marker_error`
* `max_track_error`
* `cam_image_topic`
* `cam_info_topic`
* `output_frame`
* `bundle_files`, `med_filt_size` (bundle launches)
* `nof_markers` (training launch)

Example overrides:

```bash
ros2 launch ar_track_alvar pr2_indiv_no_kinect.launch.py \
  marker_size:=9.0 \
  cam_image_topic:=/my_camera/image_raw \
  cam_info_topic:=/my_camera/camera_info \
  output_frame:=my_camera

ros2 launch ar_track_alvar pr2_bundle_no_kinect.launch.py \
  marker_size:=9.0 \
  cam_image_topic:=/my_camera/image_raw \
  cam_info_topic:=/my_camera/camera_info \
  output_frame:=my_camera \
  bundle_files:="$HOME/ar_bundles/mmker.xml"
```

`ar_track.launch.py` includes two **example** static transform publishers
(`world -> map` and `base_link -> usb_cam`). They are convenient for a quick
test but may conflict with your robot's TF tree. Comment them out or use your
own TF publisher in a real system. The launch file sets `output_frame:=usb_cam`
and expects the camera driver to stamp images with the same frame; if your
driver uses another frame (the `usb_cam` default is `default_cam`), override
`output_frame` or set the driver's `frame_id` parameter accordingly. Its
default topic names (`/usb_cam/image_raw`, `/usb_cam/camera_info`) assume the
camera node runs in the `/usb_cam` namespace; override `cam_image_topic` and
`cam_info_topic` otherwise.

The default bundles used by the PR2 bundle launch files are
`truthTableLeg.xml` and `table_8_9_10.xml` from the installed package share
directory. `pr2_bundle.launch.py` also sets `max_track_error` to 0.0, matching
the upstream ROS 1 launch file.

---

## 13. Bundle files and bundle training

A bundle is a set of markers mounted rigidly together. The bundle XML records
where each marker's four corners are in a common coordinate frame, in
centimetres. At runtime the node detects any subset of the members and
computes the pose of the **master** marker.

### 13.1 Bundle XML format

```xml
<?xml version="1.0" encoding="UTF-8" standalone="no" ?>
<multimarker markers="3">
    <marker index="8" status="1">
        <corner x="-2.2" y="-2.2" z="0" />
        <corner x="2.2" y="-2.2" z="0" />
        <corner x="2.2" y="2.2" z="0" />
        <corner x="-2.2" y="2.2" z="0" />
    </marker>
    <marker index="9" status="1">
        <corner x="6.8" y="-2.2" z="0" />
        <corner x="11.2" y="-2.2" z="0" />
        <corner x="11.2" y="2.2" z="0" />
        <corner x="6.8" y="2.2" z="0" />
    </marker>
    <marker index="10" status="1">
        <corner x="-2.2" y="-11.2" z="0" />
        <corner x="2.2" y="-11.2" z="0" />
        <corner x="2.2" y="-6.8" z="0" />
        <corner x="-2.2" y="-6.8" z="0" />
    </marker>
</multimarker>
```

Rules:

* `markers="N"` must equal the number of `<marker>` entries.
* The **first** `<marker>` entry is the bundle's master marker. The node
  reports the master pose on `ar_pose_marker` and (for the NoKinect node) in
  TF.
* `index` is the printed marker ID. It must match the marker image.
* `status="1"` marks the marker as part of the bundle.
* The four corners are listed in order. In ALVAR's bundle frame, the edge from
  the first corner to the second is the marker's +x direction and the edge
  from the second to the third is +y. For a flat bundle all `z` values are 0.
* All coordinates are in centimetres. `marker_size` is still needed for
  detection and is not inferred from the XML.

`bundle_files` is a **single string** containing space-separated paths. Quote
it in the shell and in launch arguments. Absolute paths are recommended:

```bash
-p bundle_files:="/home/user/bundles/left.xml /home/user/bundles/right.xml"
```

If a file cannot be opened, the node logs a fatal error and shuts down.

### 13.2 Training a bundle with `trainMarkerBundle`

1. Generate markers with IDs `0 .. N-1`, where `N = nof_markers`. For example,
   for four 9 cm markers, print them as one 2x2 sheet with 10 cm pitch:

   ```bash
   cd ~/ar_markers
   printf '2\n2\n10\n10\n0\n0\n' | \
     ros2 run ar_track_alvar createMarker -ar -ucm -s 9
   ```

   Or generate four separate PNG files:

   ```bash
   cd ~/ar_markers
   for id in 0 1 2 3; do ros2 run ar_track_alvar createMarker -ucm -s 9 "$id"; done
   ```

   Do not pass several IDs to a single `createMarker` call: they would overlap
   (see [section 7.1](#71-basic-usage)). Training does not read the XML emitted
   by `createMarker`; its broken corner convention is described in
   [section 7.4](#74-output-files).

2. Mount the markers rigidly on a flat board. Keep the layout fixed during
   training and during later use.

3. Start the trainer with a camera that publishes an image and camera info:

   ```bash
   ros2 run ar_track_alvar trainMarkerBundle --ros-args \
     -p nof_markers:=4 \
     -p marker_size:=9.0 \
     -p output_frame:=camera \
     -r camera_image:=/camera/image_raw \
     -r camera_info:=/camera/camera_info
   ```

   Alternatively:

   ```bash
   ros2 launch ar_track_alvar pr2_train.launch.py \
     nof_markers:=4 marker_size:=9.0 \
     cam_image_topic:=/camera/image_raw \
     cam_info_topic:=/camera/camera_info \
     output_frame:=camera
   ```

4. Select the OpenCV window. Show at least **two** markers in each view and
   press `p` to add a measurement. Move and rotate the board between
   measurements so that every marker is seen from several viewpoints. Each
   marker must appear in at least two measurements for the initialiser to
   work.

5. Press `o` and wait for `Optimizing done`. If optimisation fails, add more
   measurements from different angles and try again.

6. Press `s` to save `mmarker.xml` in the current working directory. It is a
   normal bundle XML file, but the trainer initialises the master marker
   through the same corner function described in
   [section 7.4](#74-output-files): check that the master's four corners form
   a square of side `marker_size` before passing it to a bundle node. If the
   first corner sits at `(0, 0, 0)` while the others are at +/- size/2, correct
   that first corner or use a hand-authored file.

7. Press `q` to quit.

`a` captures one measurement per second for five seconds, which is useful for
a quick sweep. `l` loads an existing `mmarker.xml` if you want to continue a
previous session. `r` clears all measurements.

The trainer saves to `mmarker.xml` (two `m`s) in the directory from which the
node was started; it does not write to the package share directory.

---

## 14. Units and coordinate conventions

* `marker_size`, the `-s` option of `createMarker`, and the corner
  coordinates in bundle XML are in **centimetres**.
* Published translations in `ar_pose_marker` and TF are in **metres**.
* Poses are marker poses in `output_frame`. When `output_frame` is the camera
  optical frame, this follows ALVAR's camera convention: x right, y down,
  z forward. The marker's own axes follow ALVAR's marker convention; the port
  does not rotate them. Inspect the result with `tf2_echo` or RViz before
  relying on axis directions in a controller.
* `marker_size` must be the outer black square, measured after printing.
* The image/cloud header timestamp is used for the TF transform and for the
  `max_frequency` gate. If your driver publishes zero or non-monotonic
  timestamps, the gate may drop frames; set `max_frequency:=0.0` for
  debugging.

---

## 15. Troubleshooting

### No markers are detected

* Measure the printed black square and make sure `marker_size` matches it.
  A mismatch in `marker_size` does not always prevent detection, but it makes
  the reported distance wrong.
* Check `marker_resolution` and `marker_margin`. A 5x5 marker needs
  `marker_resolution:=5`; a 7x7 marker needs `marker_resolution:=7`. The
  margin must match the `-m` value used to generate the marker (default 2).
* IDs above 255 are 7x7 markers. If the node is left at 5, they are decoded
  incorrectly or not at all.
* The marker must be large enough in the image. Move closer, use a larger
  print, or use a higher-resolution camera.
* Avoid motion blur, glare and very low contrast. Matte paper helps.
* Check that `camera_info` arrived and that its `k`/`d` values are correct.
  The node drops images until the first camera-info message arrives, and it
  never updates the calibration afterwards. Restart the node after changing
  calibration.
* Check that `enabled` is true. In the NoKinect nodes, also check the last
  message on `~/enable_detection`.
* Check that the image is a raw `bgr8`, `rgb8` or `mono8` image. Compressed
  images are not subscribed by these nodes.

### The marker is detected but the distance or orientation is wrong

* Re-measure the printed marker and update `marker_size`.
* Keep the marker flat. Kinect nodes fit a plane and are more robust to small
  deviations; the NoKinect nodes assume a planar marker.
* Make sure `output_frame` is connected to the camera frame in TF. If the
  lookup fails, the node logs an error and falls back to identity, so the pose
  is effectively in the camera frame while being labelled `output_frame`.
* Check for a unit mismatch: `marker_size` is cm, but published positions are
  m.
* For a bundle, check the XML corners. If the master's first corner is at
  `(0, 0, 0)` instead of `(-size/2, -size/2, 0)`, the reported pose is offset;
  see [section 7.4](#74-output-files).

### `~/.ros/log` reports a TF error, or the pose frame is wrong

* ROS 2 frame IDs must not start with `/`. Use `usb_cam`, not `/usb_cam`.
* `output_frame` must exist in the TF tree. Publish a static transform or use
  the camera frame itself.
* In `ar_track.launch.py`, the two static transforms are examples and may
  conflict with your robot. Remove them if you already publish those frames.

### Bundle pose is not published

* At least one member marker must be visible. The master marker does not have
  to be visible.
* Check that every marker ID in the XML matches a printed marker and that the
  markers are rigidly mounted in the recorded layout.
* Check the node log at startup: a missing or malformed bundle file causes a
  fatal error and shutdown.
* Remember that bundle nodes publish only the **master** marker in
  `ar_pose_marker`; non-master members appear only as RViz markers (and, for
  `findMarkerBundles`, as TF frames).

### The Kinect node detects nothing

* The `camera_image` input must be an organized point cloud with XYZ and RGB,
  depth in metres, registered to the RGB camera used for `camera_info`.
* An unorganized cloud (`height == 1`) cannot be indexed by pixel coordinates
  and will not work.
* Check that `camera_info` is the RGB camera info, not the depth camera info.
* Try `individualMarkersNoKinect` on the RGB image first to confirm that the
  marker itself is detectable.

### `ros2 topic echo` shows nothing

* Check the node name. With `ros2 run` it is `/marker_detect`; with a launch
  file it is `/ar_track_alvar`.
* Check that the node is still running and that `camera_info` and
  `camera_image` are being received.
* The published topics are relative names. If you started the node in a
  namespace, use the fully qualified topic name.
* If you use a custom QoS subscriber, make sure it is compatible with the
  publisher's default QoS.

### `trainMarkerBundle` does not open a window

* It needs a display. On a headless machine, run it under Xvfb and use a
  virtual display, or use a desktop session.
* Select the OpenCV window before pressing keys; otherwise the key events go
  to the terminal.
* Training requires at least two markers in each measurement and at least two
  measurements per marker.

### `createMarker` does not produce a file

* It writes to the current working directory. Check that the directory is
  writable.
* With `-ar`, the six prompts read from standard input. Pipe the answers or
  type them interactively; do not append IDs after `-ar`.
* `-r` does not force the resolution of `MarkerData` markers; see
  [section 7.5](#75--r-caveat).

---

## 16. Verification status

The port was built from source and exercised on ROS 2 Humble / Ubuntu 22.04
with synthetic inputs:

* Both packages build cleanly with `colcon`.
* `individualMarkersNoKinect` detects IDs 8, 9 and 10 from a rendered
  `table_8_9_10.png` image at approximately 0.13 m, with unit-norm
  quaternions.
* `findMarkerBundlesNoKinect` recovers the master marker (ID 8) of
  `table_8_9_10.xml` from the same image.
* `individualMarkers` and `findMarkerBundles` work with a synthetic organized
  `PointCloud2` (RGB, depth in metres) and produce unit-norm quaternions.
* `createMarker` produces single markers, multi-marker sheets and array
  layouts; IDs 255 and 256 were checked to select 5x5 and 7x7 grids
  respectively.
* `trainMarkerBundle` starts under Xvfb, receives camera info and opens its
  OpenCV window. A full training session with a physical board was not
  performed.

Not verified:

* Physical USB, Kinect, RealSense or stereo cameras.
* A real PR2 robot or the original PR2 launch scenarios.
* ROS 2 distributions other than Humble.
* Long-running stability, CPU load or latency on embedded hardware.

If you test the package on real hardware, contributions with reproducible
results are welcome.

---

## 17. Known limitations

* **No compressed image transport.** The image subscribers use the `raw`
  transport only.
* **No depth-to-colour registration.** The Kinect nodes require an already
  registered, organized RGB point cloud.
* **Bundle output contains only masters.** `ar_pose_marker` reports the master
  marker of each bundle, matching the ROS 1 behavior. Non-master members are
  visualization/TF only.
* **TF asymmetry.** `findMarkerBundlesNoKinect` broadcasts only the master in
  `output_frame`; `findMarkerBundles` broadcasts all detected markers in the
  cloud frame.
* **Restart required for some parameters.** `marker_resolution` and
  `marker_margin` are runtime-settable only in `individualMarkersNoKinect`.
  `output_frame`, `output_frame_from_msg`, `bundle_files`, `med_filt_size` and
  `nof_markers` are read at startup.
* **Only the first `camera_info` message is used.** The calibration is stored
  once at startup; later `camera_info` messages are ignored. Restart the node
  after changing calibration.
* **`createMarker` writes a broken multi-marker XML.** The upstream ALVAR
  `MultiMarker::PointCloudCorners3d()` puts the first corner of each marker at
  the marker origin, so the exported XML is not a valid bundle layout. See
  [section 7.4](#74-output-files).
* **`createMarker -r` is ineffective for `MarkerData`.** The resolution is
  chosen from the payload; see [section 7.5](#75--r-caveat).
* **String/file/URL markers are not a supported ROS input.** The nodes publish
  a numeric `id` and are designed for numeric markers.
* **ArToolkit markers are not detected by these nodes.** `createMarker -a`
  generates markers for ALVAR's `MarkerArtoolkit` class; the ROS nodes use
  `MarkerData`.
* **No ROS 2 lifecycle or component nodes.** The executables are ordinary
  `rclcpp` nodes with their own `main()`.
* **ROS 1 tests are not included.** See
  [section 2](#2-what-the-port-changed).
* **Training needs a GUI.** `trainMarkerBundle` uses an OpenCV window and
  keyboard shortcuts.

---

## 18. Credits

* **ALVAR 2.0.0** — VTT Technical Research Centre of Finland. ALVAR is the
  AR tag tracking library at the core of this package.
* **`ar_track_alvar` ROS wrapper** — Scott Niekum and contributors. The
  original ROS 1 package, its nodes, launch scenarios, bundle files, marker
  images and documentation are the basis of this port.
* **Kinect plane fitting and median filter** — Willow Garage, Inc.
* **ROS 2 port** — the contributors of this repository.

Original source: <https://github.com/ros-perception/ar_track_alvar>, branch
`noetic-devel`, commit
`a857aab4d6ac144478fb92f6624e0088473f4b33`.

If this port is useful in academic work, please cite or acknowledge the
original ALVAR and `ar_track_alvar` projects as appropriate. This repository
does not grant permission to use the names or logos of the original authors or
organisations to imply endorsement.
