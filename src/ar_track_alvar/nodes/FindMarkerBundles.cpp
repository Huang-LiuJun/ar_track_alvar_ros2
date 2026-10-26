/*
  Software License Agreement (BSD License)
  Copyright (c) 2012, Scott Niekum
  All rights reserved.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:
  * Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the following
  disclaimer in the documentation and/or other materials provided
  with the distribution.
  * Neither the name of the Willow Garage nor the names of its
  contributors may be used to endorse or promote products derived
  from this software without specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.

  author: Scott Niekum
*/
#include <algorithm>
#include <cmath>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ar_track_alvar_msgs/msg/alvar_marker.hpp>
#include <ar_track_alvar_msgs/msg/alvar_markers.hpp>

#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/MultiMarkerBundle.h"
#include "ar_track_alvar/MultiMarkerInitializer.h"
#include "ar_track_alvar/filter/kinect_filtering.h"
#include "ar_track_alvar/filter/medianFilter.h"

#define MAIN_MARKER 1
#define VISIBLE_MARKER 2
#define GHOST_MARKER 3

namespace gm = geometry_msgs::msg;
namespace ata = ar_track_alvar;

typedef pcl::PointXYZRGB ARPoint;
typedef pcl::PointCloud<ARPoint> ARCloud;

using namespace alvar;

class FindMarkerBundles : public rclcpp::Node
{
public:
  FindMarkerBundles()
    : Node("marker_detect"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    marker_size_ = declare_parameter("marker_size", 10.0);
    max_new_marker_error_ = declare_parameter("max_new_marker_error", 0.08);
    max_track_error_ = declare_parameter("max_track_error", 0.2);
    max_frequency_ = declare_parameter("max_frequency", 8.0);
    marker_resolution_ = declare_parameter("marker_resolution", 5);
    marker_margin_ = declare_parameter("marker_margin", 2);
    med_filt_size_ = declare_parameter("med_filt_size", 10);
    const std::string bundle_string =
        declare_parameter("bundle_files", std::string(""));
    output_frame_ = declare_parameter("output_frame", std::string(""));
    enabled_ = declare_parameter("enabled", true);

    if (output_frame_.empty())
    {
      RCLCPP_FATAL(get_logger(), "Param 'output_frame' has to be set.");
      rclcpp::shutdown();
      throw std::runtime_error("Param 'output_frame' has to be set.");
    }

    std::vector<std::string> bundle_files;
    std::stringstream ss(bundle_string);
    std::string token;
    while (std::getline(ss, token, ' '))
    {
      if (!token.empty())
        bundle_files.push_back(token);
    }

    marker_detector_.SetMarkerSize(marker_size_, marker_resolution_,
                                   marker_margin_);

    // Load the marker bundle XML files
    for (const auto& bundle_file : bundle_files)
    {
      MultiMarker load_helper;
      if (!load_helper.Load(bundle_file.c_str(), FILE_FORMAT_XML))
      {
        RCLCPP_FATAL(get_logger(), "Cannot load file %s", bundle_file.c_str());
        rclcpp::shutdown();
        throw std::runtime_error("Cannot load bundle file: " + bundle_file);
      }
      std::vector<int> id_vector = load_helper.getIndices();
      auto bundle = std::make_unique<MultiMarkerBundle>(id_vector);
      bundle->Load(bundle_file.c_str(), FILE_FORMAT_XML);
      master_id_.push_back(bundle->getMasterId());
      bundle_indices_.push_back(bundle->getIndices());
      calcAndSaveMasterCoords(*bundle);
      multi_marker_bundles_.push_back(std::move(bundle));
    }
    n_bundles_ = static_cast<int>(multi_marker_bundles_.size());
    bundle_poses_.resize(n_bundles_);
    bundles_seen_.resize(n_bundles_, 0);
    master_visible_.resize(n_bundles_, false);
    for (int i = 0; i < n_bundles_; i++)
      bundle_poses_[i].Reset();

    // Create median filters
    for (int i = 0; i < n_bundles_; i++)
      med_filts_.push_back(std::make_unique<ata::MedianFilter>(med_filt_size_));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    ar_marker_pub_ = create_publisher<ar_track_alvar_msgs::msg::AlvarMarkers>(
        "ar_pose_marker", 0);
    rviz_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "visualization_marker", 0);
    rviz_marker_pub2_ = create_publisher<visualization_msgs::msg::Marker>(
        "ARmarker_points", 0);

    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", rclcpp::SensorDataQoS(),
        std::bind(&FindMarkerBundles::camInfoCallback, this,
                  std::placeholders::_1));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "camera_image", rclcpp::SensorDataQoS(),
        std::bind(&FindMarkerBundles::pointCloudCallback, this,
                  std::placeholders::_1));

    param_cb_ = add_on_set_parameters_callback(
        std::bind(&FindMarkerBundles::paramCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "AR bundle tracker started: %d bundle(s)",
                n_bundles_);
  }

private:
  void camInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (cam_.getCamInfo_)
      return;
    cam_.SetCameraInfo(*msg);
    cam_.getCamInfo_ = true;
    RCLCPP_INFO(get_logger(), "Camera info received (%dx%d)", msg->width,
                msg->height);
  }

  rcl_interfaces::msg::SetParametersResult paramCallback(
      const std::vector<rclcpp::Parameter>& params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto& param : params)
    {
      const std::string& name = param.get_name();
      if (name == "enabled")
        enabled_ = param.as_bool();
      else if (name == "max_frequency")
        max_frequency_ = param.as_double();
      else if (name == "marker_size")
      {
        marker_size_ = param.as_double();
        marker_detector_.SetMarkerSize(marker_size_, marker_resolution_,
                                       marker_margin_);
      }
      else if (name == "max_new_marker_error")
        max_new_marker_error_ = param.as_double();
      else if (name == "max_track_error")
        max_track_error_ = param.as_double();
    }
    RCLCPP_INFO(get_logger(), "AR tracker reconfigured: %s %.2f %.2f %.2f %.2f",
                enabled_ ? "ENABLED" : "DISABLED", max_frequency_, marker_size_,
                max_new_marker_error_, max_track_error_);
    return result;
  }

  // Debugging utility function
  void draw3dPoints(const ARCloud::Ptr& cloud, const std::string& frame,
                    int color, int id, double rad)
  {
    visualization_msgs::msg::Marker rviz_marker;

    rviz_marker.header.frame_id = frame;
    rviz_marker.header.stamp = now();
    rviz_marker.id = id;
    rviz_marker.ns = "3dpts";

    rviz_marker.scale.x = rad;
    rviz_marker.scale.y = rad;
    rviz_marker.scale.z = rad;

    rviz_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    rviz_marker.action = visualization_msgs::msg::Marker::ADD;

    if (color == 1)
    {
      rviz_marker.color.r = 0.0f;
      rviz_marker.color.g = 1.0f;
      rviz_marker.color.b = 1.0f;
      rviz_marker.color.a = 1.0;
    }
    if (color == 2)
    {
      rviz_marker.color.r = 1.0f;
      rviz_marker.color.g = 0.0f;
      rviz_marker.color.b = 1.0f;
      rviz_marker.color.a = 1.0;
    }
    if (color == 3)
    {
      rviz_marker.color.r = 1.0f;
      rviz_marker.color.g = 1.0f;
      rviz_marker.color.b = 0.0f;
      rviz_marker.color.a = 1.0;
    }

    gm::Point p;
    for (auto& point : cloud->points)
    {
      p.x = point.x;
      p.y = point.y;
      p.z = point.z;
      rviz_marker.points.push_back(p);
    }

    rviz_marker.lifetime = rclcpp::Duration(1, 0);
    rviz_marker_pub2_->publish(rviz_marker);
  }

  void drawArrow(const gm::Point& start, const tf2::Matrix3x3& mat,
                 const std::string& frame, int color, int id)
  {
    visualization_msgs::msg::Marker rviz_marker;

    rviz_marker.header.frame_id = frame;
    rviz_marker.header.stamp = now();
    rviz_marker.id = id;
    rviz_marker.ns = "arrow";

    rviz_marker.scale.x = 0.01;
    rviz_marker.scale.y = 0.01;
    rviz_marker.scale.z = 0.1;

    rviz_marker.type = visualization_msgs::msg::Marker::ARROW;
    rviz_marker.action = visualization_msgs::msg::Marker::ADD;

    for (int i = 0; i < 3; i++)
    {
      rviz_marker.points.clear();
      rviz_marker.points.push_back(start);
      gm::Point end;
      end.x = start.x + mat[0][i];
      end.y = start.y + mat[1][i];
      end.z = start.z + mat[2][i];
      rviz_marker.points.push_back(end);
      rviz_marker.id += 10 * i;
      rviz_marker.lifetime = rclcpp::Duration(1, 0);

      if (color == 1)
      {
        rviz_marker.color.r = 1.0f;
        rviz_marker.color.g = 0.0f;
        rviz_marker.color.b = 0.0f;
        rviz_marker.color.a = 1.0;
      }
      if (color == 2)
      {
        rviz_marker.color.r = 0.0f;
        rviz_marker.color.g = 1.0f;
        rviz_marker.color.b = 0.0f;
        rviz_marker.color.a = 1.0;
      }
      if (color == 3)
      {
        rviz_marker.color.r = 0.0f;
        rviz_marker.color.g = 0.0f;
        rviz_marker.color.b = 1.0f;
        rviz_marker.color.a = 1.0;
      }
      color += 1;

      rviz_marker_pub2_->publish(rviz_marker);
    }
  }

  // Infer the master tag corner positions from the other observed tags.
  // Also does some of the bookkeeping for tracking that MultiMarker::_GetPose
  // does.
  int inferCorners(const ARCloud& cloud, MultiMarkerBundle& master,
                   ARCloud& bund_corners)
  {
    bund_corners.clear();
    bund_corners.resize(4);
    for (int i = 0; i < 4; i++)
    {
      bund_corners[i].x = 0;
      bund_corners[i].y = 0;
      bund_corners[i].z = 0;
    }

    // Reset the marker_status to 1 for all markers in point_cloud for tracking
    // purposes
    for (int& marker_status : master.marker_status)
    {
      if (marker_status > 0)
        marker_status = 1;
    }

    int n_est = 0;

    // For every detected marker
    for (auto& i : *marker_detector_.markers)
    {
      const Marker* marker = &i;
      int id = static_cast<int>(marker->GetId());
      int index = master.get_id_index(id);
      if (index < 0)
        continue;

      // But only if we have corresponding points in the pointcloud
      if (master.marker_status[index] > 0 && marker->valid)
      {
        n_est++;

        std::string marker_frame = "ar_marker_";
        std::stringstream mark_out;
        mark_out << id;
        marker_frame += mark_out.str();

        // Grab the precomputed corner coords and correct for the weird Alvar
        // coord system
        for (int j = 0; j < 4; ++j)
        {
          const cv::Point3d& corner_coord = master.rel_corners[index][j];
          gm::PointStamped p, output_p;
          p.header.frame_id = marker_frame;
          p.point.x = corner_coord.y / 100.0;
          p.point.y = -corner_coord.x / 100.0;
          p.point.z = corner_coord.z / 100.0;

          try
          {
            output_p = tf_buffer_.transform(p, cloud.header.frame_id,
                                            tf2::durationFromSec(0.1));
          }
          catch (const tf2::TransformException& ex)
          {
            RCLCPP_ERROR(get_logger(), "ERROR InferCorners: %s", ex.what());
            return -1;
          }

          bund_corners[j].x += output_p.point.x;
          bund_corners[j].y += output_p.point.y;
          bund_corners[j].z += output_p.point.z;
        }
        master.marker_status[index] = 2;  // Used for tracking
      }
    }

    // Divide to take the average of the summed estimates
    if (n_est > 0)
    {
      for (int i = 0; i < 4; i++)
      {
        bund_corners[i].x /= static_cast<float>(n_est);
        bund_corners[i].y /= static_cast<float>(n_est);
        bund_corners[i].z /= static_cast<float>(n_est);
      }
    }

    return 0;
  }

  int planeFitPoseImprovement(int id, const ARCloud& corners_3D,
                              const ARCloud::Ptr& selected_points,
                              const ARCloud& cloud, Pose& p)
  {
    ata::PlaneFitResult res = ata::fitPlane(selected_points);
    gm::PoseStamped pose;
    pose.header.stamp = pcl_conversions::fromPCL(cloud.header).stamp;
    pose.header.frame_id = cloud.header.frame_id;
    pose.pose.position = ata::centroid(*res.inliers);

    draw3dPoints(selected_points, cloud.header.frame_id, 1, id, 0.005);

    // Get 2 points that point forward in marker x direction
    int i1, i2;
    if (std::isnan(corners_3D[0].x) || std::isnan(corners_3D[0].y) ||
        std::isnan(corners_3D[0].z) || std::isnan(corners_3D[3].x) ||
        std::isnan(corners_3D[3].y) || std::isnan(corners_3D[3].z))
    {
      if (std::isnan(corners_3D[1].x) || std::isnan(corners_3D[1].y) ||
          std::isnan(corners_3D[1].z) || std::isnan(corners_3D[2].x) ||
          std::isnan(corners_3D[2].y) || std::isnan(corners_3D[2].z))
      {
        return -1;
      }
      i1 = 1;
      i2 = 2;
    }
    else
    {
      i1 = 0;
      i2 = 3;
    }

    // Get 2 points the point forward in marker y direction
    int i3, i4;
    if (std::isnan(corners_3D[0].x) || std::isnan(corners_3D[0].y) ||
        std::isnan(corners_3D[0].z) || std::isnan(corners_3D[1].x) ||
        std::isnan(corners_3D[1].y) || std::isnan(corners_3D[1].z))
    {
      if (std::isnan(corners_3D[3].x) || std::isnan(corners_3D[3].y) ||
          std::isnan(corners_3D[3].z) || std::isnan(corners_3D[2].x) ||
          std::isnan(corners_3D[2].y) || std::isnan(corners_3D[2].z))
      {
        return -1;
      }
      i3 = 2;
      i4 = 3;
    }
    else
    {
      i3 = 1;
      i4 = 0;
    }

    ARCloud::Ptr orient_points(new ARCloud());
    orient_points->points.push_back(corners_3D[i1]);
    draw3dPoints(orient_points, cloud.header.frame_id, 3, id + 1000, 0.008);

    orient_points->clear();
    orient_points->points.push_back(corners_3D[i2]);
    draw3dPoints(orient_points, cloud.header.frame_id, 2, id + 2000, 0.008);

    int succ;
    succ = ata::extractOrientation(res.coeffs, corners_3D[i1], corners_3D[i2],
                                   corners_3D[i3], corners_3D[i4],
                                   pose.pose.orientation);
    if (succ < 0)
      return -1;

    tf2::Matrix3x3 mat;
    succ = ata::extractFrame(res.coeffs, corners_3D[i1], corners_3D[i2],
                             corners_3D[i3], corners_3D[i4], mat);
    if (succ < 0)
      return -1;

    drawArrow(pose.pose.position, mat, cloud.header.frame_id, 1, id);

    p.translation[0] = pose.pose.position.x * 100.0;
    p.translation[1] = pose.pose.position.y * 100.0;
    p.translation[2] = pose.pose.position.z * 100.0;
    p.quaternion[1] = pose.pose.orientation.x;
    p.quaternion[2] = pose.pose.orientation.y;
    p.quaternion[3] = pose.pose.orientation.z;
    p.quaternion[0] = pose.pose.orientation.w;

    return 0;
  }

  // Updates the bundle poses by detecting markers and using all markers in a
  // bundle to infer the master tag's position
  void getMultiMarkerPoses(cv::Mat& image, ARCloud& cloud)
  {
    for (int i = 0; i < n_bundles_; i++)
    {
      master_visible_[i] = false;
      bundles_seen_[i] = 0;
    }

    // Detect and track the markers
    int detected = marker_detector_.Detect(
        image, &cam_, true, false, max_new_marker_error_, max_track_error_,
        CVSEQ, true);
    if (detected)
    {
      for (size_t i = 0; i < marker_detector_.markers->size(); i++)
      {
        std::vector<cv::Point, Eigen::aligned_allocator<cv::Point>> pixels;
        Marker* m = &((*marker_detector_.markers)[i]);
        int id = static_cast<int>(m->GetId());

        // Get the 3D inner corner points - more stable than outer corners that
        // can "fall off" object
        int resol = m->GetRes();
        int ori = m->ros_orientation;

        PointDouble pt1, pt2, pt3, pt4;
        pt4 = m->ros_marker_points_img[0];
        pt3 = m->ros_marker_points_img[resol - 1];
        pt1 = m->ros_marker_points_img[(resol * resol) - resol];
        pt2 = m->ros_marker_points_img[(resol * resol) - 1];

        m->ros_corners_3D[0] = cloud(pt1.x, pt1.y);
        m->ros_corners_3D[1] = cloud(pt2.x, pt2.y);
        m->ros_corners_3D[2] = cloud(pt3.x, pt3.y);
        m->ros_corners_3D[3] = cloud(pt4.x, pt4.y);

        if (ori >= 0 && ori < 4)
        {
          if (ori != 0)
          {
            std::rotate(m->ros_corners_3D.begin(),
                        m->ros_corners_3D.begin() + ori,
                        m->ros_corners_3D.end());
          }
        }
        else
        {
          RCLCPP_ERROR(get_logger(),
                       "FindMarkerBundles: Bad Orientation: %i for ID: %i", ori,
                       id);
        }

        // Check if we have spotted a master tag
        int master_ind = -1;
        for (int j = 0; j < n_bundles_; j++)
        {
          if (id == master_id_[j])
            master_visible_[j] = true;
          master_ind = j;
        }

        // Mark the bundle that marker belongs to as "seen"
        int bundle_ind = -1;
        for (int j = 0; j < n_bundles_; j++)
        {
          for (size_t k = 0; k < bundle_indices_[j].size(); k++)
          {
            if (bundle_indices_[j][k] == id)
            {
              bundle_ind = j;
              bundles_seen_[j] += 1;
              break;
            }
          }
        }

        // Get the 3D marker points
        for (const PointDouble& p : m->ros_marker_points_img)
          pixels.emplace_back(p.x, p.y);
        ARCloud::Ptr selected_points = ata::filterCloud(cloud, pixels);

        // Use the kinect data to find a plane and pose for the marker
        int ret = planeFitPoseImprovement(i, m->ros_corners_3D, selected_points,
                                          cloud, m->pose);

        // If the plane fit fails...
        if (ret < 0)
        {
          // Mark this tag as invalid
          m->valid = false;
          // If this was a master tag, reset its visibility
          if (master_ind >= 0)
            master_visible_[master_ind] = false;
          // decrement the number of markers seen in this bundle
          if (bundle_ind >= 0)
            bundles_seen_[bundle_ind] -= 1;
        }
        else
          m->valid = true;
      }

      // For each master tag, infer the 3D position of its corners from other
      // visible tags. Then, do a plane fit to those new corners.
      ARCloud inferred_corners;
      for (int i = 0; i < n_bundles_; i++)
      {
        if (bundles_seen_[i] > 0)
        {
          if (inferCorners(cloud, *(multi_marker_bundles_[i]),
                           inferred_corners) >= 0)
          {
            ARCloud::Ptr inferred_cloud(new ARCloud(inferred_corners));
            planeFitPoseImprovement(i + 5000, inferred_corners, inferred_cloud,
                                    cloud, bundle_poses_[i]);
          }
          Pose ret_pose;
          if (med_filt_size_ > 0)
          {
            med_filts_[i]->addPose(bundle_poses_[i]);
            med_filts_[i]->getMedian(ret_pose);
            bundle_poses_[i] = ret_pose;
          }
        }
      }
    }
  }

  // Given the pose of a marker, builds the appropriate ROS messages for later
  // publishing
  void makeMarkerMsgs(int type, int id, Pose& p,
                      const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                      const tf2::Transform& cam_to_output,
                      visualization_msgs::msg::Marker* rviz_marker,
                      ar_track_alvar_msgs::msg::AlvarMarker* ar_pose_marker,
                      int confidence)
  {
    const double px = p.translation[0] / 100.0;
    const double py = p.translation[1] / 100.0;
    const double pz = p.translation[2] / 100.0;
    const double qx = p.quaternion[1];
    const double qy = p.quaternion[2];
    const double qz = p.quaternion[3];
    const double qw = p.quaternion[0];

    // Get the marker pose in the camera frame
    tf2::Quaternion rotation(qx, qy, qz, qw);
    tf2::Vector3 origin(px, py, pz);
    tf2::Transform t(rotation, origin);  // transform from cam to marker
    tf2::Transform marker_pose = t;

    // Publish the cam to marker transform for each marker
    std::string marker_frame = "ar_marker_";
    std::stringstream out;
    out << id;
    marker_frame += out.str();

    geometry_msgs::msg::TransformStamped cam_to_marker;
    cam_to_marker.header.stamp = image_msg->header.stamp;
    cam_to_marker.header.frame_id = image_msg->header.frame_id;
    cam_to_marker.child_frame_id = marker_frame;
    tf2::toMsg(t, cam_to_marker.transform);
    tf_broadcaster_->sendTransform(cam_to_marker);

    // Create the rviz visualization message
    tf2::toMsg(marker_pose, rviz_marker->pose);
    rviz_marker->header.frame_id = image_msg->header.frame_id;
    rviz_marker->header.stamp = image_msg->header.stamp;
    rviz_marker->id = id;

    rviz_marker->scale.x = 1.0 * marker_size_ / 100.0;
    rviz_marker->scale.y = 1.0 * marker_size_ / 100.0;
    rviz_marker->scale.z = 0.2 * marker_size_ / 100.0;

    if (type == MAIN_MARKER)
      rviz_marker->ns = "main_shapes";
    else
      rviz_marker->ns = "basic_shapes";

    rviz_marker->type = visualization_msgs::msg::Marker::CUBE;
    rviz_marker->action = visualization_msgs::msg::Marker::ADD;

    // Determine a color and opacity, based on marker type
    if (type == MAIN_MARKER)
    {
      rviz_marker->color.r = 1.0f;
      rviz_marker->color.g = 0.0f;
      rviz_marker->color.b = 0.0f;
      rviz_marker->color.a = 1.0;
    }
    else if (type == VISIBLE_MARKER)
    {
      rviz_marker->color.r = 0.0f;
      rviz_marker->color.g = 1.0f;
      rviz_marker->color.b = 0.0f;
      rviz_marker->color.a = 0.7;
    }
    else if (type == GHOST_MARKER)
    {
      rviz_marker->color.r = 0.0f;
      rviz_marker->color.g = 0.0f;
      rviz_marker->color.b = 1.0f;
      rviz_marker->color.a = 0.5;
    }

    rviz_marker->lifetime = rclcpp::Duration(0, 100000000);

    // Only publish the pose of the master tag in each bundle, since that's all
    // we really care about aside from visualization
    if (type == MAIN_MARKER)
    {
      tf2::Transform tag_pose_output = cam_to_output * marker_pose;

      tf2::toMsg(tag_pose_output, ar_pose_marker->pose.pose);
      ar_pose_marker->header.frame_id = output_frame_;
      ar_pose_marker->header.stamp = image_msg->header.stamp;
      ar_pose_marker->id = id;
      ar_pose_marker->confidence = confidence;
    }
  }

  // Callback to handle getting kinect point clouds and processing them
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!enabled_)
      return;

    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    if (max_frequency_ > 0.0 && last_frame_time_.nanoseconds() != 0 &&
        (stamp - last_frame_time_).seconds() < 1.0 / max_frequency_)
      return;
    last_frame_time_ = stamp;

    auto image_msg = std::make_shared<sensor_msgs::msg::Image>();

    if (!cam_.getCamInfo_)
      return;

    try
    {
      // Get the transformation from the camera to the output frame for this
      // image capture
      tf2::Transform cam_to_output = tf2::Transform::getIdentity();
      try
      {
        const auto transform = tf_buffer_.lookupTransform(
            output_frame_, msg->header.frame_id, msg->header.stamp,
            tf2::durationFromSec(1.0));
        tf2::fromMsg(transform.transform, cam_to_output);
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_ERROR(get_logger(), "%s", ex.what());
      }

      // Init and clear visualization markers
      visualization_msgs::msg::Marker rviz_marker;
      ar_track_alvar_msgs::msg::AlvarMarker ar_pose_marker;
      ar_pose_markers_.markers.clear();

      // Convert cloud to PCL
      ARCloud cloud;
      pcl::fromROSMsg(*msg, cloud);

      // Get an OpenCV image from the cloud
      pcl::toROSMsg(*msg, *image_msg);
      image_msg->header.stamp = msg->header.stamp;
      image_msg->header.frame_id = msg->header.frame_id;

      cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

      getMultiMarkerPoses(cv_ptr_->image, cloud);

      for (auto& marker : *marker_detector_.markers)
      {
        int id = static_cast<int>(marker.GetId());

        // Draw if id is valid
        if (id >= 0)
        {
          // Don't draw if it is a master tag...we do this later, a bit
          // differently
          bool should_draw = true;
          for (int j = 0; j < n_bundles_; j++)
          {
            if (id == master_id_[j])
              should_draw = false;
          }
          if (should_draw && marker.valid)
          {
            Pose p = marker.pose;
            makeMarkerMsgs(VISIBLE_MARKER, id, p, image_msg, cam_to_output,
                           &rviz_marker, &ar_pose_marker, 1);
            rviz_marker_pub_->publish(rviz_marker);
          }
        }
      }

      // Draw the main markers, whether they are visible or not -- but only if
      // at least 1 marker from their bundle is currently seen
      for (int i = 0; i < n_bundles_; i++)
      {
        if (bundles_seen_[i] > 0)
        {
          makeMarkerMsgs(MAIN_MARKER, master_id_[i], bundle_poses_[i],
                         image_msg, cam_to_output, &rviz_marker,
                         &ar_pose_marker, bundles_seen_[i]);
          rviz_marker_pub_->publish(rviz_marker);
          ar_pose_markers_.markers.push_back(ar_pose_marker);
        }
      }

      ar_marker_pub_->publish(ar_pose_markers_);
    }
    catch (const cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "ar_track_alvar: Image error: %s",
                   image_msg->encoding.c_str());
    }
  }

  // Create a ROS frame out of the known corners of a tag in the weird marker
  // coord frame used by Alvar markers (x right y forward z up). p0-->p1 should
  // point in Alvar's pos X direction, p1-->p2 should point in Alvar's pos Y
  // direction.
  int makeMasterTransform(const cv::Point3d& p0, const cv::Point3d& p1,
                          const cv::Point3d& p2, const cv::Point3d& p3,
                          tf2::Transform& retT)
  {
    const tf2::Vector3 q0(p0.x, p0.y, p0.z);
    const tf2::Vector3 q1(p1.x, p1.y, p1.z);
    const tf2::Vector3 q2(p2.x, p2.y, p2.z);
    const tf2::Vector3 q3(p3.x, p3.y, p3.z);

    // (inverse) matrix with the given properties
    const tf2::Vector3 v = (q1 - q0).normalized();
    const tf2::Vector3 w = (q2 - q1).normalized();
    const tf2::Vector3 n = v.cross(w);
    tf2::Matrix3x3 m(v[0], v[1], v[2], w[0], w[1], w[2], n[0], n[1], n[2]);
    m = m.inverse();

    // Translate to quaternion
    if (m.determinant() <= 0)
      return -1;

    // Use Eigen for this part, matching the original ROS 1 implementation
    Eigen::Matrix3f eig_m;
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        eig_m(i, j) = m[i][j];
      }
    }
    Eigen::Quaternion<float> eig_quat(eig_m);

    tf2::Quaternion quat(eig_quat.x(), eig_quat.y(), eig_quat.z(),
                         eig_quat.w());
    quat = quat.normalized();

    const double qx = (q0.x() + q1.x() + q2.x() + q3.x()) / 4.0;
    const double qy = (q0.y() + q1.y() + q2.y() + q3.y()) / 4.0;
    const double qz = (q0.z() + q1.z() + q2.z() + q3.z()) / 4.0;
    const tf2::Vector3 origin(qx, qy, qz);

    retT = tf2::Transform(quat, origin);  // transform from master to marker

    return 0;
  }

  // Find the coordinates of the master marker with respect to the coord frame
  // of each of its child markers. This data is used for later estimation of
  // the master marker pose from the child poses.
  int calcAndSaveMasterCoords(MultiMarkerBundle& master)
  {
    int mast_id = master.master_id;
    std::vector<cv::Point3d> rel_corner_coords;

    // Go through all the markers associated with this bundle
    for (size_t i = 0; i < master.marker_indices.size(); i++)
    {
      int mark_id = master.marker_indices[i];
      rel_corner_coords.clear();

      // Get the coords of the corners of the child marker in the master frame
      cv::Point3d mark_corners[4];
      for (int j = 0; j < 4; j++)
      {
        mark_corners[j] = master.pointcloud[master.pointcloud_index(mark_id, j)];
      }

      // Use them to find a transform from the master frame to the child frame
      tf2::Transform tform;
      makeMasterTransform(mark_corners[0], mark_corners[1], mark_corners[2],
                          mark_corners[3], tform);

      // Finally, find the coords of the corners of the master in the child
      // frame
      for (int j = 0; j < 4; j++)
      {
        cv::Point3d corner_coord =
            master.pointcloud[master.pointcloud_index(mast_id, j)];
        const tf2::Vector3 corner_vec(corner_coord.x, corner_coord.y,
                                      corner_coord.z);
        const tf2::Vector3 ans = tform.inverse() * corner_vec;
        rel_corner_coords.emplace_back(ans.x(), ans.y(), ans.z());
      }

      master.rel_corners.push_back(rel_corner_coords);
    }

    return 0;
  }

  Camera cam_;
  MarkerDetector<MarkerData> marker_detector_;
  cv_bridge::CvImagePtr cv_ptr_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::vector<std::unique_ptr<MultiMarkerBundle>> multi_marker_bundles_;
  std::vector<Pose> bundle_poses_;
  std::vector<int> master_id_;
  std::vector<std::vector<int>> bundle_indices_;
  std::vector<int> bundles_seen_;
  std::vector<bool> master_visible_;
  std::vector<std::unique_ptr<ata::MedianFilter>> med_filts_;
  int n_bundles_ = 0;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<ar_track_alvar_msgs::msg::AlvarMarkers>::SharedPtr
      ar_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      rviz_marker_pub2_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  ar_track_alvar_msgs::msg::AlvarMarkers ar_pose_markers_;

  bool enabled_;
  double max_frequency_;
  double marker_size_;
  double max_new_marker_error_;
  double max_track_error_;
  std::string output_frame_;
  int marker_resolution_;
  int marker_margin_;
  int med_filt_size_;
  rclcpp::Time last_frame_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<FindMarkerBundles>();
    rclcpp::spin(node);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("find_marker_bundles"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
