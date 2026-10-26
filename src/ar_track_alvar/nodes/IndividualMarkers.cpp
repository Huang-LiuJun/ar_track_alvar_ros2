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
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
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

#include "ar_track_alvar/CvTestbed.h"
#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/Shared.h"
#include "ar_track_alvar/filter/kinect_filtering.h"

namespace gm = geometry_msgs::msg;
namespace ata = ar_track_alvar;

typedef pcl::PointXYZRGB ARPoint;
typedef pcl::PointCloud<ARPoint> ARCloud;

using namespace alvar;

class IndividualMarkers : public rclcpp::Node
{
public:
  IndividualMarkers()
    : Node("marker_detect"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    marker_size_ = declare_parameter("marker_size", 10.0);
    max_new_marker_error_ = declare_parameter("max_new_marker_error", 0.08);
    max_track_error_ = declare_parameter("max_track_error", 0.2);
    max_frequency_ = declare_parameter("max_frequency", 8.0);
    marker_resolution_ = declare_parameter("marker_resolution", 5);
    marker_margin_ = declare_parameter("marker_margin", 2);
    output_frame_from_msg_ = declare_parameter("output_frame_from_msg", false);
    output_frame_ = declare_parameter("output_frame", std::string(""));
    enabled_ = declare_parameter("enabled", true);

    if (!output_frame_from_msg_ && output_frame_.empty())
    {
      RCLCPP_FATAL(get_logger(),
                   "Param 'output_frame' has to be set if the output frame is "
                   "not derived from the point cloud message.");
      rclcpp::shutdown();
      throw std::runtime_error("Param 'output_frame' has to be set.");
    }

    marker_detector_.SetMarkerSize(marker_size_, marker_resolution_,
                                   marker_margin_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    ar_marker_pub_ = create_publisher<ar_track_alvar_msgs::msg::AlvarMarkers>(
        "ar_pose_marker", 0);
    rviz_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "visualization_marker", 0);
    rviz_marker_pub2_ = create_publisher<visualization_msgs::msg::Marker>(
        "ARmarker_points", 0);

    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", rclcpp::SensorDataQoS(),
        std::bind(&IndividualMarkers::camInfoCallback, this,
                  std::placeholders::_1));

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "camera_image", rclcpp::SensorDataQoS(),
        std::bind(&IndividualMarkers::pointCloudCallback, this,
                  std::placeholders::_1));

    param_cb_ = add_on_set_parameters_callback(
        std::bind(&IndividualMarkers::paramCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "AR tracker started (point cloud input)");
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
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
      p.x = cloud->points[i].x;
      p.y = cloud->points[i].y;
      p.z = cloud->points[i].z;
      rviz_marker.points.push_back(p);
    }

    rviz_marker.lifetime = rclcpp::Duration(1, 0);
    rviz_marker_pub2_->publish(rviz_marker);
  }

  void drawArrow(gm::Point start, tf2::Matrix3x3 mat, std::string frame,
                 int color, int id)
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

  int planeFitPoseImprovement(int id, const ARCloud& corners_3D,
                              ARCloud::Ptr selected_points, const ARCloud& cloud,
                              Pose& p)
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

  void getMarkerPoses(cv::Mat& image, ARCloud& cloud)
  {
    int detected = marker_detector_.Detect(
        image, &cam_, true, false, max_new_marker_error_, max_track_error_,
        CVSEQ, true);
    if (detected)
    {
      for (size_t i = 0; i < marker_detector_.markers->size(); i++)
      {
        std::vector<cv::Point, Eigen::aligned_allocator<cv::Point>> pixels;
        Marker* m = &((*marker_detector_.markers)[i]);
        int id = m->GetId();

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

        // Get the 3D marker points
        for (const PointDouble& p : m->ros_marker_points_img)
          pixels.push_back(cv::Point(p.x, p.y));
        ARCloud::Ptr selected_points = ata::filterCloud(cloud, pixels);

        // Use the kinect data to find a plane and pose for the marker
        planeFitPoseImprovement(i, m->ros_corners_3D, selected_points, cloud,
                                m->pose);
      }
    }
  }

  void pointCloudCallback(
      const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!enabled_)
      return;

    const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
    if (max_frequency_ > 0.0 && last_frame_time_.nanoseconds() != 0 &&
        (stamp - last_frame_time_).seconds() < 1.0 / max_frequency_)
      return;
    last_frame_time_ = stamp;

    auto image_msg = std::make_shared<sensor_msgs::msg::Image>();

    // If desired, use the frame in the message's header.
    if (output_frame_from_msg_)
      output_frame_ = msg->header.frame_id;

    if (!cam_.getCamInfo_)
      return;

    // Convert cloud to PCL
    ARCloud cloud;
    pcl::fromROSMsg(*msg, cloud);

    // Get an OpenCV image from the cloud
    pcl::toROSMsg(*msg, *image_msg);
    image_msg->header.stamp = msg->header.stamp;
    image_msg->header.frame_id = msg->header.frame_id;

    cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

    getMarkerPoses(cv_ptr_->image, cloud);

    tf2::Transform cam_to_output = tf2::Transform::getIdentity();
    if (image_msg->header.frame_id != output_frame_)
    {
      try
      {
        const auto transform = tf_buffer_.lookupTransform(
            output_frame_, image_msg->header.frame_id, image_msg->header.stamp,
            tf2::durationFromSec(1.0));
        tf2::fromMsg(transform.transform, cam_to_output);
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_ERROR(get_logger(), "%s", ex.what());
      }
    }

    try
    {
      ar_pose_markers_.markers.clear();
      for (size_t i = 0; i < marker_detector_.markers->size(); i++)
      {
        int id = (*(marker_detector_.markers))[i].GetId();
        Pose p = (*(marker_detector_.markers))[i].pose;

        double px = p.translation[0] / 100.0;
        double py = p.translation[1] / 100.0;
        double pz = p.translation[2] / 100.0;
        double qx = p.quaternion[1];
        double qy = p.quaternion[2];
        double qz = p.quaternion[3];
        double qw = p.quaternion[0];

        tf2::Quaternion rotation(qx, qy, qz, qw);
        tf2::Vector3 origin(px, py, pz);
        tf2::Transform t(rotation, origin);
        tf2::Transform marker_pose = t;

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

        tf2::toMsg(marker_pose, rviz_marker_.pose);
        rviz_marker_.header.frame_id = image_msg->header.frame_id;
        rviz_marker_.header.stamp = image_msg->header.stamp;
        rviz_marker_.id = id;

        rviz_marker_.scale.x = 1.0 * marker_size_ / 100.0;
        rviz_marker_.scale.y = 1.0 * marker_size_ / 100.0;
        rviz_marker_.scale.z = 0.2 * marker_size_ / 100.0;
        rviz_marker_.ns = "basic_shapes";
        rviz_marker_.type = visualization_msgs::msg::Marker::CUBE;
        rviz_marker_.action = visualization_msgs::msg::Marker::ADD;
        switch (id)
        {
          case 0:
            rviz_marker_.color.r = 0.0f;
            rviz_marker_.color.g = 0.0f;
            rviz_marker_.color.b = 1.0f;
            rviz_marker_.color.a = 1.0;
            break;
          case 1:
            rviz_marker_.color.r = 1.0f;
            rviz_marker_.color.g = 0.0f;
            rviz_marker_.color.b = 0.0f;
            rviz_marker_.color.a = 1.0;
            break;
          case 2:
            rviz_marker_.color.r = 0.0f;
            rviz_marker_.color.g = 1.0f;
            rviz_marker_.color.b = 0.0f;
            rviz_marker_.color.a = 1.0;
            break;
          case 3:
            rviz_marker_.color.r = 0.0f;
            rviz_marker_.color.g = 0.5f;
            rviz_marker_.color.b = 0.5f;
            rviz_marker_.color.a = 1.0;
            break;
          case 4:
            rviz_marker_.color.r = 0.5f;
            rviz_marker_.color.g = 0.5f;
            rviz_marker_.color.b = 0.0;
            rviz_marker_.color.a = 1.0;
            break;
          default:
            rviz_marker_.color.r = 0.5f;
            rviz_marker_.color.g = 0.0f;
            rviz_marker_.color.b = 0.5f;
            rviz_marker_.color.a = 1.0;
            break;
        }
        rviz_marker_.lifetime = rclcpp::Duration(1, 0);
        rviz_marker_pub_->publish(rviz_marker_);

        tf2::Transform tag_pose_output = cam_to_output * marker_pose;

        ar_track_alvar_msgs::msg::AlvarMarker ar_pose_marker;
        tf2::toMsg(tag_pose_output, ar_pose_marker.pose.pose);
        ar_pose_marker.header.frame_id = output_frame_;
        ar_pose_marker.header.stamp = image_msg->header.stamp;
        ar_pose_marker.id = id;
        ar_pose_markers_.markers.push_back(ar_pose_marker);
      }
      ar_pose_markers_.header.stamp = image_msg->header.stamp;
      ar_marker_pub_->publish(ar_pose_markers_);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "ar_track_alvar: Image error: %s", e.what());
    }
  }

  Camera cam_;
  MarkerDetector<MarkerData> marker_detector_;
  cv_bridge::CvImagePtr cv_ptr_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<ar_track_alvar_msgs::msg::AlvarMarkers>::SharedPtr
      ar_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      rviz_marker_pub2_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  ar_track_alvar_msgs::msg::AlvarMarkers ar_pose_markers_;
  visualization_msgs::msg::Marker rviz_marker_;

  bool enabled_;
  bool output_frame_from_msg_;
  double max_frequency_;
  double marker_size_;
  double max_new_marker_error_;
  double max_track_error_;
  std::string output_frame_;
  int marker_resolution_;
  int marker_margin_;
  rclcpp::Time last_frame_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<IndividualMarkers>();
    rclcpp::spin(node);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("individual_markers"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
