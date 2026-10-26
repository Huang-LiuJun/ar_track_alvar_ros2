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

#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <ar_track_alvar_msgs/msg/alvar_marker.hpp>
#include <ar_track_alvar_msgs/msg/alvar_markers.hpp>

#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/MultiMarkerBundle.h"
#include "ar_track_alvar/MultiMarkerInitializer.h"
#include "ar_track_alvar/Shared.h"

using namespace alvar;

#define MAIN_MARKER 1
#define VISIBLE_MARKER 2
#define GHOST_MARKER 3

class FindMarkerBundlesNoKinect : public rclcpp::Node
{
public:
  FindMarkerBundlesNoKinect()
    : Node("marker_detect"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    marker_size_ = declare_parameter("marker_size", 10.0);
    max_new_marker_error_ = declare_parameter("max_new_marker_error", 0.08);
    max_track_error_ = declare_parameter("max_track_error", 0.2);
    max_frequency_ = declare_parameter("max_frequency", 8.0);
    marker_resolution_ = declare_parameter("marker_resolution", 5);
    marker_margin_ = declare_parameter("marker_margin", 2);
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
      multi_marker_bundles_.push_back(std::move(bundle));
    }
    n_bundles_ = static_cast<int>(multi_marker_bundles_.size());
    bundle_poses_.resize(n_bundles_);
    bundles_seen_.resize(n_bundles_, false);
    for (int i = 0; i < n_bundles_; i++)
      bundle_poses_[i].Reset();

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    ar_marker_pub_ = create_publisher<ar_track_alvar_msgs::msg::AlvarMarkers>(
        "ar_pose_marker", 0);
    rviz_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "visualization_marker", 0);

    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", rclcpp::SensorDataQoS(),
        std::bind(&FindMarkerBundlesNoKinect::camInfoCallback, this,
                  std::placeholders::_1));

    image_sub_ = image_transport::create_subscription(
        this, "camera_image",
        std::bind(&FindMarkerBundlesNoKinect::imageCallback, this,
                  std::placeholders::_1),
        "raw", rmw_qos_profile_sensor_data);

    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        "~/enable_detection", 1,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          enabled_ = msg->data;
        });

    param_cb_ = add_on_set_parameters_callback(
        std::bind(&FindMarkerBundlesNoKinect::paramCallback, this,
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

  // Updates the bundle poses by detecting markers and using all markers in a
  // bundle to infer the master tag's position
  void getMultiMarkerPoses(cv::Mat& image)
  {
    if (marker_detector_.Detect(image, &cam_, true, false,
                                max_new_marker_error_, max_track_error_, CVSEQ,
                                true))
    {
      for (int i = 0; i < n_bundles_; i++)
        multi_marker_bundles_[i]->Update(marker_detector_.markers, &cam_,
                                         bundle_poses_[i]);

      if (marker_detector_.DetectAdditional(image, &cam_, false) > 0)
      {
        for (int i = 0; i < n_bundles_; i++)
        {
          if (multi_marker_bundles_[i]->SetTrackMarkers(
                  marker_detector_, &cam_, bundle_poses_[i], image) > 0)
            multi_marker_bundles_[i]->Update(marker_detector_.markers, &cam_,
                                             bundle_poses_[i]);
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
                      ar_track_alvar_msgs::msg::AlvarMarker* ar_pose_marker)
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

    rviz_marker->lifetime = rclcpp::Duration(1, 0);

    // Only publish the pose of the master tag in each bundle, since that's all
    // we really care about aside from visualization
    if (type == MAIN_MARKER)
    {
      tf2::Transform tag_pose_output = cam_to_output * marker_pose;

      tf2::toMsg(tag_pose_output, ar_pose_marker->pose.pose);
      ar_pose_marker->header.frame_id = output_frame_;
      ar_pose_marker->header.stamp = image_msg->header.stamp;
      ar_pose_marker->id = id;

      std::string marker_frame = "ar_marker_";
      std::stringstream out;
      out << id;
      marker_frame += out.str();

      geometry_msgs::msg::TransformStamped output_to_marker;
      output_to_marker.header.stamp = image_msg->header.stamp;
      output_to_marker.header.frame_id = output_frame_;
      output_to_marker.child_frame_id = marker_frame;
      tf2::toMsg(tag_pose_output, output_to_marker.transform);
      tf_broadcaster_->sendTransform(output_to_marker);
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg)
  {
    if (!enabled_ || !cam_.getCamInfo_)
      return;

    const rclcpp::Time stamp(image_msg->header.stamp,
                             get_clock()->get_clock_type());
    if (max_frequency_ > 0.0 && last_frame_time_.nanoseconds() != 0 &&
        (stamp - last_frame_time_).seconds() < 1.0 / max_frequency_)
      return;
    last_frame_time_ = stamp;

    try
    {
      // Get the transformation from the camera to the output frame for this
      // image capture
      tf2::Transform cam_to_output = tf2::Transform::getIdentity();
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

      visualization_msgs::msg::Marker rviz_marker;
      ar_track_alvar_msgs::msg::AlvarMarker ar_pose_marker;
      ar_pose_markers_.markers.clear();

      cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

      getMultiMarkerPoses(cv_ptr_->image);

      // Draw the observed markers that are visible and note which bundles have
      // at least 1 marker seen
      for (int i = 0; i < n_bundles_; i++)
        bundles_seen_[i] = false;

      for (auto& marker : *marker_detector_.markers)
      {
        int id = static_cast<int>(marker.GetId());

        // Draw if id is valid
        if (id >= 0)
        {
          // Mark the bundle that marker belongs to as "seen"
          for (int j = 0; j < n_bundles_; j++)
          {
            for (size_t k = 0; k < bundle_indices_[j].size(); k++)
            {
              if (bundle_indices_[j][k] == id)
              {
                bundles_seen_[j] = true;
                break;
              }
            }
          }

          // Don't draw if it is a master tag...we do this later, a bit
          // differently
          bool should_draw = true;
          for (int i = 0; i < n_bundles_; i++)
          {
            if (id == master_id_[i])
              should_draw = false;
          }
          if (should_draw)
          {
            Pose p = marker.pose;
            makeMarkerMsgs(VISIBLE_MARKER, id, p, image_msg, cam_to_output,
                           &rviz_marker, &ar_pose_marker);
            rviz_marker_pub_->publish(rviz_marker);
          }
        }
      }

      // Draw the main markers, whether they are visible or not -- but only if
      // at least 1 marker from their bundle is currently seen
      for (int i = 0; i < n_bundles_; i++)
      {
        if (bundles_seen_[i])
        {
          makeMarkerMsgs(MAIN_MARKER, master_id_[i], bundle_poses_[i],
                         image_msg, cam_to_output, &rviz_marker,
                         &ar_pose_marker);
          rviz_marker_pub_->publish(rviz_marker);
          ar_pose_markers_.markers.push_back(ar_pose_marker);
        }
      }

      ar_marker_pub_->publish(ar_pose_markers_);
    }
    catch (const cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Could not convert from '%s' to 'rgb8'.",
                   image_msg->encoding.c_str());
    }
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
  std::vector<bool> bundles_seen_;
  int n_bundles_ = 0;

  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Publisher<ar_track_alvar_msgs::msg::AlvarMarkers>::SharedPtr
      ar_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_marker_pub_;
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
  rclcpp::Time last_frame_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<FindMarkerBundlesNoKinect>();
    rclcpp::spin(node);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("find_marker_bundles_no_kinect"), "%s",
                 e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
