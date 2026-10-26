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

#include "ar_track_alvar/CvTestbed.h"
#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/Shared.h"

using namespace alvar;

class IndividualMarkersNoKinect : public rclcpp::Node
{
public:
  IndividualMarkersNoKinect()
    : Node("marker_detect"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    marker_size_ = declare_parameter("marker_size", 10.0);
    max_new_marker_error_ = declare_parameter("max_new_marker_error", 0.08);
    max_track_error_ = declare_parameter("max_track_error", 0.2);
    max_frequency_ = declare_parameter("max_frequency", 8.0);
    marker_resolution_ = declare_parameter("marker_resolution", 5);
    marker_margin_ = declare_parameter("marker_margin", 2);
    output_frame_ = declare_parameter("output_frame", std::string(""));
    enabled_ = declare_parameter("enabled", true);

    if (output_frame_.empty())
    {
      RCLCPP_FATAL(get_logger(), "Param 'output_frame' has to be set.");
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

    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", rclcpp::SensorDataQoS(),
        std::bind(&IndividualMarkersNoKinect::camInfoCallback, this,
                  std::placeholders::_1));

    image_sub_ = image_transport::create_subscription(
        this, "camera_image",
        std::bind(&IndividualMarkersNoKinect::imageCallback, this,
                  std::placeholders::_1),
        "raw", rmw_qos_profile_sensor_data);

    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
        "~/enable_detection", 1,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          enabled_ = msg->data;
        });

    param_cb_ = add_on_set_parameters_callback(
        std::bind(&IndividualMarkersNoKinect::paramCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "AR tracker started: output_frame=%s",
                output_frame_.c_str());
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
      else if (name == "marker_resolution")
      {
        marker_resolution_ = param.as_int();
        marker_detector_.SetMarkerSize(marker_size_, marker_resolution_,
                                       marker_margin_);
      }
      else if (name == "marker_margin")
      {
        marker_margin_ = param.as_int();
        marker_detector_.SetMarkerSize(marker_size_, marker_resolution_,
                                       marker_margin_);
      }
    }
    RCLCPP_INFO(get_logger(), "AR tracker reconfigured: %s %.2f %.2f %.2f %.2f",
                enabled_ ? "ENABLED" : "DISABLED", max_frequency_, marker_size_,
                max_new_marker_error_, max_track_error_);
    return result;
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg)
  {
    if (!enabled_ || !cam_.getCamInfo_)
      return;

    const rclcpp::Time stamp(image_msg->header.stamp, get_clock()->get_clock_type());
    if (max_frequency_ > 0.0 && last_frame_time_.nanoseconds() != 0 &&
        (stamp - last_frame_time_).seconds() < 1.0 / max_frequency_)
      return;
    last_frame_time_ = stamp;

    try
    {
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

      cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

      marker_detector_.Detect(cv_ptr_->image, &cam_, true, false,
                              max_new_marker_error_, max_track_error_, CVSEQ,
                              true);

      ar_pose_markers_.markers.clear();
      for (size_t i = 0; i < marker_detector_.markers->size(); i++)
      {
        int id = (*(marker_detector_.markers))[i].GetId();
        Pose p = (*(marker_detector_.markers))[i].pose;
        double px = p.translation[0] / 100.0;
        double py = p.translation[1] / 100.0;
        double pz = p.translation[2] / 100.0;

        cv::Mat quat = cv::Mat(4, 1, CV_64F);
        p.GetQuaternion(quat);
        double qx = quat.at<double>(1, 0);
        double qy = quat.at<double>(2, 0);
        double qz = quat.at<double>(3, 0);
        double qw = quat.at<double>(0, 0);

        tf2::Quaternion rotation(qx, qy, qz, qw);
        tf2::Vector3 origin(px, py, pz);
        tf2::Transform t(rotation, origin);
        tf2::Transform marker_pose = t;

        tf2::Vector3 z_axis_cam = tf2::quatRotate(rotation, tf2::Vector3(0, 0, 1));
        // as we can't see through markers, this one is a false positive
        if (z_axis_cam.z() > 0)
        {
          continue;
        }

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
      ar_pose_markers_.header.frame_id = output_frame_;
      ar_marker_pub_->publish(ar_pose_markers_);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Error in ar_track_alvar callback: %s", e.what());
    }
  }

  Camera cam_;
  MarkerDetector<MarkerData> marker_detector_;
  cv_bridge::CvImagePtr cv_ptr_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Publisher<ar_track_alvar_msgs::msg::AlvarMarkers>::SharedPtr
      ar_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_marker_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  ar_track_alvar_msgs::msg::AlvarMarkers ar_pose_markers_;
  visualization_msgs::msg::Marker rviz_marker_;

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
    auto node = std::make_shared<IndividualMarkersNoKinect>();
    rclcpp::spin(node);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("individual_markers_no_kinect"), "%s",
                 e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
