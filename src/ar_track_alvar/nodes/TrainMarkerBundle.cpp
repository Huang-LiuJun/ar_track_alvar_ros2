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

#include <chrono>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
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
#include "ar_track_alvar/MultiMarkerBundle.h"
#include "ar_track_alvar/MultiMarkerInitializer.h"
#include "ar_track_alvar/Shared.h"

using namespace alvar;

#define MAIN_MARKER 1
#define VISIBLE_MARKER 2
#define GHOST_MARKER 3

class TrainMarkerBundle : public rclcpp::Node
{
public:
  TrainMarkerBundle()
    : Node("marker_detect"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    nof_markers_ = declare_parameter("nof_markers", 1);
    marker_size_ = declare_parameter("marker_size", 10.0);
    max_new_marker_error_ = declare_parameter("max_new_marker_error", 0.08);
    max_track_error_ = declare_parameter("max_track_error", 0.2);
    output_frame_ = declare_parameter("output_frame", std::string(""));

    if (nof_markers_ < 1)
    {
      RCLCPP_FATAL(get_logger(), "Param 'nof_markers' has to be >= 1.");
      throw std::runtime_error("Param 'nof_markers' has to be >= 1.");
    }
    if (output_frame_.empty())
    {
      RCLCPP_FATAL(get_logger(), "Param 'output_frame' has to be set.");
      throw std::runtime_error("Param 'output_frame' has to be set.");
    }

    marker_detector_.SetMarkerSize(marker_size_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    ar_marker_pub_ = create_publisher<ar_track_alvar_msgs::msg::AlvarMarkers>(
        "ar_pose_marker", 0);
    rviz_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "visualization_marker", 0);

    cam_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", rclcpp::SensorDataQoS(),
        std::bind(&TrainMarkerBundle::camInfoCallback, this,
                  std::placeholders::_1));

    image_sub_ = image_transport::create_subscription(
        this, "camera_image",
        std::bind(&TrainMarkerBundle::imageCallback, this,
                  std::placeholders::_1),
        "raw", rmw_qos_profile_sensor_data);

    cv::namedWindow("Command input window", cv::WINDOW_AUTOSIZE);
    key_timer_ = create_wall_timer(std::chrono::milliseconds(20),
                                   std::bind(&TrainMarkerBundle::keyCallback,
                                             this));

    RCLCPP_INFO(get_logger(), "Training %d markers of size %.2f cm",
                nof_markers_, marker_size_);
    std::cout << std::endl;
    std::cout << "Keyboard Shortcuts:" << std::endl;
    std::cout << "  l: load marker configuration from mmarker.xml" << std::endl;
    std::cout << "  s: save marker configuration to mmarker.xml" << std::endl;
    std::cout << "  r: reset marker configuration" << std::endl;
    std::cout << "  p: add measurement" << std::endl;
    std::cout << "  a: auto add measurements (captures once a second for 5 "
                 "seconds)"
              << std::endl;
    std::cout << "  o: optimize bundle" << std::endl;
    std::cout << "  q: quit" << std::endl;
    std::cout << std::endl;
    std::cout << "Please type commands with the openCV window selected"
              << std::endl;
    std::cout << std::endl;
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

  void initMultiMarker(Pose& pose)
  {
    std::vector<int> id_vector;
    for (int i = 0; i < nof_markers_; ++i)
      id_vector.push_back(i);
    // Each marker needs to be visible in at least two images and at most 64
    // images are used.
    multi_marker_init_ = std::make_unique<MultiMarkerInitializer>(id_vector, 2,
                                                                  64);
    pose.Reset();
    multi_marker_init_->PointCloudAdd(id_vector[0], marker_size_, pose);
    multi_marker_bundle_ = std::make_unique<MultiMarkerBundle>(id_vector);
    marker_detector_.SetMarkerSize(marker_size_);
    initialized_ = true;
  }

  double getMultiMarkerPose(cv::Mat& image, Pose& pose)
  {
    if (!initialized_)
      initMultiMarker(pose);

    double error = -1;
    if (!optimize_done_)
    {
      if (marker_detector_.Detect(image, &cam_, true, false,
                                  max_new_marker_error_, max_track_error_,
                                  CVSEQ, true))
      {
        error = multi_marker_init_->Update(marker_detector_.markers, &cam_,
                                           pose);
      }
    }
    else
    {
      if (marker_detector_.Detect(image, &cam_, true, false,
                                  max_new_marker_error_, max_track_error_,
                                  CVSEQ, true))
      {
        error = multi_marker_bundle_->Update(marker_detector_.markers, &cam_,
                                             pose);
        if ((multi_marker_bundle_->SetTrackMarkers(marker_detector_, &cam_,
                                                   pose, image) > 0) &&
            (marker_detector_.DetectAdditional(image, &cam_, false) > 0))
        {
          error = multi_marker_bundle_->Update(marker_detector_.markers, &cam_,
                                               pose);
        }
      }
    }

    if (add_measurement_)
    {
      std::cout << "Markers seen: " << marker_detector_.markers->size() << "\n";
      if (marker_detector_.markers->size() >= 2)
      {
        std::cout << "Adding measurement..." << std::endl;
        multi_marker_init_->MeasurementsAdd(marker_detector_.markers);
      }
      else
      {
        std::cout << "Not enough markers to capture measurement\n";
      }
      add_measurement_ = false;
    }

    if (optimize_)
    {
      std::cout << "Initializing..." << std::endl;
      if (!multi_marker_init_->Initialize(&cam_))
      {
        std::cout << "Initialization failed, add some more measurements."
                  << std::endl;
      }
      else
      {
        // Reset the bundle adjuster.
        multi_marker_bundle_->Reset();
        multi_marker_bundle_->MeasurementsReset();
        // Copy all measurements into the bundle adjuster.
        for (int i = 0; i < multi_marker_init_->getMeasurementCount(); ++i)
        {
          Pose p2;
          multi_marker_init_->getMeasurementPose(i, &cam_, p2);
          const std::vector<MultiMarkerInitializer::MarkerMeasurement,
                            Eigen::aligned_allocator<
                                MultiMarkerInitializer::MarkerMeasurement> >
              markers = multi_marker_init_->getMeasurementMarkers(i);
          multi_marker_bundle_->MeasurementsAdd(&markers, p2);
        }
        // Initialize the bundle adjuster with initial marker poses.
        multi_marker_bundle_->PointCloudCopy(multi_marker_init_.get());
        std::cout << "Optimizing..." << std::endl;
        if (multi_marker_bundle_->Optimize(&cam_, 0.01, 20))
        {
          std::cout << "Optimizing done" << std::endl;
          optimize_done_ = true;
        }
        else
        {
          std::cout << "Optimizing FAILED!" << std::endl;
        }
      }
      optimize_ = false;
    }
    return error;
  }

  void makeMarkerMsgs(int type, int id, Pose& p,
                      const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
                      tf2::Transform& cam_to_output,
                      visualization_msgs::msg::Marker* rviz_marker,
                      ar_track_alvar_msgs::msg::AlvarMarker* ar_pose_marker)
  {
    double px = p.translation[0] / 100.0;
    double py = p.translation[1] / 100.0;
    double pz = p.translation[2] / 100.0;

    cv::Mat quat = cv::Mat(4, 1, CV_64F);
    p.GetQuaternion(quat);
    double qx = quat.at<double>(1, 0);
    double qy = quat.at<double>(2, 0);
    double qz = quat.at<double>(3, 0);
    double qw = quat.at<double>(0, 0);

    // Get the marker pose in the camera frame
    tf2::Quaternion rotation(qx, qy, qz, qw);
    tf2::Vector3 origin(px, py, pz);
    tf2::Transform t(rotation, origin);

    tf2::Transform marker_pose = t;

    // Publish the transform from the camera to the marker
    if (type == MAIN_MARKER)
    {
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
    }

    // Create the rviz visualization message
    tf2::toMsg(marker_pose, rviz_marker->pose);
    rviz_marker->header.frame_id = image_msg->header.frame_id;
    rviz_marker->header.stamp = image_msg->header.stamp;
    rviz_marker->id = id;

    rviz_marker->scale.x = 1.0 * marker_size_ / 100.0;
    rviz_marker->scale.y = 1.0 * marker_size_ / 100.0;
    rviz_marker->scale.z = 0.2 * marker_size_ / 100.0;
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
      rviz_marker->color.a = 1.0;
    }
    else if (type == GHOST_MARKER)
    {
      rviz_marker->color.r = 0.0f;
      rviz_marker->color.g = 0.0f;
      rviz_marker->color.b = 1.0f;
      rviz_marker->color.a = 0.5;
    }

    rviz_marker->lifetime = rclcpp::Duration(1, 0);

    // Get the pose of the tag in the camera frame, then convert to the output
    // frame (usually torso)
    tf2::Transform tag_pose_output = cam_to_output * marker_pose;

    // Create the pose marker message
    tf2::toMsg(tag_pose_output, ar_pose_marker->pose.pose);
    ar_pose_marker->header.frame_id = output_frame_;
    ar_pose_marker->header.stamp = image_msg->header.stamp;
    ar_pose_marker->id = id;
  }

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image_msg)
  {
    // Check if automatic measurement collection should be triggered
    if (auto_collect_)
    {
      auto_count_++;
      add_measurement_ = true;
      if (auto_count_ >= 5)
        auto_collect_ = false;
    }

    // If we've already gotten the cam info, then go ahead
    if (!cam_.getCamInfo_)
      return;

    try
    {
      // Get the transformation from the Camera to the output frame for this
      // image capture
      tf2::Transform cam_to_output = tf2::Transform::getIdentity();
      try
      {
        const auto transform = tf_buffer_.lookupTransform(
            image_msg->header.frame_id, output_frame_,
            image_msg->header.stamp, tf2::durationFromSec(1.0));
        tf2::fromMsg(transform.transform, cam_to_output);
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_ERROR(get_logger(), "%s", ex.what());
      }

      visualization_msgs::msg::Marker rvizMarker;
      ar_track_alvar_msgs::msg::AlvarMarker ar_pose_marker;
      ar_pose_markers_.markers.clear();

      // Convert the image
      cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

      // Get the estimated pose of the main markers by using all the markers in
      // each bundle
      double error = getMultiMarkerPose(cv_ptr_->image, bundle_pose_);
      (void)error;

      if (optimize_done_)
      {
        // Draw the main marker
        makeMarkerMsgs(MAIN_MARKER, 0, bundle_pose_, image_msg, cam_to_output,
                       &rvizMarker, &ar_pose_marker);
        rviz_marker_pub_->publish(rvizMarker);
        ar_pose_markers_.markers.push_back(ar_pose_marker);
      }
      // Now grab the poses of the other markers that are visible
      for (size_t i = 0; i < marker_detector_.markers->size(); i++)
      {
        int id = (*(marker_detector_.markers))[i].GetId();

        // Don't do the main marker (id=0) if we've already drawn it
        if (id > 0 || ((!optimize_done_) && id == 0))
        {
          Pose p = (*(marker_detector_.markers))[i].pose;
          makeMarkerMsgs(VISIBLE_MARKER, id, p, image_msg, cam_to_output,
                         &rvizMarker, &ar_pose_marker);
          rviz_marker_pub_->publish(rvizMarker);
          ar_pose_markers_.markers.push_back(ar_pose_marker);
        }
      }
      ar_pose_markers_.header.stamp = image_msg->header.stamp;
      ar_pose_markers_.header.frame_id = output_frame_;
      ar_marker_pub_->publish(ar_pose_markers_);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "Could not convert from '%s' to 'rgb8'.",
                   image_msg->encoding.c_str());
    }

    // Sleep if we are auto collecting
    if (auto_collect_)
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  // Do something based on keystrokes from menu
  void keyCallback()
  {
    int key = cv::waitKey(1);
    if (key < 0)
      return;

    if (key == 'r')
    {
      std::cout << "Reseting multi marker" << std::endl;
      multi_marker_init_->Reset();
      multi_marker_init_->MeasurementsReset();
      multi_marker_bundle_->Reset();
      multi_marker_bundle_->MeasurementsReset();
      add_measurement_ = false;
      optimize_ = false;
      optimize_done_ = false;
    }
    else if (key == 'l')
    {
      if (multi_marker_bundle_->Load("mmarker.xml", FILE_FORMAT_XML))
      {
        std::cout << "Multi marker loaded" << std::endl;
        multi_marker_init_->PointCloudCopy(multi_marker_bundle_.get());
        optimize_done_ = true;
      }
      else
        std::cout << "Cannot load multi marker" << std::endl;
    }
    else if (key == 's')
    {
      if (multi_marker_bundle_->Save("mmarker.xml", FILE_FORMAT_XML))
        std::cout << "Multi marker saved" << std::endl;
      else
        std::cout << "Cannot save multi marker" << std::endl;
    }
    else if (key == 'p')
    {
      add_measurement_ = true;
    }
    else if (key == 'a')
    {
      auto_count_ = 0;
      auto_collect_ = true;
    }
    else if (key == 'o')
    {
      optimize_ = true;
    }
    else if (key == 'q')
    {
      rclcpp::shutdown();
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
  rclcpp::Publisher<ar_track_alvar_msgs::msg::AlvarMarkers>::SharedPtr
      ar_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_marker_pub_;
  rclcpp::TimerBase::SharedPtr key_timer_;

  std::unique_ptr<MultiMarkerInitializer> multi_marker_init_;
  std::unique_ptr<MultiMarkerBundle> multi_marker_bundle_;
  Pose bundle_pose_;
  ar_track_alvar_msgs::msg::AlvarMarkers ar_pose_markers_;

  int nof_markers_;
  double marker_size_;
  double max_new_marker_error_;
  double max_track_error_;
  std::string output_frame_;

  bool initialized_ = false;
  bool add_measurement_ = false;
  bool optimize_ = false;
  bool optimize_done_ = false;
  bool auto_collect_ = false;
  int auto_count_ = 0;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  try
  {
    auto node = std::make_shared<TrainMarkerBundle>();
    rclcpp::spin(node);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("train_marker_bundle"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
