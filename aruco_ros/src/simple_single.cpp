/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/
/**
 * @file simple_single.cpp
 * @author Bence Magyar
 * @date June 2012
 * @version 0.1
 * @brief ROS version of the example named "simple" in the ArUco software package.
 */

#include <iostream>

#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include "aruco_ros/aruco_ros_utils.hpp"

#if __has_include("cv_bridge/cv_bridge.hpp")
#include "cv_bridge/cv_bridge.hpp"
#else
#include "cv_bridge/cv_bridge.h"
#endif
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/asserts.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

class ArucoSimple : public rclcpp::Node
{
private:
  rclcpp::Node::SharedPtr subNode_;
  cv::Mat inImage_;
  aruco::CameraParameters camParam_;
  tf2::Stamped<tf2::Transform> rightToLeft_;
  bool useRectifiedImages_;
  aruco::MarkerDetector mDetector_;
  std::vector<aruco::Marker> markers_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  bool cam_info_received_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher debug_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr position_pub_;
  // rviz visualization marker
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pixel_pub_;
  std::string marker_frame_;
  std::string camera_frame_;
  std::string reference_frame_;

  double marker_size_;
  int marker_id_;

  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber image_sub_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:
  ArucoSimple()
  : Node("aruco_single"), cam_info_received_(false)
  {
  }

  bool setup()
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    subNode_ = this->create_sub_node(this->get_name());

    it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    if (this->has_parameter("corner_refinement")) {
      RCLCPP_WARN(
        this->get_logger(),
        "Corner refinement options have been removed in ArUco 3.0.0, "
        "corner_refinement ROS parameter is deprecated");
    }

    aruco::MarkerDetector::Params params = mDetector_.getParameters();
    std::string thresh_method;
    switch (params.thresMethod) {
      case aruco::MarkerDetector::ThresMethod::THRES_ADAPTIVE:
        thresh_method = "THRESH_ADAPTIVE";
        break;
      case aruco::MarkerDetector::ThresMethod::THRES_AUTO_FIXED:
        thresh_method = "THRESH_AUTO_FIXED";
        break;
      default:
        thresh_method = "UNKNOWN";
        break;
    }

    // Print parameters of ArUco marker detector:
    RCLCPP_INFO_STREAM(this->get_logger(), "Threshold method: " << thresh_method);

    // Declare node parameters
    this->declare_parameter<double>("marker_size", 0.05);
    this->declare_parameter<int>("marker_id", 300);
    this->declare_parameter<std::string>("reference_frame", "");
    this->declare_parameter<std::string>("camera_frame", "");
    this->declare_parameter<std::string>("marker_frame", "");
    this->declare_parameter<bool>("image_is_rectified", true);
    this->declare_parameter<float>("min_marker_size", 0.02);
    this->declare_parameter<std::string>("detection_mode", "");

    float min_marker_size;  // percentage of image area
    this->get_parameter_or<float>("min_marker_size", min_marker_size, 0.02);

    std::string detection_mode;
    this->get_parameter_or<std::string>("detection_mode", detection_mode, "DM_FAST");
    if (detection_mode == "DM_FAST") {
      mDetector_.setDetectionMode(aruco::DM_FAST, min_marker_size);
    } else if (detection_mode == "DM_VIDEO_FAST") {
      mDetector_.setDetectionMode(aruco::DM_VIDEO_FAST, min_marker_size);
    } else {
      // Aruco version 2 mode
      mDetector_.setDetectionMode(aruco::DM_NORMAL, min_marker_size);
    }

    RCLCPP_INFO_STREAM(
      this->get_logger(), "Marker size min: " << min_marker_size << " of image area");
    RCLCPP_INFO_STREAM(this->get_logger(), "Detection mode: " << detection_mode);

    image_sub_ = it_->subscribe("/image", 1, &ArucoSimple::image_callback, this);
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", 1, std::bind(
        &ArucoSimple::cam_info_callback, this,
        std::placeholders::_1));

    image_pub_ = it_->advertise(this->get_name() + std::string("/result"), 1);
    debug_pub_ = it_->advertise(this->get_name() + std::string("/debug"), 1);
    pose_pub_ = subNode_->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 100);
    transform_pub_ =
      subNode_->create_publisher<geometry_msgs::msg::TransformStamped>("transform", 100);
    position_pub_ = subNode_->create_publisher<geometry_msgs::msg::Vector3Stamped>("position", 100);
    marker_pub_ = subNode_->create_publisher<visualization_msgs::msg::Marker>("marker", 10);
    pixel_pub_ = subNode_->create_publisher<geometry_msgs::msg::PointStamped>("pixel", 10);

    this->get_parameter_or<double>("marker_size", marker_size_, 0.05);
    this->get_parameter_or<int>("marker_id", marker_id_, 300);
    this->get_parameter_or<std::string>("reference_frame", reference_frame_, "");
    this->get_parameter_or<std::string>("camera_frame", camera_frame_, "");
    this->get_parameter_or<std::string>("marker_frame", marker_frame_, "");
    this->get_parameter_or<bool>("image_is_rectified", useRectifiedImages_, true);

    rcpputils::assert_true(
      camera_frame_ != "" && marker_frame_ != "",
      "Found the camera frame or the marker_frame to be empty!. camera_frame : " +
      camera_frame_ + " and marker_frame : " + marker_frame_);

    if (reference_frame_.empty()) {
      reference_frame_ = camera_frame_;
    }

    RCLCPP_INFO(
      this->get_logger(), "ArUco node started with marker size of %f m and marker id to track: %d",
      marker_size_, marker_id_);
    RCLCPP_INFO(
      this->get_logger(), "ArUco node will publish pose to TF with %s as parent and %s as child.",
      reference_frame_.c_str(), marker_frame_.c_str());

    // dyn_rec_server.setCallback(boost::bind(&ArucoSimple::reconf_callback, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "Setup of aruco_simple node is successful!");
    return true;
  }

  bool getTransform(
    const std::string & refFrame, const std::string & childFrame,
    geometry_msgs::msg::TransformStamped & transform)
  {
    std::string errMsg;

    if (!tf_buffer_->canTransform(
        refFrame, childFrame, tf2::TimePointZero,
        tf2::durationFromSec(0.5), &errMsg))
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to get pose from TF: " << errMsg);
      return false;
    } else {
      try {
        transform = tf_buffer_->lookupTransform(
          refFrame, childFrame, tf2::TimePointZero, tf2::durationFromSec(
            0.5));
      } catch (const tf2::TransformException & e) {
        RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "Error in lookupTransform of " << childFrame << " in " << refFrame << " : " << e.what());
        return false;
      }
    }
    return true;
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    if ((image_pub_.getNumSubscribers() == 0) &&
      (debug_pub_.getNumSubscribers() == 0) &&
      (pose_pub_->get_subscription_count() == 0) &&
      (transform_pub_->get_subscription_count() == 0) &&
      (position_pub_->get_subscription_count() == 0) &&
      (marker_pub_->get_subscription_count() == 0) &&
      (pixel_pub_->get_subscription_count() == 0))
    {
      RCLCPP_DEBUG(this->get_logger(), "No subscribers, not looking for ArUco markers");
      return;
    }

    if (cam_info_received_) {
      builtin_interfaces::msg::Time curr_stamp = msg->header.stamp;
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::RGB8);
        inImage_ = cv_ptr->image;

        // detection results will go into "markers"
        markers_.clear();
        // ok, let's detect
        mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);
        // for each marker, draw info and its boundaries in the image
        for (std::size_t i = 0; i < markers_.size(); ++i) {
          // only publishing the selected marker
          if (markers_[i].id == marker_id_) {
            tf2::Transform transform = aruco_ros::arucoMarker2Tf2(markers_[i]);
            tf2::Stamped<tf2::Transform> cameraToReference;
            cameraToReference.setIdentity();

            if (reference_frame_ != camera_frame_) {
              geometry_msgs::msg::TransformStamped transform_stamped;
              getTransform(reference_frame_, camera_frame_, transform_stamped);
              tf2::fromMsg(transform_stamped, cameraToReference);
            }

            transform = static_cast<tf2::Transform>(cameraToReference) *
              static_cast<tf2::Transform>(rightToLeft_) *
              transform;

            geometry_msgs::msg::TransformStamped stampedTransform;
            stampedTransform.header.frame_id = reference_frame_;
            stampedTransform.header.stamp = curr_stamp;
            stampedTransform.child_frame_id = marker_frame_;
            tf2::toMsg(transform, stampedTransform.transform);
            tf_broadcaster_->sendTransform(stampedTransform);
            geometry_msgs::msg::PoseStamped poseMsg;
            poseMsg.header = stampedTransform.header;
            tf2::toMsg(transform, poseMsg.pose);
            poseMsg.header.frame_id = reference_frame_;
            poseMsg.header.stamp = curr_stamp;
            pose_pub_->publish(poseMsg);

            transform_pub_->publish(stampedTransform);

            geometry_msgs::msg::Vector3Stamped positionMsg;
            positionMsg.header = stampedTransform.header;
            positionMsg.vector = stampedTransform.transform.translation;
            position_pub_->publish(positionMsg);

            geometry_msgs::msg::PointStamped pixelMsg;
            pixelMsg.header = stampedTransform.header;
            pixelMsg.point.x = markers_[i].getCenter().x;
            pixelMsg.point.y = markers_[i].getCenter().y;
            pixelMsg.point.z = 0;
            pixel_pub_->publish(pixelMsg);

            // publish rviz marker representing the ArUco marker patch
            visualization_msgs::msg::Marker visMarker;
            visMarker.header = stampedTransform.header;
            visMarker.id = 1;
            visMarker.type = visualization_msgs::msg::Marker::CUBE;
            visMarker.action = visualization_msgs::msg::Marker::ADD;
            visMarker.pose = poseMsg.pose;
            visMarker.scale.x = marker_size_;
            visMarker.scale.y = marker_size_;
            visMarker.scale.z = 0.001;
            visMarker.color.r = 1.0;
            visMarker.color.g = 0;
            visMarker.color.b = 0;
            visMarker.color.a = 1.0;
            visMarker.lifetime = builtin_interfaces::msg::Duration();
            visMarker.lifetime.sec = 3;
            marker_pub_->publish(visMarker);
          }
          // but drawing all the detected markers
          markers_[i].draw(inImage_, cv::Scalar(0, 0, 255), 2);
        }

        // draw a 3d cube in each marker if there is 3d info
        if (camParam_.isValid() && marker_size_ > 0) {
          for (std::size_t i = 0; i < markers_.size(); ++i) {
            aruco::CvDrawingUtils::draw3dAxis(inImage_, markers_[i], camParam_);
          }
        }

        if (image_pub_.getNumSubscribers() > 0) {
          // show input with augmented information
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = curr_stamp;
          out_msg.encoding = sensor_msgs::image_encodings::RGB8;
          out_msg.image = inImage_;
          image_pub_.publish(out_msg.toImageMsg());
        }

        if (debug_pub_.getNumSubscribers() > 0) {
          // show also the internal image resulting from the threshold operation
          cv_bridge::CvImage debug_msg;
          debug_msg.header.stamp = curr_stamp;
          debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
          debug_msg.image = mDetector_.getThresholdedImage();
          debug_pub_.publish(debug_msg.toImageMsg());
        }
      } catch (cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }
    }
  }

  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::msg::CameraInfo & msg)
  {
    if (!cam_info_received_) {
      camParam_ = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages_);

      // handle cartesian offset between stereo pairs
      // see the sensor_msgs/CameraInfo documentation for details
      rightToLeft_.setIdentity();
      rightToLeft_.setOrigin(tf2::Vector3(-msg.p[3] / msg.p[0], -msg.p[7] / msg.p[5], 0.0));

      cam_info_received_ = true;
    }
  }

//  void reconf_callback(aruco_ros::ArucoThresholdConfig & config, uint32_t level)
//  {
//    mDetector_.setDetectionMode(
//      aruco::DetectionMode(config.detection_mode),
//      config.min_image_size);
//    if (config.normalizeImage) {
//      RCLCPP_WARN("normalizeImageIllumination is unimplemented!");
//    }
//  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<ArucoSimple> aruco_simple = std::make_shared<ArucoSimple>();
  aruco_simple->setup();
  rclcpp::spin(aruco_simple);
  rclcpp::shutdown();
}
