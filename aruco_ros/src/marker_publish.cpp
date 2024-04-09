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
 * @file marker_publish.cpp
 * @author Bence Magyar
 * @date June 2014
 * @brief Modified copy of simple_single.cpp to publish all markers visible
 * (modified by Josh Langsfeld, 2014)
 */

#include <iostream>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rcpputils/asserts.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include "aruco_msgs/msg/marker_array.hpp"
#include "aruco_ros/aruco_ros_utils.hpp"

using namespace std::chrono_literals;

namespace aruco_ros {
class ArUcoMarkerPublisher {
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr sub_node_;
  // ArUco stuff
  aruco::MarkerDetector detector_;
  aruco::CameraParameters cam_param_;
  std::vector<aruco::Marker> markers_;

  // node params
  bool use_rectified_images_;
  std::string marker_frame_;
  std::string camera_frame_;
  std::string reference_frame_;
  double marker_size_;

  // ROS pub-sub
  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;

  image_transport::Publisher image_pub_;
  image_transport::Publisher debug_pub_;
  rclcpp::Publisher<aruco_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr marker_list_pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<aruco_msgs::msg::MarkerArray> marker_msg_;
  cv::Mat in_image_;
  bool use_cam_info_;
  bool cam_info_received_;
  std_msgs::msg::UInt32MultiArray marker_list_msg_;

public:
  ArUcoMarkerPublisher(const rclcpp::NodeOptions &options)
      : node_(rclcpp::Node::make_shared("marker_publisher", options)), use_cam_info_(true),
        cam_info_received_(false) {
    setup();
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
    return node_->get_node_base_interface();
  }

  bool setup() {
    RCLCPP_INFO(node_->get_logger(), "*************************");
    RCLCPP_INFO(node_->get_logger(), "   ArUcoMarkerPublisher  ");
    RCLCPP_INFO(node_->get_logger(), "*************************");

    sub_node_ = node_->create_sub_node(node_->get_name());
    // Declare node parameters
    node_->declare_parameter<double>("marker_size", 0.05);
    node_->declare_parameter<std::string>("reference_frame", "");
    node_->declare_parameter<std::string>("camera_frame", "");
    node_->declare_parameter<bool>("image_is_rectified", true);
    node_->declare_parameter<bool>("use_camera_info", true);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    it_ = std::make_unique<image_transport::ImageTransport>(node_);
    image_sub_ = it_->subscribe("/image", 1, &ArUcoMarkerPublisher::on_image, this);

    node_->get_parameter_or<bool>("use_camera_info", use_cam_info_, true);
    node_->get_parameter_or<double>("marker_size", marker_size_, 0.05);
    if (use_cam_info_) {
      cam_info_sub_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
          "/camera_info", 1,
          std::bind(&ArUcoMarkerPublisher::on_cam_info, this, std::placeholders::_1));
      node_->get_parameter_or<bool>("image_is_rectified", use_rectified_images_, true);
      node_->get_parameter_or<std::string>("reference_frame", reference_frame_, "");
      node_->get_parameter_or<std::string>("camera_frame", camera_frame_, "");
      rcpputils::assert_true(
          !(camera_frame_.empty() && !reference_frame_.empty()),
          "Either the camera frame is empty and also reference frame is empty..");
      if (reference_frame_.empty()) {
        reference_frame_ = camera_frame_;
      }
    } else {
      cam_param_ = aruco::CameraParameters();
    }

    image_pub_ = it_->advertise(node_->get_name() + std::string("/result"), 1);
    debug_pub_ = it_->advertise(node_->get_name() + std::string("/debug"), 1);
    marker_pub_ = sub_node_->create_publisher<aruco_msgs::msg::MarkerArray>("markers", 100);
    marker_list_pub_ =
        sub_node_->create_publisher<std_msgs::msg::UInt32MultiArray>("markers_list", 10);

    marker_msg_ = aruco_msgs::msg::MarkerArray::Ptr(new aruco_msgs::msg::MarkerArray());
    marker_msg_->header.frame_id = reference_frame_;

    log_parameters();
    RCLCPP_INFO(node_->get_logger(), "Successfully setup the marker publisher!");

    return true;
  }

  void log_parameters() {
    RCLCPP_INFO(node_->get_logger(), " * Marker size: %f", marker_size_);
    RCLCPP_INFO(node_->get_logger(), " * Reference frame: %s", reference_frame_.c_str());
    RCLCPP_INFO(node_->get_logger(), " * Camera frame: %s", camera_frame_.c_str());
    RCLCPP_INFO(node_->get_logger(), " * Image is rectified: %s",
                use_rectified_images_ ? "true" : "false");
    RCLCPP_INFO(node_->get_logger(), " * Use camera info: %s", use_cam_info_ ? "true" : "false");
  }

  bool get_transform(const std::string &refFrame, const std::string &childFrame,
                     geometry_msgs::msg::TransformStamped &transform) {
    std::string errMsg;

    if (!tf_buffer_->canTransform(refFrame, childFrame, tf2::TimePointZero,
                                  tf2::durationFromSec(0.5), &errMsg)) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Unable to get pose from TF : " << errMsg.c_str());
      return false;
    } else {
      try {
        transform = tf_buffer_->lookupTransform(refFrame, childFrame, tf2::TimePointZero,
                                                tf2::durationFromSec(0.5));
      } catch (const tf2::TransformException &e) {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Error in lookupTransform of "
                                                     << childFrame << " in " << refFrame << " : "
                                                     << e.what());
        return false;
      }
    }
    return true;
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    bool publishMarkers = marker_pub_->get_subscription_count() > 0;
    bool publishMarkersList = marker_list_pub_->get_subscription_count() > 0;
    bool publishImage = image_pub_.getNumSubscribers() > 0;
    bool publishDebug = debug_pub_.getNumSubscribers() > 0;

    if (!publishMarkers && !publishMarkersList && !publishImage && !publishDebug) {
      return;
    }

    builtin_interfaces::msg::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(*msg.get(), sensor_msgs::image_encodings::RGB8);
      in_image_ = cv_ptr->image;

      // clear out previous detection results
      markers_.clear();

      // ok, let's detect
      detector_.detect(in_image_, markers_, cam_param_, marker_size_, false);

      // marker array publish
      if (publishMarkers) {
        marker_msg_->markers.clear();
        marker_msg_->markers.resize(markers_.size());
        marker_msg_->header.stamp = curr_stamp;

        for (std::size_t i = 0; i < markers_.size(); ++i) {
          aruco_msgs::msg::Marker &marker_i = marker_msg_->markers.at(i);
          marker_i.header.stamp = curr_stamp;
          marker_i.id = markers_.at(i).id;
          marker_i.confidence = 1.0;
        }

        // if there is camera info let's do 3D stuff
        if (use_cam_info_ && cam_info_received_) {
          // get the current transform from the camera frame to output ref frame
          tf2::Stamped<tf2::Transform> cameraToReference;
          cameraToReference.setIdentity();

          if (reference_frame_ != camera_frame_) {
            geometry_msgs::msg::TransformStamped transform;
            get_transform(reference_frame_, camera_frame_, transform);
            tf2::fromMsg(transform, cameraToReference);
          }

          // now find the transform for each detected marker
          for (std::size_t i = 0; i < markers_.size(); ++i) {
            aruco_msgs::msg::Marker &marker_i = marker_msg_->markers.at(i);
            tf2::Transform transform = aruco_ros::arucoMarker2Tf2(markers_[i]);
            transform = static_cast<tf2::Transform>(cameraToReference) * transform;
            tf2::toMsg(transform, marker_i.pose.pose);
            marker_i.header.frame_id = reference_frame_;
          }
        }

        // publish marker array
        if (marker_msg_->markers.size() > 0) {
          marker_pub_->publish(*marker_msg_);
        }
      }

      if (publishMarkersList) {
        marker_list_msg_.data.resize(markers_.size());
        for (std::size_t i = 0; i < markers_.size(); ++i) {
          marker_list_msg_.data[i] = markers_[i].id;
        }

        marker_list_pub_->publish(marker_list_msg_);
      }

      // draw detected markers on the image for visualization
      for (std::size_t i = 0; i < markers_.size(); ++i) {
        markers_[i].draw(in_image_, cv::Scalar(0, 0, 255), 2);
      }

      // draw a 3D cube in each marker if there is 3D info
      if (cam_param_.isValid() && marker_size_ > 0) {
        for (std::size_t i = 0; i < markers_.size(); ++i) {
          aruco::CvDrawingUtils::draw3dAxis(in_image_, markers_[i], cam_param_);
        }
      }

      // publish input image with markers drawn on it
      if (publishImage) {
        // show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = in_image_;
        image_pub_.publish(out_msg.toImageMsg());
      }

      // publish image after internal image processing
      if (publishDebug) {
        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = detector_.getThresholdedImage();
        debug_pub_.publish(debug_msg.toImageMsg());
      }
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  // wait for one camerainfo, then shut down that subscriber
  void on_cam_info(const sensor_msgs::msg::CameraInfo &msg) {
    if (!cam_info_received_) {
      cam_param_ = aruco_ros::rosCameraInfo2ArucoCamParams(msg, use_rectified_images_);
      cam_info_received_ = true;
    }
  }
};
} // namespace aruco_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(aruco_ros::ArUcoMarkerPublisher)
