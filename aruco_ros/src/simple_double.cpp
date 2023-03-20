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
 * @file simple_double.cpp
 * @author Bence Magyar
 * @date June 2012
 * @version 0.1
 * @brief ROS version of the example named "simple" in the ArUco software package.
 */

#include <iostream>

#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include "aruco_ros/aruco_ros_utils.hpp"

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/utility.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/asserts.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

rclcpp::Node::SharedPtr node = nullptr;
rclcpp::Node::SharedPtr subNode = nullptr;
cv::Mat inImage;
aruco::CameraParameters camParam;
bool useRectifiedImages, normalizeImageIllumination;
int dctComponentsToRemove;
aruco::MarkerDetector mDetector;
std::vector<aruco::Marker> markers;
rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub;
bool cam_info_received;
image_transport::Publisher image_pub;
image_transport::Publisher debug_pub;
rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub1;
rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub2;
std::string child_name1;  // NOLINT(runtime/string)
std::string parent_name;  // NOLINT(runtime/string)
std::string child_name2;  // NOLINT(runtime/string)

double marker_size;
int marker_id1;
int marker_id2;
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  double ticksBefore = cv::getTickCount();
  if (cam_info_received) {
    builtin_interfaces::msg::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      inImage = cv_ptr->image;

      if (normalizeImageIllumination) {
        RCLCPP_WARN(node->get_logger(), "normalizeImageIllumination is unimplemented!");
//        cv::Mat inImageNorm;
//        pal_vision_util::dctNormalization(inImage, inImageNorm, dctComponentsToRemove);
//        inImage = inImageNorm;
      }

      // detection results will go into "markers"
      markers.clear();
      // ok, let's detect
      mDetector.detect(inImage, markers, camParam, marker_size, false);
      // for each marker, draw info and its boundaries in the image
      for (unsigned int i = 0; i < markers.size(); ++i) {
        // only publishing the selected marker
        if (markers[i].id == marker_id1) {
          tf2::Transform transform = aruco_ros::arucoMarker2Tf2(markers[i]);
          geometry_msgs::msg::TransformStamped m1_transform;
          m1_transform.header.frame_id = parent_name;
          m1_transform.header.stamp = curr_stamp;
          m1_transform.child_frame_id = child_name1;
          tf2::toMsg(transform, m1_transform.transform);
          tf_broadcaster_->sendTransform(m1_transform);
          geometry_msgs::msg::Pose poseMsg;
          tf2::toMsg(transform, poseMsg);
          pose_pub1->publish(poseMsg);
        } else if (markers[i].id == marker_id2) {
          tf2::Transform transform = aruco_ros::arucoMarker2Tf2(markers[i]);
          geometry_msgs::msg::TransformStamped m2_transform;
          m2_transform.header.frame_id = parent_name;
          m2_transform.header.stamp = curr_stamp;
          m2_transform.child_frame_id = child_name2;
          tf2::toMsg(transform, m2_transform.transform);
          tf_broadcaster_->sendTransform(m2_transform);
          geometry_msgs::msg::Pose poseMsg;
          tf2::toMsg(transform, poseMsg);
          pose_pub2->publish(poseMsg);
        }

        // but drawing all the detected markers
        markers[i].draw(inImage, cv::Scalar(0, 0, 255), 2);
      }

      // paint a circle in the center of the image
      cv::circle(
        inImage, cv::Point(inImage.cols / 2, inImage.rows / 2), 4, cv::Scalar(0, 255, 0),
        1);

      if (markers.size() == 2) {
        float x[2], y[2], u[2], v[2];
        for (unsigned int i = 0; i < 2; ++i) {
          RCLCPP_DEBUG_STREAM(
            node->get_logger(),
            "Marker(" << i << ") at camera coordinates = (" << markers[i].Tvec.at<float>(
              0,
              0) << ", " <<
              markers[i].Tvec.at<float>(1, 0) << ", " << markers[i].Tvec.at<float>(2, 0));
          // normalized coordinates of the marker
          x[i] = markers[i].Tvec.at<float>(0, 0) / markers[i].Tvec.at<float>(2, 0);
          y[i] = markers[i].Tvec.at<float>(1, 0) / markers[i].Tvec.at<float>(2, 0);
          // undistorted pixel
          u[i] = x[i] *
            camParam.CameraMatrix.at<float>(0, 0) + camParam.CameraMatrix.at<float>(0, 2);
          v[i] = y[i] *
            camParam.CameraMatrix.at<float>(1, 1) + camParam.CameraMatrix.at<float>(1, 2);
        }

        RCLCPP_DEBUG_STREAM(
          node->get_logger(),
          "Mid point between the two markers in the image = (" << (x[0] + x[1]) / 2 << ", " <<
            (y[0] + y[1]) / 2 << ")");

//        // paint a circle in the mid point of the normalized coordinates of both markers
//        cv::circle(
//          inImage, cv::Point((u[0] + u[1]) / 2, (v[0] + v[1]) / 2), 3, cv::Scalar(
//            0, 0,
//            255),
//          cv::FILLED);

        // compute the midpoint in 3D:
        float midPoint3D[3];  // 3D point
        for (unsigned int i = 0; i < 3; ++i) {
          midPoint3D[i] = (markers[0].Tvec.at<float>(i, 0) + markers[1].Tvec.at<float>(i, 0)) / 2;
        }
        // now project the 3D mid point to normalized coordinates
        float midPointNormalized[2];
        midPointNormalized[0] = midPoint3D[0] / midPoint3D[2];  // x
        midPointNormalized[1] = midPoint3D[1] / midPoint3D[2];  // y
        u[0] = midPointNormalized[0] *
          camParam.CameraMatrix.at<float>(0, 0) + camParam.CameraMatrix.at<float>(0, 2);
        v[0] = midPointNormalized[1] *
          camParam.CameraMatrix.at<float>(1, 1) + camParam.CameraMatrix.at<float>(1, 2);

        RCLCPP_DEBUG_STREAM(
          node->get_logger(),
          "3D Mid point between the two markers in undistorted pixel coordinates = (" <<
            u[0] << ", " << v[0] << ")");

        // paint a circle in the mid point of the normalized coordinates of both markers
        cv::circle(inImage, cv::Point(u[0], v[0]), 3, cv::Scalar(0, 0, 255), cv::FILLED);
      }

      // draw a 3D cube in each marker if there is 3D info
      if (camParam.isValid() && marker_size > 0) {
        for (unsigned int i = 0; i < markers.size(); ++i) {
          aruco::CvDrawingUtils::draw3dCube(inImage, markers[i], camParam);
        }
      }

      if (image_pub.getNumSubscribers() > 0) {
        // show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage;
        image_pub.publish(out_msg.toImageMsg());
      }

      if (debug_pub.getNumSubscribers() > 0) {
        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector.getThresholdedImage();
        debug_pub.publish(debug_msg.toImageMsg());
      }

      RCLCPP_DEBUG(
        node->get_logger(), "runtime: %f ms",
        1000 * (cv::getTickCount() - ticksBefore) / cv::getTickFrequency());
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }
}

// wait for one camerainfo and then don't update the info
void cam_info_callback(const sensor_msgs::msg::CameraInfo & msg)
{
  if (!cam_info_received) {
    camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);
    cam_info_received = true;
  }
}

// void reconf_callback(aruco_ros::ArucoThresholdConfig &config, std::uint32_t level)
// {
//   mDetector.setDetectionMode(aruco::DetectionMode(config.detection_mode), config.min_image_size);
//   normalizeImageIllumination = config.normalizeImage;
//   dctComponentsToRemove = config.dctComponentsToRemove;
// }

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  node = std::make_shared<rclcpp::Node>("aruco_double");
  subNode = node->create_sub_node(node->get_name());

  // Declare node parameters
  node->declare_parameter<bool>("image_is_rectified", true);
  node->declare_parameter<double>("marker_size", 0.05);
  node->declare_parameter<int>("marker_id1", 582);
  node->declare_parameter<int>("marker_id2", 26);
  node->declare_parameter<bool>("normalizeImage", true);
  node->declare_parameter<int>("dct_components_to_remove", 2);
  node->declare_parameter<std::string>("parent_name", "");
  node->declare_parameter<std::string>("child_name1", "");
  node->declare_parameter<std::string>("child_name2", "");

  image_transport::ImageTransport it(node);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node.get());

  // dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> server;
  // dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig>::CallbackType f_;
  // f_ = boost::bind(&reconf_callback, _1, _2);
  // server.setCallback(f_);

  normalizeImageIllumination = false;

  node->get_parameter_or<bool>("image_is_rectified", useRectifiedImages, true);
  RCLCPP_INFO_STREAM(node->get_logger(), "Image is rectified: " << useRectifiedImages);

  image_transport::Subscriber image_sub = it.subscribe("/image", 1, &image_callback);
  cam_info_sub = node->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", 1,
    cam_info_callback);

  cam_info_received = false;
  image_pub = it.advertise(node->get_name() + std::string("/result"), 1);
  debug_pub = it.advertise(node->get_name() + std::string("/debug"), 1);
  pose_pub1 = subNode->create_publisher<geometry_msgs::msg::Pose>("pose", 100);
  pose_pub2 = subNode->create_publisher<geometry_msgs::msg::Pose>("pose2", 100);

  node->get_parameter_or<double>("marker_size", marker_size, 0.05);
  node->get_parameter_or<int>("marker_id1", marker_id1, 582);
  node->get_parameter_or<int>("marker_id2", marker_id2, 26);
  node->get_parameter_or<bool>("normalizeImage", normalizeImageIllumination, true);
  node->get_parameter_or<int>("dct_components_to_remove", dctComponentsToRemove, 2);
  if (dctComponentsToRemove == 0) {
    normalizeImageIllumination = false;
  }

  node->get_parameter_or<std::string>("parent_name", parent_name, "");
  node->get_parameter_or<std::string>("child_name1", child_name1, "");
  node->get_parameter_or<std::string>("child_name2", child_name2, "");

  if (parent_name == "" || child_name1 == "" || child_name2 == "") {
    RCLCPP_ERROR(node->get_logger(), "parent_name and/or child_name was not set!");
    rclcpp::shutdown();
    return -1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "ArUco node started with marker size of %f meters and marker ids to track: %d, %d",
    marker_size, marker_id1, marker_id2);
  RCLCPP_INFO(
    node->get_logger(),
    "ArUco node will publish pose to TF with (%s, %s) and (%s, %s) as (parent,child).",
    parent_name.c_str(), child_name1.c_str(), parent_name.c_str(), child_name2.c_str());

  rclcpp::spin(node);
  rclcpp::shutdown();
}
