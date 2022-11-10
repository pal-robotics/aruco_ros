// NOLINT(legal/copyright)

#include <iostream>

#include "aruco_ros/aruco_ros_utils.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv4/opencv2/calib3d.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/transform_datatypes.h"

aruco::CameraParameters aruco_ros::rosCameraInfo2ArucoCamParams(
  const sensor_msgs::msg::CameraInfo & cam_info,
  bool useRectifiedParameters)
{
  cv::Mat cameraMatrix(3, 4, CV_64FC1, 0.0);
  cv::Mat distorsionCoeff(4, 1, CV_64FC1);
  cv::Size size(cam_info.width, cam_info.height);

  if (useRectifiedParameters) {
    cameraMatrix.setTo(0);
    cameraMatrix.at<double>(0, 0) = cam_info.p[0];
    cameraMatrix.at<double>(0, 1) = cam_info.p[1];
    cameraMatrix.at<double>(0, 2) = cam_info.p[2];
    cameraMatrix.at<double>(0, 3) = cam_info.p[3];
    cameraMatrix.at<double>(1, 0) = cam_info.p[4];
    cameraMatrix.at<double>(1, 1) = cam_info.p[5];
    cameraMatrix.at<double>(1, 2) = cam_info.p[6];
    cameraMatrix.at<double>(1, 3) = cam_info.p[7];
    cameraMatrix.at<double>(2, 0) = cam_info.p[8];
    cameraMatrix.at<double>(2, 1) = cam_info.p[9];
    cameraMatrix.at<double>(2, 2) = cam_info.p[10];
    cameraMatrix.at<double>(2, 3) = cam_info.p[11];

    for (int i = 0; i < 4; ++i) {
      distorsionCoeff.at<double>(i, 0) = 0;
    }
  } else {
    cv::Mat cameraMatrixFromK(3, 3, CV_64FC1, 0.0);
    for (int i = 0; i < 9; ++i) {
      cameraMatrixFromK.at<double>(i % 3, i - (i % 3) * 3) = cam_info.k[i];
    }
    cameraMatrixFromK.copyTo(cameraMatrix(cv::Rect(0, 0, 3, 3)));


    if (cam_info.d.size() == 4) {
      for (int i = 0; i < 4; ++i) {
        distorsionCoeff.at<double>(i, 0) = cam_info.d[i];
      }
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger(
          "aruco_ros"), "length of camera_info D vector is not 4, assuming zero distortion...");
      for (int i = 0; i < 4; ++i) {
        distorsionCoeff.at<double>(i, 0) = 0;
      }
    }
  }

  return aruco::CameraParameters(cameraMatrix, distorsionCoeff, size);
}

tf2::Transform aruco_ros::arucoMarker2Tf2(const aruco::Marker & marker)
{
  cv::Mat rot(3, 3, CV_64FC1);
  cv::Mat Rvec64;
  marker.Rvec.convertTo(Rvec64, CV_64FC1);
  cv::Rodrigues(Rvec64, rot);
  cv::Mat tran64;
  marker.Tvec.convertTo(tran64, CV_64FC1);

  tf2::Matrix3x3 tf_rot(rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
    rot.at<double>(1, 0),
    rot.at<double>(1, 1), rot.at<double>(1, 2), rot.at<double>(2, 0), rot.at<double>(2, 1),
    rot.at<double>(2, 2));

  tf2::Vector3 tf_orig(tran64.at<double>(0, 0), tran64.at<double>(1, 0), tran64.at<double>(2, 0));

  return tf2::Transform(tf_rot, tf_orig);
}


std::vector<aruco::Marker> aruco_ros::detectMarkers(
  const cv::Mat & img,
  const aruco::CameraParameters & cam_params,
  float marker_size,
  aruco::MarkerDetector * detector,
  bool normalize_ilumination,
  bool correct_fisheye)
{
  std::vector<aruco::Marker> markers;
  try {
    if (normalize_ilumination) {
      RCLCPP_WARN(rclcpp::get_logger("aruco_ros"), "normalizeImageIllumination is unimplemented!");
      //        cv::Mat inImageNorm;
      //        pal_vision_util::dctNormalization(inImage, inImageNorm,
      //        dctComponentsToRemove); inImage = inImageNorm;
    }

    // detection results will go into "markers"
    markers.clear();
    // ok, let's detect
    if (detector) {
      detector->detect(img, markers, cam_params, marker_size, false, correct_fisheye);
    } else {
      aruco::MarkerDetector default_detector;
      default_detector.detect(img, markers, cam_params, marker_size, false, correct_fisheye);
    }
    return markers;
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("aruco_ros"), "cv_bridge exception: %s", e.what());

    return markers;
  }
}


visualization_msgs::msg::Marker aruco_ros::visMarkerFromPose(
  const geometry_msgs::msg::PoseStamped & pose, double marker_size, int marker_id)
{
  visualization_msgs::msg::Marker visMarker;
  visMarker.header = pose.header;
  visMarker.id = marker_id;
  visMarker.type = visualization_msgs::msg::Marker::CUBE;
  visMarker.action = visualization_msgs::msg::Marker::ADD;
  visMarker.pose = pose.pose;
  visMarker.scale.x = marker_size;
  visMarker.scale.y = marker_size;
  visMarker.scale.z = 0.001;
  visMarker.color.r = 1.0;
  visMarker.color.g = 0;
  visMarker.color.b = 0;
  visMarker.color.a = 1.0;
  visMarker.lifetime = builtin_interfaces::msg::Duration();
  visMarker.lifetime.sec = 3;
  return visMarker;
}
