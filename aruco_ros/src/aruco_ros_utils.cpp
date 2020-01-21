#include <aruco_ros/aruco_ros_utils.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

aruco::CameraParameters aruco_ros::rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo& cam_info,
                                                                bool useRectifiedParameters)
{
  cv::Mat cameraMatrix(3, 4, CV_64FC1);
  cv::Mat distorsionCoeff(4, 1, CV_64FC1);
  cv::Size size(cam_info.width, cam_info.height);

  if (useRectifiedParameters)
  {
    cameraMatrix.setTo(0);
    cameraMatrix.at<double>(0, 0) = cam_info.P[0];
    cameraMatrix.at<double>(0, 1) = cam_info.P[1];
    cameraMatrix.at<double>(0, 2) = cam_info.P[2];
    cameraMatrix.at<double>(0, 3) = cam_info.P[3];
    cameraMatrix.at<double>(1, 0) = cam_info.P[4];
    cameraMatrix.at<double>(1, 1) = cam_info.P[5];
    cameraMatrix.at<double>(1, 2) = cam_info.P[6];
    cameraMatrix.at<double>(1, 3) = cam_info.P[7];
    cameraMatrix.at<double>(2, 0) = cam_info.P[8];
    cameraMatrix.at<double>(2, 1) = cam_info.P[9];
    cameraMatrix.at<double>(2, 2) = cam_info.P[10];
    cameraMatrix.at<double>(2, 3) = cam_info.P[11];

    for (int i = 0; i < 4; ++i)
      distorsionCoeff.at<double>(i, 0) = 0;
  }
  else
  {
    for (int i = 0; i < 9; ++i)
      cameraMatrix.at<double>(i % 3, i - (i % 3) * 3) = cam_info.K[i];

    if (cam_info.D.size() == 4)
    {
      for (int i = 0; i < 4; ++i)
        distorsionCoeff.at<double>(i, 0) = cam_info.D[i];
    }
    else
    {
      ROS_WARN("length of camera_info D vector is not 4, assuming zero distortion...");
      for (int i = 0; i < 4; ++i)
        distorsionCoeff.at<double>(i, 0) = 0;
    }
  }

  return aruco::CameraParameters(cameraMatrix, distorsionCoeff, size);
}

tf::Transform aruco_ros::arucoMarker2Tf(const aruco::Marker &marker)
{
  tf2::Transform tf2_tf = arucoMarker2Tf2(marker);
  return tf::Transform(
        tf::Matrix3x3(tf::Quaternion(tf2_tf.getRotation().x(),
                                     tf2_tf.getRotation().y(),
                                     tf2_tf.getRotation().z(),
                                     tf2_tf.getRotation().w())),
                      tf::Vector3(tf2_tf.getOrigin().x(),
                                  tf2_tf.getOrigin().y(),
                                  tf2_tf.getOrigin().z()));
}
tf2::Transform aruco_ros::arucoMarker2Tf2(const aruco::Marker &marker)
{
  cv::Mat rot(3, 3, CV_64FC1);
  cv::Mat Rvec64;
  marker.Rvec.convertTo(Rvec64, CV_64FC1);
  cv::Rodrigues(Rvec64, rot);
  cv::Mat tran64;
  marker.Tvec.convertTo(tran64, CV_64FC1);

  tf2::Matrix3x3 tf_rot(rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2), rot.at<double>(1, 0),
                       rot.at<double>(1, 1), rot.at<double>(1, 2), rot.at<double>(2, 0), rot.at<double>(2, 1),
                       rot.at<double>(2, 2));

  tf2::Vector3 tf_orig(tran64.at<double>(0, 0), tran64.at<double>(1, 0), tran64.at<double>(2, 0));

  return tf2::Transform(tf_rot, tf_orig);
}


std::vector<aruco::Marker> aruco_ros::detectMarkers(const cv::Mat &img, const aruco::CameraParameters &cam_params, float marker_size, aruco::MarkerDetector *detector, bool normalize_ilumination, bool correct_fisheye)
{
  std::vector<aruco::Marker> markers;
  try
  {
    if (normalize_ilumination)
    {
      ROS_WARN("normalizeImageIllumination is unimplemented!");
      //        cv::Mat inImageNorm;
      //        pal_vision_util::dctNormalization(inImage, inImageNorm,
      //        dctComponentsToRemove); inImage = inImageNorm;
    }
    
    // detection results will go into "markers"
    markers.clear();
    // ok, let's detect 
    if (detector)
    {
      detector->detect(img, markers, cam_params, marker_size, false, correct_fisheye);
    }
    else
    {
      aruco::MarkerDetector default_detector;
      default_detector.detect(img, markers, cam_params, marker_size, false, correct_fisheye);
    }
    return markers;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return markers;
  }
}


visualization_msgs::Marker aruco_ros::visMarkerFromPose(const geometry_msgs::PoseStamped &pose, double marker_size, int marker_id)
{
  visualization_msgs::Marker visMarker;
  visMarker.header = pose.header;
  visMarker.id = marker_id;
  visMarker.type = visualization_msgs::Marker::CUBE;
  visMarker.action = visualization_msgs::Marker::ADD;
  visMarker.pose = pose.pose;
  visMarker.scale.x = marker_size;
  visMarker.scale.y = marker_size;
  visMarker.scale.z = 0.001;
  visMarker.color.r = 1.0;
  visMarker.color.g = 0;
  visMarker.color.b = 0;
  visMarker.color.a = 1.0;
  visMarker.lifetime = ros::Duration(3.0);
  return visMarker;
}
