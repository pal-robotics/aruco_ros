// NOLINT(legal/copyright)

#ifndef ARUCO_ROS__ARUCO_ROS_UTILS_HPP_
#define ARUCO_ROS__ARUCO_ROS_UTILS_HPP_

#include <vector>

#include "aruco/aruco.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "visualization_msgs/msg/marker.hpp"

namespace aruco_ros
{
/**
 * @brief rosCameraInfo2ArucoCamParams gets the camera intrinsics from a CameraInfo message and copies them
 *                                     to aruco_ros own data structure
 * @param cam_info
 * @param useRectifiedParameters if true, the intrinsics are taken from cam_info.P and the distortion parameters
 *                               are set to 0. Otherwise, cam_info.K and cam_info.D are taken.
 * @return
 */
aruco::CameraParameters rosCameraInfo2ArucoCamParams(
  const sensor_msgs::msg::CameraInfo & cam_info,
  bool useRectifiedParameters);

tf2::Transform arucoMarker2Tf2(const aruco::Marker & marker);

std::vector<aruco::Marker> detectMarkers(
  const cv::Mat & img,
  const aruco::CameraParameters & cam_params,
  float marker_size,
  aruco::MarkerDetector * detector = nullptr,
  bool normalize_ilumination = false, bool correct_fisheye = false);

visualization_msgs::msg::Marker visMarkerFromPose(
  const geometry_msgs::msg::PoseStamped & pose,
  double marker_size, int marker_id = 1);

}  // namespace aruco_ros

#endif  // ARUCO_ROS__ARUCO_ROS_UTILS_HPP_
