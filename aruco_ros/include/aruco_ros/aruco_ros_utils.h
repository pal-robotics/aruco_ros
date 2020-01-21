#ifndef ARUCO_ROS_UTILS_H
#define ARUCO_ROS_UTILS_H

#include <aruco/aruco.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

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
aruco::CameraParameters rosCameraInfo2ArucoCamParams(const sensor_msgs::CameraInfo& cam_info,
                                                     bool useRectifiedParameters);

tf::Transform arucoMarker2Tf(const aruco::Marker& marker);
tf2::Transform arucoMarker2Tf2(const aruco::Marker& marker);

std::vector<aruco::Marker> detectMarkers(const cv::Mat& img,
                                         const aruco::CameraParameters& cam_params,
                                         float marker_size,
                                         aruco::MarkerDetector* detector = nullptr,
                                         bool normalize_ilumination = false, bool correct_fisheye = false);

visualization_msgs::Marker visMarkerFromPose(const geometry_msgs::PoseStamped& pose,
                                             double marker_size, int marker_id = 1);

}
#endif // ARUCO_ROS_UTILS_H
