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
* @file multi_board.cpp
* @author Sarthak Mittal
* @date April 2020
* @version 0.2.4
* @brief ROS version of the example named "simple" in the Aruco software package.
*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <aruco/aruco.h>
#include <aruco/boarddetector.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>

using namespace aruco;

struct board_t {
  int uid;
  std::string frame_id;
  BoardConfiguration config;
};


class ArucoMultiBoard {

 private:

  cv::Mat inImage;
  aruco::CameraParameters camParam;
  tf::StampedTransform rightToLeft;
  bool useRectifiedImages;
  bool draw_markers;
  bool rotate_marker_axis;
  MarkerDetector mDetector;
  vector<Marker> markers;
  BoardDetector board_detector;
  vector<board_t> boards;
  vector<Board> boards_detected;
  bool cam_info_received;
  ros::Subscriber cam_info_sub;
  image_transport::Publisher image_pub;
  image_transport::Publisher debug_pub;
  ros::Publisher pose_pub;
  ros::Publisher transform_pub;
  ros::Publisher position_pub;
  ros::Publisher marker_pub; //rviz visualization marker
  std::string reference_frame;
  std::string camera_frame;

  double marker_size;
  std::string board_config_file;
  std::string board_config_dir;

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;

  tf::TransformListener _tfListener;

  dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> dyn_rec_server;

 public:
  ArucoMultiBoard()
      : cam_info_received(false),
        nh("~"),
        it(nh) {

    std::string refinementMethod;
    nh.param<std::string>("corner_refinement", refinementMethod, "LINES");
    if (refinementMethod == "SUBPIX")
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::SUBPIX);
    else if (refinementMethod == "HARRIS")
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::HARRIS);
    else if (refinementMethod == "NONE")
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::NONE);
    else
      mDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);

    //Print parameters of aruco marker detector:
    ROS_INFO_STREAM("Corner refinement method: " << mDetector.getCornerRefinementMethod());
    ROS_INFO_STREAM("Threshold method: " << mDetector.getThresholdMethod());
    double th1, th2;
    mDetector.getThresholdParams(th1, th2);
    ROS_INFO_STREAM("Threshold method: " << " th1: " << th1 << " th2: " << th2);
    float mins, maxs;
    mDetector.getMinMaxSize(mins, maxs);
    ROS_INFO_STREAM("Marker size min: " << mins << "  max: " << maxs);
    ROS_INFO_STREAM("Desired speed: " << mDetector.getDesiredSpeed());

    image_sub = it.subscribe("/image", 1, &ArucoMultiBoard::image_callback, this);
    cam_info_sub = nh.subscribe("/camera_info", 1, &ArucoMultiBoard::cam_info_callback, this);

    image_pub = it.advertise("result", 1);
    debug_pub = it.advertise("debug", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);
    transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
    position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);
    marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 10);

    nh.param<double>("marker_size", marker_size, 0.05);
    nh.param<std::string>("board_config", board_config_file, "");
    nh.param<std::string>("board_dir", board_config_dir, "");
    nh.param<std::string>("reference_frame", reference_frame, "");
    nh.param<std::string>("camera_frame", camera_frame, "");
    nh.param<bool>("image_is_rectified", useRectifiedImages, true);
    nh.param<bool>("draw_markers", draw_markers, false);
    nh.param<bool>("rotate_marker_axis", rotate_marker_axis, false);

    ROS_ASSERT(board_config_file != "" && board_config_dir != "");

    readBoardConfigList();
    ROS_ASSERT(!camera_frame.empty());

    if (reference_frame.empty())
      reference_frame = camera_frame;

    ROS_INFO("Aruco node will publish pose to TF with %s as parent",
             reference_frame.c_str());

    dyn_rec_server.setCallback(boost::bind(&ArucoMultiBoard::reconf_callback, this, _1, _2));
  }

  void readBoardConfigList() {
    try {
      cv::FileStorage fs(board_config_file, cv::FileStorage::READ);

      if (fs["aruco_boards"].name() != "aruco_boards")
        throw cv::Exception(81818, "ArucoMultiBoard::readBoardConfigList", "invalid file type", __FILE__, __LINE__);

      cv::FileNode FnBoards = fs["aruco_boards"];
      for (cv::FileNodeIterator it = FnBoards.begin(); it != FnBoards.end(); ++it) {
        board_t board;
        board.uid = (int) boards.size();
        board.frame_id = (std::string) (*it)["frame_id"];
        std::string path(board_config_dir);
        if (path.back() != '/')
          path.append("/");
        path.append((std::string) (*it)["filename"]);
        board.config.readFromFile(path);
        boards.push_back(board);
      }
      ROS_ASSERT(!boards.empty());
    }
    catch (std::exception &ex) {
      throw cv::Exception(81818,
                          "ArucoMultiBoard::readBoardConfigList",
                          ex.what() + string(" file=)") + board_config_file,
                          __FILE__,
                          __LINE__);
    }
  }

  bool getTransform(const std::string &refFrame,
                    const std::string &childFrame,
                    tf::StampedTransform &transform) {
    std::string errMsg;

    if (!_tfListener.waitForTransform(refFrame,
                                      childFrame,
                                      ros::Time(0),
                                      ros::Duration(0.5),
                                      ros::Duration(0.01),
                                      &errMsg)
        ) {
      ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
      return false;
    } else {
      try {
        _tfListener.lookupTransform(refFrame, childFrame,
                                    ros::Time(0),                  //get latest available
                                    transform);
      }
      catch (const tf::TransformException &e) {
        ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
        return false;
      }

    }
    return true;
  }

  void image_callback(const sensor_msgs::ImageConstPtr &msg) {

    if ((image_pub.getNumSubscribers() == 0) &&
        (debug_pub.getNumSubscribers() == 0) &&
        (pose_pub.getNumSubscribers() == 0) &&
        (transform_pub.getNumSubscribers() == 0) &&
        (position_pub.getNumSubscribers() == 0) &&
        (marker_pub.getNumSubscribers() == 0))
    {
      ROS_DEBUG("No subscribers, not looking for aruco markers");
      return;
    }

    static tf::TransformBroadcaster br;

    if (!cam_info_received) return;

    ros::Time curr_stamp(ros::Time::now());

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      inImage = cv_ptr->image;

      //detection results will go into "markers"
      markers.clear();

      //Ok, let's detect
      mDetector.detect(inImage, markers, camParam, marker_size, false);

      for (int board_index = 0; board_index < boards.size(); board_index++) {
        Board board_detected;

        //Detection of the board
        float probDetect = board_detector.detect(markers,
                                                 boards[board_index].config,
                                                 board_detected,
                                                 camParam,
                                                 marker_size);
        if (probDetect > 0.0) {
          tf::Transform transform = aruco_ros::arucoBoard2Tf(board_detected, rotate_marker_axis);

          tf::StampedTransform cameraToReference;
          cameraToReference.setIdentity();

          if (reference_frame != camera_frame) {
            getTransform(reference_frame,
                         camera_frame,
                         cameraToReference);
          }

          transform =
              static_cast<tf::Transform>(cameraToReference)
                  * static_cast<tf::Transform>(rightToLeft)
                  * transform;

          tf::StampedTransform stampedTransform(transform, curr_stamp, reference_frame, boards[board_index].frame_id);
          br.sendTransform(stampedTransform);

          geometry_msgs::PoseStamped poseMsg;
          tf::poseTFToMsg(transform, poseMsg.pose);
          poseMsg.header.frame_id = reference_frame;
          poseMsg.header.stamp = curr_stamp;
          pose_pub.publish(poseMsg);

          geometry_msgs::TransformStamped transformMsg;
          tf::transformStampedTFToMsg(stampedTransform, transformMsg);
          transform_pub.publish(transformMsg);

          geometry_msgs::Vector3Stamped positionMsg;
          positionMsg.header = transformMsg.header;
          positionMsg.vector = transformMsg.transform.translation;
          position_pub.publish(positionMsg);

          //Publish rviz marker representing the ArUco marker patch
          visualization_msgs::Marker visMarker;
          visMarker.header = transformMsg.header;
          visMarker.id = 1;
          visMarker.type   = visualization_msgs::Marker::CUBE;
          visMarker.action = visualization_msgs::Marker::ADD;
          visMarker.pose = poseMsg.pose;
          visMarker.scale.x = marker_size;
          visMarker.scale.y = 0.001;
          visMarker.scale.z = marker_size;
          visMarker.color.r = 1.0;
          visMarker.color.g = 0;
          visMarker.color.b = 0;
          visMarker.color.a = 1.0;
          visMarker.lifetime = ros::Duration(3.0);
          marker_pub.publish(visMarker);

          if (camParam.isValid()) {
            //draw board axis
            CvDrawingUtils::draw3dAxis(inImage, board_detected, camParam);
          }
        }
      }

      //for each marker, draw info and its boundaries in the image
      for (size_t i = 0; draw_markers && i < markers.size(); ++i) {
        markers[i].draw(inImage, cv::Scalar(0, 0, 255), 2);
      }

      if (image_pub.getNumSubscribers() > 0) {
        //show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.frame_id = msg->header.frame_id;
        out_msg.header.stamp = msg->header.stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage;
        image_pub.publish(out_msg.toImageMsg());
      }

      if (debug_pub.getNumSubscribers() > 0) {
        //show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.frame_id = msg->header.frame_id;
        debug_msg.header.stamp = msg->header.stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector.getThresholdedImage();
        debug_pub.publish(debug_msg.toImageMsg());
      }
    }
    catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::CameraInfo &msg) {
    camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);
    // handle cartesian offset between stereo pairs
    // see the sensor_msgs/CamaraInfo documentation for details
    rightToLeft.setIdentity();
    rightToLeft.setOrigin(
        tf::Vector3(
            -msg.P[3] / msg.P[0],
            -msg.P[7] / msg.P[5],
            0.0));

    cam_info_received = true;
    cam_info_sub.shutdown();
    ROS_INFO("Received camera info");
  }

  void reconf_callback(aruco_ros::ArucoThresholdConfig &config, uint32_t level) {
    mDetector.setThresholdParams(config.param1, config.param2);
    if (config.normalizeImage) {
      ROS_WARN("normalizeImageIllumination is unimplemented!");
    }
  }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "aruco_multi_board");

  ArucoMultiBoard node;

  ros::spin();
}
