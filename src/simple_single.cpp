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
* @brief ROS version of the example named "simple" in the Aruco software package.
*/

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>

using namespace aruco;

class ArucoSimple
{
private:
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  bool useRectifiedImages;
  MarkerDetector mDetector;
  vector<Marker> markers;
  ros::Subscriber cam_info_sub;
  bool cam_info_received;
  image_transport::Publisher image_pub;
  image_transport::Publisher debug_pub;
  ros::Publisher pose_pub;
  std::string child_name;
  std::string parent_name;

  double marker_size;
  int marker_id;

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;

public:
  ArucoSimple()
    : cam_info_received(false),
      nh("~"),
      it(nh)
  {
    image_sub = it.subscribe("/image", 1, &ArucoSimple::image_callback, this);
    cam_info_sub = nh.subscribe("/camera_info", 1, &ArucoSimple::cam_info_callback, this);

    image_pub = it.advertise("result", 1);
    debug_pub = it.advertise("debug", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 100);

    nh.param<double>("marker_size", marker_size, 0.05);
    nh.param<int>("marker_id", marker_id, 300);
    nh.param<std::string>("parent_name", parent_name, "");
    nh.param<std::string>("child_name", child_name, "");
    nh.param<bool>("image_is_rectified", useRectifiedImages, true);

    ROS_ASSERT(parent_name != "" && child_name != "");

    ROS_INFO("Aruco node started with marker size of %f m and marker id to track: %d",
             marker_size, marker_id);
    ROS_INFO("Aruco node will publish pose to TF with %s as parent and %s as child.",
             parent_name.c_str(), child_name.c_str());
  }

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    static tf::TransformBroadcaster br;
    if(cam_info_received)
    {
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
        inImage = cv_ptr->image;

        //detection results will go into "markers"
        markers.clear();
        //Ok, let's detect
        mDetector.detect(inImage, markers, camParam, marker_size, false);
        //for each marker, draw info and its boundaries in the image
        for(size_t i=0; i<markers.size(); ++i)
        {
          // only publishing the selected marker
          if(markers[i].id == marker_id)
          {
            tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
            tf::StampedTransform stampedTransform(transform, ros::Time::now(),
                                                  parent_name, child_name);
            br.sendTransform(stampedTransform);
            geometry_msgs::PoseStamped poseMsg;
            tf::poseTFToMsg(transform, poseMsg.pose);
            poseMsg.header.frame_id = parent_name;
            poseMsg.header.stamp = ros::Time::now();
            pose_pub.publish(poseMsg);
          }
          // but drawing all the detected markers
          markers[i].draw(inImage,cv::Scalar(0,0,255),2);
        }

        //draw a 3d cube in each marker if there is 3d info
        if(camParam.isValid() && marker_size!=-1)
        {
          for(size_t i=0; i<markers.size(); ++i)
          {
            CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
          }
        }

        if(image_pub.getNumSubscribers() > 0)
        {
          //show input with augmented information
          cv_bridge::CvImage out_msg;
          out_msg.header.stamp = ros::Time::now();
          out_msg.encoding = sensor_msgs::image_encodings::RGB8;
          out_msg.image = inImage;
          image_pub.publish(out_msg.toImageMsg());
        }

        if(debug_pub.getNumSubscribers() > 0)
        {
          //show also the internal image resulting from the threshold operation
          cv_bridge::CvImage debug_msg;
          debug_msg.header.stamp = ros::Time::now();
          debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
          debug_msg.image = mDetector.getThresholdedImage();
          debug_pub.publish(debug_msg.toImageMsg());
        }
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
  }

  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(const sensor_msgs::CameraInfo &msg)
  {
    camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);
    cam_info_received = true;
    cam_info_sub.shutdown();
  }
};


int main(int argc,char **argv)
{
  ros::init(argc, argv, "aruco_simple");

  ArucoSimple node;

  ros::spin();
}
