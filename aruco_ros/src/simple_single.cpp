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
#include <aruco_msgs/Marker.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace aruco;

class ArucoSimple
{
private:
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  tf::StampedTransform rightToLeft;
  bool useRectifiedImages;
  MarkerDetector mDetector;
  vector<Marker> markers;
  ros::Subscriber cam_info_sub;
  bool cam_info_received;
  image_transport::Publisher image_pub;
  image_transport::Publisher debug_pub;
  ros::Publisher pose_pub;
  ros::Publisher transform_pub; 
  ros::Publisher position_pub;
  std::string marker_frame;
  std::string camera_frame;
  std::string reference_frame;

  double marker_size;

  int marker_id[11]; // Hardcoded, max number of aruco code that can be detected is 11
  int num_markers_in_list; //

  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;

  tf::TransformListener _tfListener;

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
    pose_pub = nh.advertise<aruco_msgs::Marker>("pose", 100);
    transform_pub = nh.advertise<geometry_msgs::TransformStamped>("transform", 100);
    position_pub = nh.advertise<geometry_msgs::Vector3Stamped>("position", 100);

    nh.param<double>("marker_size", marker_size, 0.05);
    nh.param<int>("num_markers", num_markers_in_list, 11);
    nh.param<int>("marker_id_0", marker_id[0], 500);
    nh.param<int>("marker_id_1", marker_id[1], 582);
    nh.param<int>("marker_id_2", marker_id[2], 590);
    nh.param<int>("marker_id_3", marker_id[3], 614);
    nh.param<int>("marker_id_4", marker_id[4], 625);
    nh.param<int>("marker_id_5", marker_id[5], 750);
    nh.param<int>("marker_id_6", marker_id[6], 785);
    nh.param<int>("marker_id_7", marker_id[7], 798);
    nh.param<int>("marker_id_8", marker_id[8], 825);
    nh.param<int>("marker_id_9", marker_id[9], 921);
    nh.param<int>("marker_id_10", marker_id[10], 945);
    nh.param<std::string>("reference_frame", reference_frame, "");
    nh.param<std::string>("camera_frame", camera_frame, "");
    nh.param<std::string>("marker_frame", marker_frame, "");
    nh.param<bool>("image_is_rectified", useRectifiedImages, true);

    ROS_ASSERT(camera_frame != "" && marker_frame != "");
    ROS_ASSERT(num_markers_in_list <= 11);

    if ( reference_frame.empty() )
      reference_frame = camera_frame;

    ROS_INFO("Aruco node started with marker size of %f m and marker id to track: %d %d %d %d %d %d %d %d %d %d %d",
             marker_size, marker_id[0], marker_id[1], marker_id[2], marker_id[3], marker_id[4], marker_id[5],
             marker_id[6], marker_id[7], marker_id[8], marker_id[9], marker_id[10]);
    ROS_INFO("Aruco node will publish pose to TF with %s as parent and %s as child.",
             reference_frame.c_str(), marker_frame.c_str());
  }

  bool getTransform(const std::string& refFrame,
                    const std::string& childFrame,
                    tf::StampedTransform& transform)
  {
    std::string errMsg;

    if ( !_tfListener.waitForTransform(refFrame,
                                       childFrame,
                                       ros::Time(0),
                                       ros::Duration(0.5),
                                       ros::Duration(0.01),
                                       &errMsg)
         )
    {
      ROS_ERROR_STREAM("Unable to get pose from TF: " << errMsg);
      return false;
    }
    else
    {
      try
      {
        _tfListener.lookupTransform( refFrame, childFrame,
                                     ros::Time(0),                  //get latest available
                                     transform);
      }
      catch ( const tf::TransformException& e)
      {
        ROS_ERROR_STREAM("Error in lookupTransform of " << childFrame << " in " << refFrame);
        return false;
      }

    }
    return true;
  }


  bool is_marker_id_in_list(int id){
    for (unsigned i = 0; i < num_markers_in_list; ++i){
      if (marker_id[i] == id){
        return true;
      }
    }
    return false;
  }


  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    static tf::TransformBroadcaster br;
    if(cam_info_received)
    {
      ros::Time curr_stamp(ros::Time::now());
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

          if (markers.size() == 1){  // Only process markers if there is only one in FOV
            // only publishing the selected markers
            if(is_marker_id_in_list(markers[i].id))
            {
              tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
              tf::StampedTransform cameraToReference;
              cameraToReference.setIdentity();

              if ( reference_frame != camera_frame )
              {
                getTransform(reference_frame,
                             camera_frame,
                             cameraToReference);
              }

              transform =
                static_cast<tf::Transform>(cameraToReference)
                * static_cast<tf::Transform>(rightToLeft)
                * transform;

              tf::StampedTransform stampedTransform(transform, curr_stamp,
                                                    reference_frame, marker_frame);
              br.sendTransform(stampedTransform);

              aruco_msgs::Marker arucoMsg;
              arucoMsg.header.frame_id = reference_frame;
              arucoMsg.header.stamp = curr_stamp;
              arucoMsg.id = markers[i].id;
              arucoMsg.confidence = 1.0;

              geometry_msgs::PoseWithCovariance poseMsg;
              tf::poseTFToMsg(transform, poseMsg.pose);

              arucoMsg.pose = poseMsg;
              pose_pub.publish(arucoMsg);

              geometry_msgs::TransformStamped transformMsg;
              tf::transformStampedTFToMsg(stampedTransform, transformMsg);
              transform_pub.publish(transformMsg);

              geometry_msgs::Vector3Stamped positionMsg;
              positionMsg.header = transformMsg.header;
              positionMsg.vector = transformMsg.transform.translation;
              position_pub.publish(positionMsg);
            }
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
          out_msg.header.stamp = curr_stamp;
          out_msg.encoding = sensor_msgs::image_encodings::RGB8;
          out_msg.image = inImage;
          image_pub.publish(out_msg.toImageMsg());
        }

        if(debug_pub.getNumSubscribers() > 0)
        {
          //show also the internal image resulting from the threshold operation
          cv_bridge::CvImage debug_msg;
          debug_msg.header.stamp = curr_stamp;
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

    // handle cartesian offset between stereo pairs
    // see the sensor_msgs/CamaraInfo documentation for details
    rightToLeft.setIdentity();
    rightToLeft.setOrigin(
        tf::Vector3(
            -msg.P[3]/msg.P[0],
            -msg.P[7]/msg.P[5],
            0.0));

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
