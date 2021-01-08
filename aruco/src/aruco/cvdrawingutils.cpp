/**
 Copyright 2017 Rafael Muñoz Salinas. All rights reserved.

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
 */

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "cvdrawingutils.h"
#include "cameraparameters.h"

namespace aruco
{

void CvDrawingUtils::draw3dAxis(cv::Mat& Image, const CameraParameters& CP, const cv::Mat& Rvec, const cv::Mat& Tvec,
                                float axis_size)
{
  cv::Mat objectPoints(4, 3, CV_32FC1);
  objectPoints.at<float>(0, 0) = 0;
  objectPoints.at<float>(0, 1) = 0;
  objectPoints.at<float>(0, 2) = 0;
  objectPoints.at<float>(1, 0) = axis_size;
  objectPoints.at<float>(1, 1) = 0;
  objectPoints.at<float>(1, 2) = 0;
  objectPoints.at<float>(2, 0) = 0;
  objectPoints.at<float>(2, 1) = axis_size;
  objectPoints.at<float>(2, 2) = 0;
  objectPoints.at<float>(3, 0) = 0;
  objectPoints.at<float>(3, 1) = 0;
  objectPoints.at<float>(3, 2) = axis_size;

  std::vector<cv::Point2f> imagePoints;
  cv::projectPoints(objectPoints, Rvec, Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);

  // draw lines of different colors
  cv::line(Image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255, 255), 1, cv::LINE_AA);
  cv::line(Image, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0, 255), 1, cv::LINE_AA);
  cv::line(Image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0, 255), 1, cv::LINE_AA);
  putText(Image, "x", imagePoints[1], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255, 255), 2);
  putText(Image, "y", imagePoints[2], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0, 255), 2);
  putText(Image, "z", imagePoints[3], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0, 255), 2);
}

/**
 *
 *
 *
 */
void CvDrawingUtils::draw3dAxis(cv::Mat& Image, Marker& m, const CameraParameters& CP)
{
  float size = m.ssize * 3;
  cv::Mat objectPoints(4, 3, CV_32FC1);
  objectPoints.at<float>(0, 0) = 0;
  objectPoints.at<float>(0, 1) = 0;
  objectPoints.at<float>(0, 2) = 0;
  objectPoints.at<float>(1, 0) = size;
  objectPoints.at<float>(1, 1) = 0;
  objectPoints.at<float>(1, 2) = 0;
  objectPoints.at<float>(2, 0) = 0;
  objectPoints.at<float>(2, 1) = size;
  objectPoints.at<float>(2, 2) = 0;
  objectPoints.at<float>(3, 0) = 0;
  objectPoints.at<float>(3, 1) = 0;
  objectPoints.at<float>(3, 2) = size;

  std::vector<cv::Point2f> imagePoints;
  cv::projectPoints(objectPoints, m.Rvec, m.Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);

  // draw lines of different colors
  cv::line(Image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255, 255), 1, cv::LINE_AA);
  cv::line(Image, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0, 255), 1, cv::LINE_AA);
  cv::line(Image, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0, 255), 1, cv::LINE_AA);
  putText(Image, "x", imagePoints[1], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255, 255), 2);
  putText(Image, "y", imagePoints[2], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0, 255), 2);
  putText(Image, "z", imagePoints[3], cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0, 255), 2);
}

/**
 *
 *
 *
 */
void CvDrawingUtils::draw3dCube(cv::Mat& Image, Marker& m, const CameraParameters& CP, bool setYperpendicular)
{
  cv::Mat objectPoints(8, 3, CV_32FC1);
  float halfSize = m.ssize / 2.f;

  if (setYperpendicular)
  {
    objectPoints.at<float>(0, 0) = -halfSize;
    objectPoints.at<float>(0, 1) = 0;
    objectPoints.at<float>(0, 2) = -halfSize;
    objectPoints.at<float>(1, 0) = halfSize;
    objectPoints.at<float>(1, 1) = 0;
    objectPoints.at<float>(1, 2) = -halfSize;
    objectPoints.at<float>(2, 0) = halfSize;
    objectPoints.at<float>(2, 1) = 0;
    objectPoints.at<float>(2, 2) = halfSize;
    objectPoints.at<float>(3, 0) = -halfSize;
    objectPoints.at<float>(3, 1) = 0;
    objectPoints.at<float>(3, 2) = halfSize;

    objectPoints.at<float>(4, 0) = -halfSize;
    objectPoints.at<float>(4, 1) = m.ssize;
    objectPoints.at<float>(4, 2) = -halfSize;
    objectPoints.at<float>(5, 0) = halfSize;
    objectPoints.at<float>(5, 1) = m.ssize;
    objectPoints.at<float>(5, 2) = -halfSize;
    objectPoints.at<float>(6, 0) = halfSize;
    objectPoints.at<float>(6, 1) = m.ssize;
    objectPoints.at<float>(6, 2) = halfSize;
    objectPoints.at<float>(7, 0) = -halfSize;
    objectPoints.at<float>(7, 1) = m.ssize;
    objectPoints.at<float>(7, 2) = halfSize;
  }
  else
  {
    objectPoints.at<float>(0, 0) = -halfSize;
    objectPoints.at<float>(0, 1) = -halfSize;
    objectPoints.at<float>(0, 2) = 0;
    objectPoints.at<float>(1, 0) = halfSize;
    objectPoints.at<float>(1, 1) = -halfSize;
    objectPoints.at<float>(1, 2) = 0;
    objectPoints.at<float>(2, 0) = halfSize;
    objectPoints.at<float>(2, 1) = halfSize;
    objectPoints.at<float>(2, 2) = 0;
    objectPoints.at<float>(3, 0) = -halfSize;
    objectPoints.at<float>(3, 1) = halfSize;
    objectPoints.at<float>(3, 2) = 0;

    objectPoints.at<float>(4, 0) = -halfSize;
    objectPoints.at<float>(4, 1) = -halfSize;
    objectPoints.at<float>(4, 2) = m.ssize;
    objectPoints.at<float>(5, 0) = halfSize;
    objectPoints.at<float>(5, 1) = -halfSize;
    objectPoints.at<float>(5, 2) = m.ssize;
    objectPoints.at<float>(6, 0) = halfSize;
    objectPoints.at<float>(6, 1) = halfSize;
    objectPoints.at<float>(6, 2) = m.ssize;
    objectPoints.at<float>(7, 0) = -halfSize;
    objectPoints.at<float>(7, 1) = halfSize;
    objectPoints.at<float>(7, 2) = m.ssize;
  }

  std::vector<cv::Point2f> imagePoints;
  projectPoints(objectPoints, m.Rvec, m.Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);

  // draw lines of different colors
  for (int i = 0; i < 4; i++)
    cv::line(Image, imagePoints[i], imagePoints[(i + 1) % 4], cv::Scalar(0, 0, 255, 255), 1, cv::LINE_AA);

  for (int i = 0; i < 4; i++)
    cv::line(Image, imagePoints[i + 4], imagePoints[4 + (i + 1) % 4], cv::Scalar(0, 0, 255, 255), 1, cv::LINE_AA);

  for (int i = 0; i < 4; i++)
    cv::line(Image, imagePoints[i], imagePoints[i + 4], cv::Scalar(0, 0, 255, 255), 1, cv::LINE_AA);
}

} // namespace aruco
