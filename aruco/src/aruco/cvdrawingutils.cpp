/**
Copyright 2020 Rafael Muñoz Salinas. All rights reserved.

  This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation version 3 of the License.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cvdrawingutils.h"
#include "cameraparameters.h"
using namespace cv;
namespace aruco
{
void CvDrawingUtils::draw3dAxis(cv::Mat& Image, const CameraParameters& CP,
                                const cv::Mat& Rvec, const cv::Mat& Tvec, float axis_size)
{
  Mat objectPoints(4, 3, CV_32FC1);
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

  std::vector<Point2f> imagePoints;
  cv::projectPoints(objectPoints, Rvec, Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);
  // draw lines of different colours
  cv::line(Image, imagePoints[0], imagePoints[1], Scalar(0, 0, 255, 255), 1);
  cv::line(Image, imagePoints[0], imagePoints[2], Scalar(0, 255, 0, 255), 1);
  cv::line(Image, imagePoints[0], imagePoints[3], Scalar(255, 0, 0, 255), 1);
  putText(Image, "x", imagePoints[1], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255, 255), 2);
  putText(Image, "y", imagePoints[2], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0, 255), 2);
  putText(Image, "z", imagePoints[3], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0, 255), 2);
}
/****
 *
 *
 *
 ****/
void CvDrawingUtils::draw3dAxis(cv::Mat& Image, Marker& m, const CameraParameters& CP, int lineSize)
{
  float size = m.ssize * 0.6;
  Mat objectPoints(4, 3, CV_32FC1);
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

  std::vector<Point2f> imagePoints;
  cv::projectPoints(objectPoints, m.Rvec, m.Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);
  // draw lines of different colours
  cv::line(Image, imagePoints[0], imagePoints[1], Scalar(0, 0, 255, 255), lineSize);
  cv::line(Image, imagePoints[0], imagePoints[2], Scalar(0, 255, 0, 255), lineSize);
  cv::line(Image, imagePoints[0], imagePoints[3], Scalar(255, 0, 0, 255), lineSize);
  putText(Image, "x", imagePoints[1], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255, 255), 2);
  putText(Image, "y", imagePoints[2], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0, 255), 2);
  putText(Image, "z", imagePoints[3], FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0, 255), 2);
}

/****
 *
 *
 *
 ****/
void CvDrawingUtils::draw3dCube(cv::Mat& Image, Marker& m, const CameraParameters& CP,
                                int lineSize, bool setYperpendicular)
{
  Mat objectPoints(8, 3, CV_32FC1);
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

  std::vector<Point2f> imagePoints;
  projectPoints(objectPoints, m.Rvec, m.Tvec, CP.CameraMatrix, CP.Distorsion, imagePoints);
  // draw lines of different colours
  for (int i = 0; i < 4; i++)
    cv::line(Image, imagePoints[i], imagePoints[(i + 1) % 4], Scalar(0, 0, 255, 255), lineSize);

  for (int i = 0; i < 4; i++)
    cv::line(Image, imagePoints[i + 4], imagePoints[4 + (i + 1) % 4],
             Scalar(0, 0, 255, 255), lineSize);

  for (int i = 0; i < 4; i++)
    cv::line(Image, imagePoints[i], imagePoints[i + 4], Scalar(0, 0, 255, 255), lineSize);
}
}  // namespace aruco
