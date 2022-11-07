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
#include "fractaldetector.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "cvdrawingutils.h"
#include <algorithm>
#include "aruco_cvversioning.h"

namespace aruco
{
FractalDetector::FractalDetector()
{
  _markerDetector = new MarkerDetector();
  MarkerDetector::Params p;
  p.markerWarpPixSize = 10;
  _markerDetector->setParameters(p);
};

void FractalDetector::setConfiguration(int params)
{
  _fractalLabeler = FractalMarkerLabeler::create((FractalMarkerSet::CONF_TYPES)params);
  _params.configuration_type =
      FractalMarkerSet::getTypeString((FractalMarkerSet::CONF_TYPES)params);
  _markerDetector->setMarkerLabeler(_fractalLabeler);
}

void FractalDetector::setConfiguration(std::string params)
{
  _params.configuration_type = params;
  _fractalLabeler = FractalMarkerLabeler::create(params);
  _markerDetector->setMarkerLabeler(_fractalLabeler);
}

void FractalDetector::drawMarkers(cv::Mat &img)
{
  float size = std::max(1., float(img.cols) / 1280.);
  for (auto m : Markers)
    m.draw(img, cv::Scalar(0, 0, 255), size, false);
}

void FractalDetector::draw2d(cv::Mat &img)
{
  if (Markers.size() > 0)
  {
    std::map<int, FractalMarker> id_fmarker =
        _fractalLabeler->_fractalMarkerSet.fractalMarkerCollection;

    std::vector<cv::Point2f> inners;
    std::map<int, std::vector<cv::Point3f>> id_innerCorners =
        _fractalLabeler->_fractalMarkerSet.getInnerCorners();
    for (auto id_innerC : id_innerCorners)
    {
      std::vector<cv::Point3f> inner3d;
      for (auto pt : id_innerC.second)
        inners.push_back(cv::Point2f(pt.x, pt.y));
    }

    std::vector<cv::Point2f> points3d;
    std::vector<cv::Point2f> points2d;
    for (auto m : Markers)
    {
      for (auto p : id_fmarker[m.id].points)
        points3d.push_back(cv::Point2f(p.x, p.y));

      for (auto p : m)
        points2d.push_back(p);
    }

    cv::Mat H = cv::findHomography(points3d, points2d);
    std::vector<cv::Point2f> dstPnt;
    cv::perspectiveTransform(inners, dstPnt, H);

    float size = std::max(1., float(img.cols) / 1280.);
    for (auto p : dstPnt)
      cv::circle(img, p, size, cv::Scalar(0, 0, 255), CV_FILLED);
  }
}

void FractalDetector::draw3d(cv::Mat &img, bool cube, bool axis)
{
  if (Tracker.isPoseValid())
  {
    cv::Mat rot;
    cv::Rodrigues(Tracker.getRvec(), rot);

    std::vector<cv::Point3f> innerPoints3d;
    for (auto pt : Tracker.getInner3d())
    {
      cv::Mat_<double> src(3, 1, rot.type());
      src(0, 0) = pt.x;
      src(1, 0) = pt.y;
      src(2, 0) = pt.z;

      cv::Mat cam_image_point = rot * src + Tracker.getTvec();
      cam_image_point = cam_image_point / cv::norm(cam_image_point);

      if (cam_image_point.at<double>(2, 0) > 0.85)
        innerPoints3d.push_back(pt);
    }
    // Draw inner points
    if (innerPoints3d.size() > 0)
    {
      std::vector<cv::Point2f> innerPoints;
      projectPoints(innerPoints3d, Tracker.getRvec(), Tracker.getTvec(),
                    _cam_params.CameraMatrix, _cam_params.Distorsion, innerPoints);
      for (auto p : innerPoints)
        circle(img, p, 3, cv::Scalar(0, 0, 255), CV_FILLED);
    }
    // Draw cube
    if (cube)
    {
      std::map<int, FractalMarker> id_fmarker = Tracker.getFractal().fractalMarkerCollection;
      for (auto m : Markers)
        draw3dCube(img, id_fmarker[m.id], _cam_params, 2);
    }

    // Draw axes
    if (axis)
      CvDrawingUtils::draw3dAxis(img, _cam_params, getRvec(), getTvec(),
                                 Tracker.getFractal().getFractalSize() / 2);
  }
}

void FractalDetector::draw3dCube(cv::Mat &Image, FractalMarker m,
                                 const CameraParameters &CP, int lineSize)
{
  cv::Mat objectPoints(8, 3, CV_32FC1);

  float msize = m.getMarkerSize();
  float halfSize = msize / 2;

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
  objectPoints.at<float>(4, 2) = msize;
  objectPoints.at<float>(5, 0) = halfSize;
  objectPoints.at<float>(5, 1) = -halfSize;
  objectPoints.at<float>(5, 2) = msize;
  objectPoints.at<float>(6, 0) = halfSize;
  objectPoints.at<float>(6, 1) = halfSize;
  objectPoints.at<float>(6, 2) = msize;
  objectPoints.at<float>(7, 0) = -halfSize;
  objectPoints.at<float>(7, 1) = halfSize;
  objectPoints.at<float>(7, 2) = msize;


  std::vector<cv::Point2f> imagePoints;
  projectPoints(objectPoints, getRvec(), getTvec(), CP.CameraMatrix, CP.Distorsion, imagePoints);

  for (int i = 0; i < 4; i++)
    cv::line(Image, imagePoints[i], imagePoints[(i + 1) % 4], cv::Scalar(0, 0, 255, 255), lineSize);

  for (int i = 0; i < 4; i++)
    cv::line(Image, imagePoints[i + 4], imagePoints[4 + (i + 1) % 4],
             cv::Scalar(0, 0, 255, 255), lineSize);

  for (int i = 0; i < 4; i++)
    cv::line(Image, imagePoints[i], imagePoints[i + 4], cv::Scalar(0, 0, 255, 255), lineSize);
}
};  // namespace aruco
