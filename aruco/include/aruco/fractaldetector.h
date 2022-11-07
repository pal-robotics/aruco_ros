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

#ifndef _ARUCO_FractalDetector_H
#define _ARUCO_FractalDetector_H

#include "markerdetector.h"
#include "fractallabelers/fractallabeler.h"
#include "aruco_export.h"
namespace aruco
{
class ARUCO_EXPORT FractalDetector
{
  struct ARUCO_EXPORT Params
  {
    std::string configuration_type;
  };

public:
  FractalDetector();

  /**
   * @brief setConfiguration
   * @param configuration fractal id
   */
  void setConfiguration(int configuration);

  /**
   * @brief setConfiguration
   * @param configuration fractal file
   */
  void setConfiguration(std::string configuration);

  /**
   * @brief setParams
   * @param cam_params camera parameters
   * @param markerSize in meters
   */
  void setParams(const CameraParameters &cam_params, float markerSize)
  {
    _cam_params = cam_params;

    Tracker.setParams(cam_params, _fractalLabeler->_fractalMarkerSet, markerSize);
  }

  // return fractalmarkerset
  FractalMarkerSet getConfiguration()
  {
    return Tracker.getFractal();
  }

  // return true if any marker is detected, false otherwise
  bool detect(const cv::Mat &input)
  {
    Markers = _markerDetector->detect(input);

    if (Markers.size() > 0)
      return true;
    else
      return false;
  }

  // return true if the pose is estimated, false otherwise
  bool poseEstimation()
  {
    if (_cam_params.isValid())
    {
      return Tracker.fractalInnerPose(_markerDetector, Markers);
    }
    else
      return false;
  }

  // return the rotation vector. Returns an empty matrix if last call to estimatePose returned false
  cv::Mat getRvec()
  {
    return Tracker.getRvec();
  }
  // return the translation vector. Returns an empty matrix if last call to estimatePose returned false
  cv::Mat getTvec()
  {
    return Tracker.getTvec();
  }

  void drawImage(cv::Mat &img, cv::Mat &img2);

  // draw borders of markers
  void drawMarkers(cv::Mat &img);

  // draw inner corners of markers
  void draw2d(cv::Mat &img);

  // draw pose estimation axes
  void draw3d(cv::Mat &img, bool cube = true, bool axis = true);

  // draw marker as cube
  void draw3dCube(cv::Mat &Image, FractalMarker m, const CameraParameters &CP, int lineSize);

  // return detected markers
  std::vector<Marker> getMarkers()
  {
    return Markers;
  }

  // return image pyramid
  std::vector<cv::Mat> getImagePyramid()
  {
    return _markerDetector->getImagePyramid();
  }

private:
  std::vector<aruco::Marker> Markers;  // detected markers
  FractalPoseTracker Tracker;
  Params _params;
  CameraParameters _cam_params;  // Camera parameters
  cv::Ptr<FractalMarkerLabeler> _fractalLabeler;
  cv::Ptr<MarkerDetector> _markerDetector;
};
}  // namespace aruco
#endif
