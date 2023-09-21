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

#include "markerdetector.h"
#include "cameraparameters.h"
#include "markerlabeler.h"
#include "timers.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <fstream>
#include <iostream>
#include <valarray>
#include <chrono>
#include <thread>
#include "debug.h"
#include "aruco_cvversioning.h"
#include "markerdetector_impl.h"

//#ifdef _DEBUG
//#include <opencv2/highgui/highgui.hpp>
//#endif
using namespace std;
using namespace cv;

namespace aruco
{
/************************************
 *
 *
 *
 *
 ************************************/
MarkerDetector::MarkerDetector()
{
  _impl = new MarkerDetector_Impl();
  //    _iplm=std::unique_ptr<MarkerDetector_Impl>(new MarkerDetector_Impl());
}
/************************************
 *
 *
 *
 *
 ************************************/
MarkerDetector::MarkerDetector(int dict_type, float error_correction_rate)
{
  _impl = new MarkerDetector_Impl();
  setDictionary(dict_type, error_correction_rate);
  setDetectionMode(DM_NORMAL);
}
/************************************
 *
 *
 *
 *
 ************************************/
MarkerDetector::MarkerDetector(std::string dict_type, float error_correction_rate)
{
  _impl = new MarkerDetector_Impl();
  setDictionary(dict_type, error_correction_rate);
  setDetectionMode(DM_NORMAL);
}
/************************************
 *
 *
 *
 *
 ************************************/

MarkerDetector::~MarkerDetector()
{
  delete _impl;
}

void MarkerDetector::setParameters(const Params &params)
{
  _impl->setParameters(params);
}

/************************************
 *
 *
 *
 *
 ************************************/
void MarkerDetector::setDetectionMode(DetectionMode dm, float minMarkerSize)
{
  _impl->_params.setDetectionMode(dm, minMarkerSize);
}

DetectionMode MarkerDetector::getDetectionMode()
{
  return _impl->_params.detectMode;
}



/************************************
 *
 *
 *
 *
 ************************************/

std::vector<aruco::Marker> MarkerDetector::detect(const cv::Mat &input)
{
  return _impl->detect(input);
}

std::vector<aruco::Marker> MarkerDetector::detect(const cv::Mat &input,
                                                  const CameraParameters &camParams,
                                                  float markerSizeMeters,
                                                  bool setYPerperdicular, bool correctFisheye)
{
  return _impl->detect(input, camParams, markerSizeMeters, setYPerperdicular, correctFisheye);
}

/************************************
 *
 *
 *
 *
 ************************************/
void MarkerDetector::detect(const cv::Mat &input, std::vector<Marker> &detectedMarkers,
                            CameraParameters camParams, float markerSizeMeters,
                            bool setYPerpendicular, bool correctFisheye)
{
  _impl->detect(input, detectedMarkers, camParams, markerSizeMeters, setYPerpendicular, correctFisheye);
}

/**Returns operating params
 */
MarkerDetector::Params MarkerDetector::getParameters() const
{
  return _impl->getParameters();
}
/**Returns operating params
 */
MarkerDetector::Params &MarkerDetector::getParameters()
{
  return _impl->getParameters();
}

std::vector<MarkerCandidate> MarkerDetector::getCandidates() const
{
  return _impl->getCandidates();
}

std::vector<cv::Mat> MarkerDetector::getImagePyramid()
{
  return _impl->getImagePyramid();
}
cv::Ptr<MarkerLabeler> MarkerDetector::getMarkerLabeler()
{
  return _impl->getMarkerLabeler();
}
void MarkerDetector::setMarkerLabeler(cv::Ptr<MarkerLabeler> detector)
{
  _impl->setMarkerLabeler(detector);
}

void MarkerDetector::setDictionary(int dict_type, float error_correction_rate)
{
  _impl->setDictionary(dict_type, error_correction_rate);
}


void MarkerDetector::setDictionary(string dict_type, float error_correction_rate)
{
  _impl->setDictionary(dict_type, error_correction_rate);
}
cv::Mat MarkerDetector::getThresholdedImage(uint32_t idx)
{
  return _impl->getThresholdedImage(idx);
}

void MarkerDetector::Params::save(cv::FileStorage &fs) const
{
  fs << "aruco-dictionary" << dictionary;
  fs << "aruco-detectMode" << toString(detectMode);
  fs << "aruco-cornerRefinementM" << toString(cornerRefinementM);
  fs << "aruco-thresMethod" << toString(thresMethod);
  fs << "aruco-maxThreads" << maxThreads;
  fs << "aruco-borderDistThres" << borderDistThres;
  fs << "aruco-lowResMarkerSize" << lowResMarkerSize;
  fs << "aruco-minSize" << minSize;
  fs << "aruco-minSize_pix" << minSize_pix;
  fs << "aruco-enclosedMarker" << enclosedMarker;
  fs << "aruco-NAttemptsAutoThresFix" << NAttemptsAutoThresFix;
  fs << "aruco-AdaptiveThresWindowSize" << AdaptiveThresWindowSize;
  fs << "aruco-ThresHold" << ThresHold;
  fs << "aruco-AdaptiveThresWindowSize_range" << AdaptiveThresWindowSize_range;
  fs << "aruco-markerWarpPixSize" << markerWarpPixSize;
  fs << "aruco-autoSize" << autoSize;
  fs << "aruco-ts" << ts;
  fs << "aruco-pyrfactor" << pyrfactor;
  fs << "aruco-error_correction_rate" << error_correction_rate;
  fs << "aruco-trackingMinDetections" << trackingMinDetections;
  fs << "aruco-closingSize" << closingSize;
}

void MarkerDetector::Params::load(cv::FileStorage &fs)
{
  attemtpRead("aruco-maxThreads", maxThreads, fs);
  attemtpRead("aruco-borderDistThres", borderDistThres, fs);
  attemtpRead("aruco-lowResMarkerSize", lowResMarkerSize, fs);
  attemtpRead("aruco-minSize", minSize, fs);
  attemtpRead("aruco-minSize_pix", minSize_pix, fs);
  attemtpRead("aruco-enclosedMarker", enclosedMarker, fs);
  attemtpRead("aruco-NAttemptsAutoThresFix", NAttemptsAutoThresFix, fs);
  attemtpRead("aruco-AdaptiveThresWindowSize", AdaptiveThresWindowSize, fs);
  attemtpRead("aruco-ThresHold", ThresHold, fs);
  attemtpRead("aruco-AdaptiveThresWindowSize_range", AdaptiveThresWindowSize_range, fs);
  attemtpRead("aruco-markerWarpPixSize", markerWarpPixSize, fs);
  attemtpRead("aruco-autoSize", autoSize, fs);
  attemtpRead("aruco-ts", ts, fs);
  attemtpRead("aruco-pyrfactor", pyrfactor, fs);
  attemtpRead("aruco-error_correction_rate", error_correction_rate, fs);
  attemtpRead("aruco-dictionary", dictionary, fs);
  attemtpRead("aruco-trackingMinDetections", trackingMinDetections, fs);
  attemtpRead("aruco-closingSize", closingSize, fs);



  if (fs["aruco-detectMode"].type() != cv::FileNode::NONE)
  {
    string aux;
    fs["aruco-detectMode"] >> aux;
    detectMode = getDetectionModeFromString(aux);
  }
  if (fs["aruco-thresMethod"].type() != cv::FileNode::NONE)
  {
    string aux;
    fs["aruco-thresMethod"] >> aux;
    thresMethod = getCornerThresMethodFromString(aux);
  }
  if (fs["aruco-cornerRefinementM"].type() != cv::FileNode::NONE)
  {
    string aux;
    fs["aruco-cornerRefinementM"] >> aux;
    cornerRefinementM = getCornerRefinementMethodFromString(aux);
  }
}


void MarkerDetector::Params::toStream(std::ostream &str) const
{
  str.write((char *)&detectMode, sizeof(detectMode));
  str.write((char *)&maxThreads, sizeof(maxThreads));
  str.write((char *)&borderDistThres, sizeof(borderDistThres));
  str.write((char *)&lowResMarkerSize, sizeof(lowResMarkerSize));
  str.write((char *)&minSize, sizeof(minSize));
  str.write((char *)&minSize_pix, sizeof(minSize_pix));
  str.write((char *)&enclosedMarker, sizeof(enclosedMarker));
  str.write((char *)&thresMethod, sizeof(thresMethod));
  str.write((char *)&NAttemptsAutoThresFix, sizeof(NAttemptsAutoThresFix));
  str.write((char *)&AdaptiveThresWindowSize, sizeof(AdaptiveThresWindowSize));
  str.write((char *)&ThresHold, sizeof(ThresHold));
  str.write((char *)&AdaptiveThresWindowSize_range, sizeof(AdaptiveThresWindowSize_range));
  str.write((char *)&markerWarpPixSize, sizeof(markerWarpPixSize));
  str.write((char *)&cornerRefinementM, sizeof(cornerRefinementM));
  str.write((char *)&autoSize, sizeof(autoSize));
  str.write((char *)&ts, sizeof(pyrfactor));
  str.write((char *)&error_correction_rate, sizeof(error_correction_rate));
  str.write((char *)&trackingMinDetections, sizeof(trackingMinDetections));
  str.write((char *)&closingSize, sizeof(closingSize));


  _toStream(dictionary, str);
}
void MarkerDetector::Params::fromStream(std::istream &str)
{
  str.read((char *)&detectMode, sizeof(detectMode));
  str.read((char *)&maxThreads, sizeof(maxThreads));
  str.read((char *)&borderDistThres, sizeof(borderDistThres));
  str.read((char *)&lowResMarkerSize, sizeof(lowResMarkerSize));
  str.read((char *)&minSize, sizeof(minSize));
  str.read((char *)&minSize_pix, sizeof(minSize_pix));
  str.read((char *)&enclosedMarker, sizeof(enclosedMarker));
  str.read((char *)&thresMethod, sizeof(thresMethod));
  str.read((char *)&NAttemptsAutoThresFix, sizeof(NAttemptsAutoThresFix));
  str.read((char *)&AdaptiveThresWindowSize, sizeof(AdaptiveThresWindowSize));
  str.read((char *)&ThresHold, sizeof(ThresHold));
  str.read((char *)&AdaptiveThresWindowSize_range, sizeof(AdaptiveThresWindowSize_range));
  str.read((char *)&markerWarpPixSize, sizeof(markerWarpPixSize));
  str.read((char *)&cornerRefinementM, sizeof(cornerRefinementM));
  str.read((char *)&autoSize, sizeof(autoSize));
  str.read((char *)&ts, sizeof(pyrfactor));
  str.read((char *)&error_correction_rate, sizeof(error_correction_rate));
  str.read((char *)&trackingMinDetections, sizeof(trackingMinDetections));
  str.read((char *)&closingSize, sizeof(closingSize));
  _fromStream(dictionary, str);
}
/**Saves the configuration of the detector to a file
 */
void MarkerDetector::saveParamsToFile(const std::string &path) const
{
  _impl->saveParamsToFile(path);
}

/**Loads the configuration from a file
 */
void MarkerDetector::loadParamsFromFile(const std::string &path)
{
  _impl->loadParamsFromFile(path);
}

void MarkerDetector::toStream(std::ostream &str) const
{
  _impl->toStream(str);
}

void MarkerDetector::fromStream(std::istream &str)
{
  _impl->fromStream(str);
}

std::string MarkerDetector::Params::toString(DetectionMode dm)
{
  switch (dm)
  {
    case DM_FAST:
      return "DM_FAST";
    case DM_NORMAL:
      return "DM_NORMAL";
    case DM_VIDEO_FAST:
      return "DM_VIDEO_FAST";
  };
  return "DM_NORMAL";
}

DetectionMode MarkerDetector::Params::getDetectionModeFromString(const std::string &str)
{
  if (str == "DM_FAST")
    return DM_FAST;
  if (str == "DM_NORMAL")
    return DM_NORMAL;
  if (str == "DM_VIDEO_FAST")
    return DM_VIDEO_FAST;
  return DM_NORMAL;
}

std::string MarkerDetector::Params::toString(CornerRefinementMethod dm)
{
  switch (dm)
  {
    case CORNER_LINES:
      return "CORNER_LINES";
    case CORNER_SUBPIX:
      return "CORNER_SUBPIX";
    case CORNER_NONE:
      return "CORNER_NONE";
  };
  return "CORNER_SUBPIX";
}
CornerRefinementMethod MarkerDetector::Params::getCornerRefinementMethodFromString(const std::string &str)
{
  if (str == "CORNER_LINES")
    return CORNER_LINES;
  if (str == "CORNER_SUBPIX")
    return CORNER_SUBPIX;
  if (str == "CORNER_NONE")
    return CORNER_NONE;
  return CORNER_SUBPIX;
}
std::string MarkerDetector::Params::toString(MarkerDetector::ThresMethod dm)
{
  switch (dm)
  {
    case THRES_ADAPTIVE:
      return "THRES_ADAPTIVE";
    case THRES_AUTO_FIXED:
      return "THRES_AUTO_FIXED";
  };
  return "THRES_ADAPTIVE";
}
MarkerDetector::ThresMethod MarkerDetector::Params::getCornerThresMethodFromString(const std::string &str)
{
  if (str == "THRES_ADAPTIVE")
    return THRES_ADAPTIVE;
  if (str == "THRES_AUTO_FIXED")
    return THRES_AUTO_FIXED;
  return THRES_ADAPTIVE;
}
void MarkerDetector::Params::setThresholdMethod(MarkerDetector::ThresMethod method,
                                                int thresHold, int wsize, int wsize_range)
{
  AdaptiveThresWindowSize = wsize;
  thresMethod = method;
  if (thresHold == -1)
  {
    if (method == MarkerDetector::THRES_AUTO_FIXED)
      ThresHold = 100;
    else
      ThresHold = 7;
  }
  else
    ThresHold = thresHold;
  AdaptiveThresWindowSize_range = wsize_range;
}
void MarkerDetector::Params::setDetectionMode(DetectionMode dm, float minMarkerSize)
{
  detectMode = dm;
  minSize = minMarkerSize;
  if (detectMode == DM_NORMAL)
  {
    setAutoSizeSpeedUp(false);
    setThresholdMethod(THRES_ADAPTIVE);
  }
  else if (detectMode == DM_FAST)
  {
    setAutoSizeSpeedUp(false);
    setThresholdMethod(THRES_AUTO_FIXED);
  }
  else if (detectMode == DM_VIDEO_FAST)
  {
    setThresholdMethod(THRES_AUTO_FIXED);
    setAutoSizeSpeedUp(true, 0.3);
  }
}
void MarkerDetector::Params::setCornerRefinementMethod(CornerRefinementMethod method)
{
  cornerRefinementM = method;
  if (method != CORNER_SUBPIX)
    minSize = 0;
}
void MarkerDetector::Params::_toStream(const std::string &strg, std::ostream &str)
{
  uint32_t s = strg.size();
  str.write((char *)&s, sizeof(s));
  str.write(strg.c_str(), strg.size());
}
void MarkerDetector::Params::_fromStream(std::string &strg, std::istream &str)
{
  uint32_t s;
  str.read((char *)&s, sizeof(s));
  strg.resize(s);
  str.read(&strg[0], strg.size());
}

void MarkerDetector::cornerUpsample(std::vector<std::vector<cv::Point2f> > &corners,
                                    cv::Size lowResImageSize)
{
  _impl->cornerUpsample(corners, lowResImageSize);
}



};  // namespace aruco
