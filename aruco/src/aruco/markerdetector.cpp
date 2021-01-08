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

#include "markerdetector.h"
#include "cameraparameters.h"
#include "markerlabeler.h"
#include "timers.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <valarray>
#include <chrono>
#include <thread>
#include "debug.h"

#ifdef _DEBUG
#include <opencv2/highgui/highgui.hpp>
#endif

namespace aruco
{

/**
 *
 *
 *
 *
 */
MarkerDetector::MarkerDetector()
{
  markerIdDetector = aruco::MarkerLabeler::create(Dictionary::ALL_DICTS);
  setDetectionMode(DM_NORMAL);
}

/**
 *
 *
 *
 *
 */
MarkerDetector::MarkerDetector(int dict_type, float error_correction_rate)
{
  setDictionary(dict_type, error_correction_rate);
  setDetectionMode(DM_NORMAL);
}

/**
 *
 *
 *
 *
 */
MarkerDetector::MarkerDetector(std::string dict_type, float error_correction_rate)
{
  setDictionary(dict_type, error_correction_rate);
  setDetectionMode(DM_NORMAL);

}

/**
 *
 *
 *
 *
 */
MarkerDetector::~MarkerDetector()
{
}

/**
 *
 *
 *
 *
 */
void MarkerDetector::setDetectionMode(DetectionMode dm, float minMarkerSize)
{
  _detectMode = dm;
  _params.minSize = minMarkerSize;
  if (_detectMode == DM_NORMAL)
  {
    _params.setAutoSizeSpeedUp(false);
//    _params.setThresholdMethod(THRES_AUTO_FIXED);
//    _params.NAttemptsAutoThresFix=30;
    _params.setThresholdMethod(THRES_ADAPTIVE);
  }
  else if (_detectMode == DM_FAST)
  {
    _params.setAutoSizeSpeedUp(false);
    _params.setThresholdMethod(THRES_AUTO_FIXED);
  }
  else if (_detectMode == DM_VIDEO_FAST)
  {
    _params.setThresholdMethod(THRES_AUTO_FIXED);
    _params.setAutoSizeSpeedUp(true, 0.3);
  }
}

DetectionMode MarkerDetector::getDetectionMode()
{
  return _detectMode;
}

/**
 *
 *
 *
 *
 */
std::vector<aruco::Marker> MarkerDetector::detect(const cv::Mat& input)
{
  std::vector<Marker> detectedMarkers;
  detect(input, detectedMarkers);
  return detectedMarkers;
}

std::vector<aruco::Marker> MarkerDetector::detect(const cv::Mat& input, const CameraParameters& camParams,
                                                  float markerSizeMeters, bool setYPerperdicular, bool correctFisheye)
{
  std::vector<Marker> detectedMarkers;
  detect(input, detectedMarkers, camParams, markerSizeMeters, setYPerperdicular, correctFisheye);
  return detectedMarkers;
}

/**
 *
 *
 *
 *
 */
void MarkerDetector::detect(const cv::Mat& input, std::vector<Marker>& detectedMarkers, CameraParameters camParams,
                            float markerSizeMeters, bool setYPerpendicular, bool correctFisheye)
{
  if (camParams.CamSize != input.size() && camParams.isValid() && markerSizeMeters > 0)
  {
    // must resize camera parameters if we want to compute properly marker poses
    CameraParameters cp_aux = camParams;
    cp_aux.resize(input.size());
    detect(input, detectedMarkers, cp_aux.CameraMatrix, cp_aux.Distorsion, cp_aux.ExtrinsicMatrix, markerSizeMeters, setYPerpendicular, correctFisheye);
  }
  else
  {
    detect(input, detectedMarkers, camParams.CameraMatrix, camParams.Distorsion, camParams.ExtrinsicMatrix, markerSizeMeters, setYPerpendicular, correctFisheye);
  }
}

int MarkerDetector::getMarkerWarpSize()
{

  auto bis = markerIdDetector->getBestInputSize();
  if (bis != -1)
    return bis;

  int ndiv = markerIdDetector->getNSubdivisions();
  if (ndiv == -1)
    ndiv = 7; // set any possible value (it is used for non dictionary based labelers)
  return _params._markerWarpPixSize * ndiv; // this is the minimum size that the smallest marker will have
}

void MarkerDetector::buildPyramid(std::vector<cv::Mat> &ImagePyramid, const cv::Mat &grey_img, int minSize)
{
  // determine number of pyramid images
  int npyrimg = 1;
  cv::Size imgpsize = grey_img.size();
  while (imgpsize.width > minSize)
  {
    imgpsize = cv::Size(imgpsize.width / _params.pyrfactor, imgpsize.height / _params.pyrfactor);
    npyrimg++;
  }

  ImagePyramid.resize(npyrimg);
  imagePyramid[0] = grey_img;

  // now, create pyramid images
  imgpsize = grey_img.size();
  for (int i = 1; i < npyrimg; i++)
  {
    cv::Size nsize(ImagePyramid[i - 1].cols / _params.pyrfactor, ImagePyramid[i - 1].rows / _params.pyrfactor);
    cv::resize(ImagePyramid[i - 1], ImagePyramid[i], nsize);
  }
}

/**
 *
 */
std::vector<MarkerDetector::MarkerCandidate> MarkerDetector::thresholdAndDetectRectangles(const cv::Mat & input,
                                                                                          int thres_param1,
                                                                                          int thres_param2, bool erode,
                                                                                          cv::Mat &auxThresImage)
{
  // ensure that _thresParam1 % 2 == 1
  ScopedTimerEvents tev("hafc " + std::to_string(thres_param1));
  if (thres_param1 < 3)
    thres_param1 = 3;
  else if (((int)thres_param1) % 2 != 1)
    thres_param1 = (int)(thres_param1 + 1);

  cv::Mat auxImage;
  if (!erode)
    auxImage = auxThresImage;
  if (_params._thresMethod == THRES_AUTO_FIXED)
  {
    cv::threshold(input, auxImage, static_cast<int>(thres_param2), 255, cv::THRESH_BINARY_INV);
  }
  else
    cv::adaptiveThreshold(input, auxImage, 255., cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV,
                          static_cast<int>(thres_param1), static_cast<int>(thres_param2));
  tev.add("thres");

  if (erode)
  {
    cv::erode(auxImage, auxThresImage, getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3), cv::Point(1, 1)));
    tev.add("erode");
  }

  std::vector<MarkerCandidate> MarkerCanditates;

  // calculate the min_max contour sizes
  int thisImageMinSize = int(3.5 * float(_params.lowResMarkerSize));

  // if image is eroded, minSize must be adapted
  std::vector<cv::Vec4i> hierarchy;
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(auxThresImage, contours, cv::noArray(), cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
  tev.add("find-cont");
  std::vector<cv::Point> approxCurve;

//#define _aruco_debug_detectrectangles
#ifdef _aruco_debug_detectrectangles
  cv::Mat simage;
  cv::cvtColor(input,simage,cv::COLOR_GRAY2BGR);
#endif

  // for each contour, analyze if it is a parallelepiped likely to be the marker
  for (unsigned int i = 0; i < contours.size(); i++)
  {

#ifdef _aruco_debug_detectrectangles
    drawContour(simage, contours[i], Scalar(125, 125, 255) );
#endif

    // check it is a possible element by first checking that is is large enough
    if (thisImageMinSize < int(contours[i].size()))
    {
      // can approximate to a convex rect?
      cv::approxPolyDP(contours[i], approxCurve, double(contours[i].size()) * 0.05, true);

      if (approxCurve.size() == 4 && cv::isContourConvex(approxCurve))
      {

#ifdef _aruco_debug_detectrectangles
        drawApproxCurve(simage, approxCurve, Scalar(255, 0, 255),1);
#endif

        // ensure that the distance between consecutive points is large enough
        float minDist = std::numeric_limits<float>::max();
        for (int j = 0; j < 4; j++)
        {
          float d = cv::norm(approxCurve[j] - approxCurve[(j + 1) % 4]);
          if (d < minDist)
            minDist = d;
        }

        // add the points
        MarkerCanditates.push_back(MarkerCandidate());
        for (int j = 0; j < 4; j++)
          MarkerCanditates.back().push_back(
              cv::Point2f(static_cast<float>(approxCurve[j].x), static_cast<float>(approxCurve[j].y)));

        // now, if it is eroded, must enlarge 1 bit the corners to go to the real location
        if (erode)
        {
          // for each opposite pair, take the line joining them and move one pixel apart
          // ideally, Bresenham's algorithm should be used
          enlargeMarkerCandidate(MarkerCanditates.back(), 1);
        }

#ifdef _aruco_debug_detectrectangles
        MarkerCanditates.back().draw(simage,Scalar(255, 255, 0),1,false);
#endif

      }
    }
  }

#ifdef _aruco_debug_detectrectangles
  cv::imshow("contours",simage);
#endif

  return MarkerCanditates;
}

void MarkerDetector::thresholdAndDetectRectangles_thread()
{
  while (true)
  {
//    std::stringstream sstr;
//    sstr << "thread-" << std::this_thread::get_id() << " "
//        << std::chrono::high_resolution_clock::now().time_since_epoch().count();
//    ScopedTimerEvents tev(sstr.str());
    bool erode = false;
    auto tad = _tasks.pop();
//    tev.add("pop");
    if (tad.task == EXIT_TASK)
      return;
    else if (tad.task == ERODE_TASK)
      erode = true;
    _vcandidates[tad.outIdx] = thresholdAndDetectRectangles(_thres_Images[tad.inIdx], tad.param1, tad.param2, erode,
                                                            _thres_Images[tad.outIdx]);
//    tev.add("thres param: "+to_string(tad.param1));
  }
}

std::vector<aruco::MarkerDetector::MarkerCandidate> MarkerDetector::thresholdAndDetectRectangles(const cv::Mat &image)
{
  // compute the different values of param1
  std::vector<int> p1_values;
  for (int i =
      static_cast<int>(std::max(3., _params._AdaptiveThresWindowSize - 2. * _params._AdaptiveThresWindowSize_range));
      i <= _params._AdaptiveThresWindowSize + 2 * _params._AdaptiveThresWindowSize_range; i += 2)
    p1_values.push_back(i);

  std::size_t nimages = p1_values.size();
  _vcandidates.resize(nimages);
  _thres_Images.resize(nimages + 1);
  _thres_Images.back() = image; // add at the end the original image

  // first, thresholded images
  ThresAndDetectRectTASK tad;
  std::vector<ThresAndDetectRectTASK> vtad;

  ThreadTasks task = THRESHOLD_TASK;
  if (_params.enclosedMarker)
    task = ERODE_TASK;
  for (std::size_t i = 0; i < p1_values.size(); i++)
  {
    tad.inIdx = int(_thres_Images.size() - 1);
    tad.param1 = p1_values[i];
    tad.param2 = _params._ThresHold;
    tad.outIdx = i;
    tad.task = task;
    _tasks.push(tad);
    vtad.push_back(tad);
  }

  // reserve images
  for (std::size_t i = 0; i < nimages; i++)
    _thres_Images[i].create(image.size(), CV_8UC1);

  // how many threads will be used?
  int nthreads = 0;
  if (_params.maxThreads <= 0) // if allowed to use all , take max()-1, since the buildpyramid must be working at this moment
    nthreads = std::thread::hardware_concurrency() - 1;
  else
    nthreads = std::max(1, _params.maxThreads - 1);

  tad.task = EXIT_TASK;
  for (int i = 0; i < nthreads; i++)
    _tasks.push(tad);

  if (nthreads == 1)
  {
    // no threads
    ScopeTimer Timer("non-parallel");
    thresholdAndDetectRectangles_thread();
  }
  else
  {
    // parallell mode
    // add the final task END
    ScopeTimer Timer("parallel");

    // run the tasks (in parallel)
    std::vector<std::thread> threads;
    for (int i = 0; i < nthreads; i++)
      threads.push_back(std::thread(&MarkerDetector::thresholdAndDetectRectangles_thread, this));

    // wait for them to finish
    for (auto &th : threads)
      th.join();
  }

  std::vector<MarkerCandidate> joined;
  joinVectors(_vcandidates, joined);
  return joined;
}

std::vector<MarkerDetector::MarkerCandidate> MarkerDetector::prefilterCandidates(
    std::vector<aruco::MarkerDetector::MarkerCandidate> &MarkerCanditates, cv::Size imgSize)
{
  /***********************************************************************************************
   * CANDIDATE PREFILTERING- Merge and Remove candidates so that only reliable ones are returned *
   ***********************************************************************************************/

  // sort the points in anti-clockwise order
  std::valarray<bool> swapped(false, MarkerCanditates.size()); // used later
  for (unsigned int i = 0; i < MarkerCanditates.size(); i++)
  {
    // trace a line between the first and second point.
    // if the third point is at the right side, then the points are anti-clockwise
    double dx1 = MarkerCanditates[i][1].x - MarkerCanditates[i][0].x;
    double dy1 = MarkerCanditates[i][1].y - MarkerCanditates[i][0].y;
    double dx2 = MarkerCanditates[i][2].x - MarkerCanditates[i][0].x;
    double dy2 = MarkerCanditates[i][2].y - MarkerCanditates[i][0].y;
    double o = (dx1 * dy2) - (dy1 * dx2);

    if (o < 0.0)
    {
      // if the third point is in the left side, then sort in anti-clockwise order
      std::swap(MarkerCanditates[i][1], MarkerCanditates[i][3]);
      swapped[i] = true;
    }
  }

  // remove these elements which corners are too close to each other
  // first detect candidates to be removed
  std::vector<std::pair<int, int>> TooNearCandidates;
  for (unsigned int i = 0; i < MarkerCanditates.size(); i++)
  {
    // calculate the average distance of each corner to the nearest corner of the other marker candidate
    for (unsigned int j = i + 1; j < MarkerCanditates.size(); j++)
    {
      std::valarray<float> vdist(4);
      for (int c = 0; c < 4; c++)
        vdist[c] = cv::norm(MarkerCanditates[i][c] - MarkerCanditates[j][c]);

      // if distance is too small
      if (vdist[0] < 6 && vdist[1] < 6 && vdist[2] < 6 && vdist[3] < 6)
        TooNearCandidates.push_back(std::pair<int, int>(i, j));
    }
  }

  // mark for removal the element of  the pair with smaller perimeter
  std::vector<bool> toRemove(MarkerCanditates.size(), false);
  for (unsigned int i = 0; i < TooNearCandidates.size(); i++)
  {
    if (perimeter(MarkerCanditates[TooNearCandidates[i].first])
        > perimeter(MarkerCanditates[TooNearCandidates[i].second]))
      toRemove[TooNearCandidates[i].second] = true;
    else
      toRemove[TooNearCandidates[i].first] = true;
  }

  // find these too near borders and remove them
  // remove markers with corners too near the image limits
  int borderDistThresX = static_cast<int>(_params.borderDistThres * float(imgSize.width));
  int borderDistThresY = static_cast<int>(_params.borderDistThres * float(imgSize.height));
  for (std::size_t i = 0; i < MarkerCanditates.size(); i++)
  {
    // delete if any of the corners is too near image border
    for (std::size_t c = 0; c < MarkerCanditates[i].size(); c++)
    {
      if (MarkerCanditates[i][c].x < borderDistThresX || MarkerCanditates[i][c].y < borderDistThresY
          || MarkerCanditates[i][c].x > imgSize.width - borderDistThresX
          || MarkerCanditates[i][c].y > imgSize.height - borderDistThresY)
      {
        toRemove[i] = true;
      }
    }
  }

  // move to output only valid ones
  std::vector<MarkerCandidate> finalCandidates;
  finalCandidates.reserve(MarkerCanditates.size());
  for (std::size_t i = 0; i < MarkerCanditates.size(); i++)
    if (!toRemove[i])
      finalCandidates.push_back(MarkerCanditates[i]);
  return finalCandidates;
}

void addToImageHist(cv::Mat &im, std::vector<float>&hist)
{
  for (int y = 0; y < im.rows; y++)
  {
    uchar*ptr = im.ptr<uchar>(y);
    for (int x = 0; x < im.cols; x++)
      hist[ptr[x]]++;
  }
}

int Otsu(std::vector<float> &hist)
{
  float sum = 0, invsum;
  for (auto c : hist)
    sum += c;
  invsum = 1. / sum;
  for (auto &c : hist)
    c *= invsum;

  float maxVar = 0;
  int bestT = -1;
  for (int t = 1; t < 256; t++)
  {
    float w0 = 0, w1 = 0, mean0 = 0, mean1 = 0;
    for (int v = 0; v < t; v++)
    {
      w0 += hist[v];
      mean0 += float(v) * hist[v];
    }
    for (int v = t; v < 256; v++)
    {
      w1 += hist[v];
      mean1 += hist[v] * float(v);
    }
    if (w0 > 1e-4 && w1 > 1e-4)
    {
      mean0 /= w0;
      mean1 /= w1;
      float var = w0 * w1 * (mean0 - mean1) * (mean0 - mean1);
//      std::cout << t << " : " << var << "|" << w0 << " " << w1 << " " << mean0 << " " << mean1 << std::endl;
      if (var > maxVar)
      {
        maxVar = var;
        bestT = t;
      }
    }
  }

  return bestT;
}

/***********************************************
 * Main detection function. Performs all steps *
 ***********************************************/
void MarkerDetector::detect(const cv::Mat& input, std::vector<Marker>& detectedMarkers, cv::Mat camMatrix,
                            cv::Mat distCoeff, cv::Mat extrinsics, float markerSizeMeters, bool setYPerpendicular, bool correctFisheye)
{
  // clear input data
  detectedMarkers.clear();
  _vcandidates.clear();
  _candidates.clear();
  ScopedTimerEvents Timer("detect");

  // it must be a 3 channel image
  if (input.type() == CV_8UC3)
    cv::cvtColor(input, grey, cv::COLOR_BGR2GRAY);
//  convertToGray(input, grey);
  else
    grey = input;
  Timer.add("ConvertGrey");

  /*****************************************************************
   * CREATE LOW RESOLUTION IMAGE IN WHICH MARKERS WILL BE DETECTED *
   *****************************************************************/

  float ResizeFactor = 1;

  // use the minimum and markerWarpSize to determine the optimal image size on which to do rectangle detection
  cv::Mat imgToBeThresHolded;
  cv::Size maxImageSize = grey.size();
  auto minpixsize = getMinMarkerSizePix(input.size()); // min pixel size of the marker in the original image
  if (_params.lowResMarkerSize < minpixsize)
  {
    ResizeFactor = float(_params.lowResMarkerSize) / float(minpixsize);
    if (ResizeFactor < 0.9)
    {
      // do not waste time if smaller than this
      _debug_msg("Scale factor=" << ResizeFactor, 1);
      maxImageSize.width = float(grey.cols) * ResizeFactor + 0.5;
      maxImageSize.height = float(grey.rows) * ResizeFactor + 0.5;
      if (maxImageSize.width % 2 != 0)
        maxImageSize.width++;
      if (maxImageSize.height % 2 != 0)
        maxImageSize.height++;
      cv::resize(grey, imgToBeThresHolded, maxImageSize, 0, 0, cv::INTER_NEAREST);
//      cv::resize(grey, imgToBeThresHolded, maxImageSize, 0, 0, cv::INTER_LINEAR);
    }
  }

  if (imgToBeThresHolded.empty()) // if not set in previous step, add original now
    imgToBeThresHolded = grey;

  Timer.add("CreateImageToTheshold");
  bool needPyramid = true; // ResizeFactor < 1/_params.pyrfactor; // only use pyramid if working on a big image.
  std::thread buildPyramidThread;
  if (needPyramid)
  {
    if (_params.maxThreads > 1)
      buildPyramidThread = std::thread([&]
      {
        buildPyramid(imagePyramid, grey, 2 * getMarkerWarpSize());
      });
    else
      buildPyramid(imagePyramid, grey, 2 * getMarkerWarpSize());
    Timer.add("BuildPyramid");
  }
  else
  {
    imagePyramid.resize(1);
    imagePyramid[0] = grey;
  }

  int nAttemptsAutoFix = 0;
  bool keepLookingFor = false;
  std::vector<float> hist(256, 0);
  do
  {
    /**************************************************
     * THRESHOLD IMAGES AND DETECT INITIAL RECTANGLES *
     **************************************************/
    std::vector<MarkerCandidate> MarkerCanditates;
    MarkerCanditates = thresholdAndDetectRectangles(imgToBeThresHolded);
    thres = _thres_Images[0];

    _debug_exec(10,
        {
          // only executes when compiled in DEBUG mode if debug level is at least 10
          // show the thresholded images
          for (std::size_t i = 0; i < _thres_Images.size(); i++)
          {
            std::stringstream sstr; sstr << "thres-" << i;
            cv::namedWindow(sstr.str(),cv::WINDOW_NORMAL);
            cv::imshow(sstr.str(),_thres_Images[i]);
          }
        }
    );

    Timer.add("Threshold and Detect rectangles");

    // prefilter candidates
    _debug_exec(10,
        // only executes when compiled in DEBUG mode if debug level is at least 10
        // show the thresholded images
        cv::Mat imrect;
        cv::cvtColor(imgToBeThresHolded, imrect, cv::COLOR_GRAY2BGR);
        for (auto m : MarkerCanditates)
          m.draw(imrect, cv::Scalar(0, 245, 0));
        cv::imshow("rect-nofiltered", imrect);
    );

    MarkerCanditates = prefilterCandidates(MarkerCanditates, imgToBeThresHolded.size());

    Timer.add("prefilterCandidates");

    _debug_exec(10,
        // only executes when compiled in DEBUG mode if debug level is at least 10
        // show the thresholded images
        cv::Mat imrect;
        cv::cvtColor(imgToBeThresHolded, imrect, cv::COLOR_GRAY2BGR);
        for (auto m : MarkerCanditates)
          m.draw(imrect, cv::Scalar(0, 245, 0));
        cv::imshow("rect-filtered", imrect);
    );

    // before going on, make sure the piramid is built
    if (buildPyramidThread.joinable())
      buildPyramidThread.join();

    /************************************************************************
     * CANDIDATE CLASSIFICATION: Decide which candidates are really markers *
     ************************************************************************/
    auto markerWarpSize = getMarkerWarpSize();

    detectedMarkers.clear();
    _candidates.clear();
    for (auto &b : hist)
      b = 0;
    float desiredarea = std::pow(static_cast<float>(markerWarpSize), 2.f);
    for (std::size_t i = 0; i < MarkerCanditates.size(); i++)
    {
      // Find projective homography
      cv::Mat canonicalMarker, canonicalMarkerAux;

      cv::Mat inToWarp = imgToBeThresHolded;
      MarkerCandidate points2d_pyr = MarkerCanditates[i];
      if (needPyramid)
      {
        // warping is one of the most time consuming operations, especially when the region is large.
        // To reduce computing time, let us find in the image pyramid, the best configuration to save time

        // indicates how much bigger observation is wrt to desired patch
        std::size_t imgPyrIdx = 0;
        for (std::size_t p = 1; p < imagePyramid.size(); p++)
        {
          if (MarkerCanditates[i].getArea() / std::pow(4, p) >= desiredarea)
            imgPyrIdx = p;
          else
            break;
        }
        inToWarp = imagePyramid[imgPyrIdx];

        // move points to the image level p
        float ratio = float(inToWarp.cols) / float(imgToBeThresHolded.cols);
        for (auto& p : points2d_pyr)
          p *= ratio; // 1. / std::pow(2, imgPyrIdx);

      }

      warp(inToWarp, canonicalMarker, cv::Size(markerWarpSize, markerWarpSize), points2d_pyr);
      int id, nRotations;
      double min, Max;
      cv::minMaxIdx(canonicalMarker, &min, &Max);
      canonicalMarker.copyTo(canonicalMarkerAux);
      std::string additionalInfo;

      _debug_exec(10,
          // only executes when compiled in DEBUG mode if debug level is at least 10
          // show the thresholded images
          std::stringstream sstr; sstr << "test-" << i;
          std::cout << "test" << i << std::endl;
          cv::namedWindow(sstr.str(), cv::WINDOW_NORMAL);
          cv::imshow(sstr.str(), canonicalMarkerAux);
          cv::waitKey(0);
      );

      if (markerIdDetector->detect(canonicalMarkerAux, id, nRotations, additionalInfo))
      {
        detectedMarkers.push_back(MarkerCanditates[i]);
        detectedMarkers.back().id = id;
        detectedMarkers.back().dict_info = additionalInfo;

        // sort the points so that they are always in the same order no matter the camera orientation
        std::rotate(detectedMarkers.back().begin(), detectedMarkers.back().begin() + 4 - nRotations,
                    detectedMarkers.back().end());
        _debug_exec(10,
            // only executes when compiled in DEBUG mode if debug level is at least 10
            // show the thresholded images
            std::stringstream sstr; sstr << "can-" << detectedMarkers.back().id;
            cv::namedWindow(sstr.str(), cv::WINDOW_NORMAL);
            cv::imshow(sstr.str(), canonicalMarker);
            std::cout << "ID=" << id << " " << detectedMarkers.back() << std::endl;
        );
        if (_params._thresMethod == THRES_AUTO_FIXED)
          addToImageHist(canonicalMarker, hist);
        }
        else
          _candidates.push_back(MarkerCanditates[i]);
        }
        Timer.add("Marker classification");
        if (detectedMarkers.size() == 0 && _params._thresMethod == THRES_AUTO_FIXED
            && ++nAttemptsAutoFix < _params.NAttemptsAutoThresFix)
        {
          _params._ThresHold = 10 + rand() % 230;
          keepLookingFor = true;
        }
        else
          keepLookingFor = false;
      } while (keepLookingFor);

      if (_params._thresMethod == THRES_AUTO_FIXED)
      {
        int newThres = Otsu(hist);
        if (newThres > 0)
          _params._ThresHold = float(newThres);
      }

#ifdef debug_lines
      cv::imshow("image-lines",image);
      cv::waitKey(10);
#endif

      // now, move the points to the original image (upsample corners)
      if (input.cols != imgToBeThresHolded.cols)
      {
        cornerUpsample(detectedMarkers, imgToBeThresHolded.size());
        Timer.add("Corner Upsample");
      }

      /*********************************
       * CORNER REFINEMENT IF REQUIRED *
       *********************************/
      // refine the corner location if enclosed markers and we did not do it via upsampling
      if (detectedMarkers.size() > 0 /* &&_params.enclosedMarker */ && input.size() == imgToBeThresHolded.size())
      {
        int halfwsize = 2 * float(input.cols) / float(imgToBeThresHolded.cols) + 0.5;
        std::vector<cv::Point2f> Corners;
        for (unsigned int i = 0; i < detectedMarkers.size(); i++)
          for (int c = 0; c < 4; c++)
            Corners.push_back(detectedMarkers[i][c]);
        cornerSubPix(grey, Corners, cv::Size(halfwsize, halfwsize), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 12, 0.005));
        // copy back
        for (unsigned int i = 0; i < detectedMarkers.size(); i++)
          for (int c = 0; c < 4; c++)
            detectedMarkers[i][c] = Corners[i * 4 + c];
        Timer.add("Corner Refinement");
      }

      /*************************
       * REMOVAL OF DUPLICATED *
       *************************/

      // sort by id
      std::sort(detectedMarkers.begin(), detectedMarkers.end());

      // there might be still the case that a marker is detected twice because of the double border indicated earlier,
      // detect and remove these cases
      std::vector<bool> toRemove(detectedMarkers.size(), false);

      for (int i = 0; i < int(detectedMarkers.size()) - 1; i++)
      {
        for (int j = i + 1; j < int(detectedMarkers.size()) && !toRemove[i]; j++)
        {
          if (detectedMarkers[i].id == detectedMarkers[j].id
              && detectedMarkers[i].dict_info == detectedMarkers[j].dict_info)
          {
            // deletes the one with smaller perimeter
            if (perimeter(detectedMarkers[i]) < perimeter(detectedMarkers[j]))
              toRemove[i] = true;
            else
              toRemove[j] = true;
          }
        }
      }

      removeElements(detectedMarkers, toRemove);

      /*************************
      * MARKER POSE ESTIMATION *
      **************************/
      // detect the position of detected markers if desired
      if (camMatrix.rows != 0 && markerSizeMeters > 0)
      {
        for (unsigned int i = 0; i < detectedMarkers.size(); i++)
          detectedMarkers[i].calculateExtrinsics(markerSizeMeters, camMatrix, distCoeff, extrinsics, setYPerpendicular, correctFisheye);
        Timer.add("Pose Estimation");
      }

      // compute _markerMinSize
      float mlength = std::numeric_limits<float>::max();
      for (const auto &marker : detectedMarkers)
      {
        float l = 0;
        for (int c = 0; c < 4; c++)
          l += cv::norm(marker[c] - marker[(c + 1) % 4]);
        if (mlength > l)
          mlength = l;
      }

      float markerMinSize;
      if (mlength != std::numeric_limits<float>::max())
        markerMinSize = mlength / (4 * std::max(input.cols, input.rows));
      else
        markerMinSize = 0;
      if (_params._autoSize)
      {
        _params.minSize = markerMinSize * (1 - _params._ts);
      }
}

/**
 * Expands the corners of the candidate to reach the real locations
 * Used in eroded images
 */
void MarkerDetector::enlargeMarkerCandidate(MarkerCandidate &cand, int fact)
{
  for (int j = 0; j < 2; j++)
  {
    auto startp = j;
    auto endp = (j + 2) % 4;

    // sort so that the nearest to x is first
    if (cand[startp].x > cand[endp].x)
    {
      std::swap(startp, endp);
    }
    const float _180 = 3.14159f;

    const float _22 = 3.14159 / 8.f;
    const float _3_22 = 3. * 3.14159f / 8.f;
    const float _5_22 = 5.f * 3.14159f / 8.f;
    const float _7_22 = 7.f * 3.14159f / 8.f;

    int incx = 0, incy = 0;

    // compute the angle
    auto v1 = cand[endp] - cand[startp];
    float angle = std::atan2(v1.y, v1.x);
    if (_22 < angle && angle < 3 * _22)
    {
      // a
      incx = incy = fact;
    }
    else if (-_22 < angle && angle < _22)
    {
      // b
      incx = fact;
      incy = 0;
    }
    else if (-_3_22 < angle && angle < -_22)
    {
      // c
      incx = fact;
      incy = -fact;
    }
    else if (-_5_22 < angle && angle < -_3_22)
    {
      // D
      incx = 0;
      incy = -fact;
    }
    else if (-_7_22 < angle && angle < -_5_22)
    {
      // E
      incx = -fact;
      incy = -fact;
    }
    else if ((-_180 < angle && angle < -_7_22) || (_7_22 < angle && angle < _180))
    {
      // f
      incx = -fact;
      incy = 0;
    }
    else if ((_5_22 < angle && angle < _7_22))
    {
      // g
      incx = -fact;
      incy = fact;
    }
    else if ((_3_22 < angle && angle < _5_22))
    {
      // h
      incx = fact;
      incy = fact;
    }
    cand[endp].x += incx;
    cand[endp].y += incy;
    cand[startp].x -= incx;
    cand[startp].y -= incy;
  }
}

int MarkerDetector::getMinMarkerSizePix(cv::Size orginput_imageSize) const
{
  if (_params.minSize == -1 && _params.minSize_pix == -1)
    return 0;

  // calculate the min_max contour sizes
  int maxDim = std::max(orginput_imageSize.width, orginput_imageSize.height);
  int minSize = 0;
  if (_params.minSize != -1)
    minSize = static_cast<float>(_params.minSize) * static_cast<float>(maxDim);
  if (_params.minSize_pix != -1)
    minSize = std::min(_params.minSize_pix, minSize);

  return minSize;
}

/**
 *
 *
 *
 *
 */
bool MarkerDetector::warp(cv::Mat& in, cv::Mat& out, cv::Size size, std::vector<cv::Point2f> points)
{
  if (points.size() != 4)
    throw cv::Exception(9001, "point.size() != 4", "MarkerDetector::warp", __FILE__, __LINE__);

  // obtain the perspective transform
  cv::Point2f pointsRes[4], pointsIn[4];
  for (int i = 0; i < 4; i++)
    pointsIn[i] = points[i];
  pointsRes[0] = (cv::Point2f(0, 0));
  pointsRes[1] = cv::Point2f(static_cast<float>(size.width - 1), 0.f);
  pointsRes[2] = cv::Point2f(static_cast<float>(size.width - 1), static_cast<float>(size.height - 1));
  pointsRes[3] = cv::Point2f(0.f, static_cast<float>(size.height - 1));
  cv::Mat M = getPerspectiveTransform(pointsIn, pointsRes);
  cv::warpPerspective(in, out, M, size, cv::INTER_LINEAR);
//  cv::warpPerspective(in, out, M, size, cv::INTER_NEAREST);
  return true;
}

/**
 *
 *
 *
 *
 */
int MarkerDetector::perimeter(const std::vector<cv::Point2f>& a)
{
  int sum = 0;
  for (unsigned int i = 0; i < a.size(); i++)
  {
    int i2 = (i + 1) % a.size();
    sum += static_cast<int>(std::sqrt(
        (a[i].x - a[i2].x) * (a[i].x - a[i2].x) + (a[i].y - a[i2].y) * (a[i].y - a[i2].y)));
  }

  return sum;
}

/**
 */
void MarkerDetector::interpolate2Dline(const std::vector<cv::Point2f>& inPoints, cv::Point3f& outLine)
{
  float minX, maxX, minY, maxY;
  minX = maxX = inPoints[0].x;
  minY = maxY = inPoints[0].y;

  for (unsigned int i = 1; i < inPoints.size(); i++)
  {
    if (inPoints[i].x < minX)
      minX = inPoints[i].x;
    if (inPoints[i].x > maxX)
      maxX = inPoints[i].x;
    if (inPoints[i].y < minY)
      minY = inPoints[i].y;
    if (inPoints[i].y > maxY)
      maxY = inPoints[i].y;
  }

  // create matrices of equation system
  const int pointsCount = static_cast<int>(inPoints.size());
  cv::Mat A(pointsCount, 2, CV_32FC1, cv::Scalar(0));
  cv::Mat B(pointsCount, 1, CV_32FC1, cv::Scalar(0));
  cv::Mat X;

  if (maxX - minX > maxY - minY)
  {
    // Ax + C = y
    for (int i = 0; i < pointsCount; i++)
    {
      A.at<float>(i, 0) = inPoints[i].x;
      A.at<float>(i, 1) = 1.;
      B.at<float>(i, 0) = inPoints[i].y;
    }

    // solve system
    solve(A, B, X, cv::DECOMP_SVD);
    // return Ax + By + C
    outLine = cv::Point3f(X.at<float>(0, 0), -1., X.at<float>(1, 0));
  }
  else
  {
    // By + C = x
    for (int i = 0; i < pointsCount; i++)
    {
      A.at<float>(i, 0) = inPoints[i].y;
      A.at<float>(i, 1) = 1.;
      B.at<float>(i, 0) = inPoints[i].x;
    }

    // solve system
    solve(A, B, X, cv::DECOMP_SVD);
    // return Ax + By + C
    outLine = cv::Point3f(-1., X.at<float>(0, 0), X.at<float>(1, 0));
  }
}

/**
 */
cv::Point2f MarkerDetector::getCrossPoint(const cv::Point3f& line1, const cv::Point3f& line2)
{
  // create matrices of equation system
  cv::Mat A(2, 2, CV_32FC1, cv::Scalar(0));
  cv::Mat B(2, 1, CV_32FC1, cv::Scalar(0));
  cv::Mat X;

  A.at<float>(0, 0) = line1.x;
  A.at<float>(0, 1) = line1.y;
  B.at<float>(0, 0) = -line1.z;

  A.at<float>(1, 0) = line2.x;
  A.at<float>(1, 1) = line2.y;
  B.at<float>(1, 0) = -line2.z;

  // solve system
  solve(A, B, X, cv::DECOMP_SVD);
  return cv::Point2f(X.at<float>(0, 0), X.at<float>(1, 0));
}

void MarkerDetector::cornerUpsample(std::vector<Marker>& MarkerCanditates, cv::Size lowResImageSize)
{
  cornerUpsample_SUBP(MarkerCanditates, lowResImageSize);
}

void MarkerDetector::cornerUpsample_SUBP(std::vector<Marker>& MarkerCanditates, cv::Size lowResImageSize)
{
  if (MarkerCanditates.size() == 0)
    return;
  // first, determine the image in the pyramid nearest to this one
  int startPyrImg = 0;

  for (std::size_t i = 0; i < imagePyramid.size(); i++)
  {
    if (lowResImageSize.width < imagePyramid[i].cols)
      startPyrImg = i;
    else
      break;
  }

//#define _aruco_marker_detector_fast

  cv::Size prevLowResSize = lowResImageSize;
  for (int curpyr = startPyrImg; curpyr >= 0; curpyr--)
  {
    float factor = float(imagePyramid[curpyr].cols) / float(prevLowResSize.width);

    // upsample corner locations
    for (auto &m : MarkerCanditates)
      for (auto &point : m)
      {
        point *= factor;
      }

    int halfwsize = 0.5 + 2.5 * factor;
    std::vector<cv::Point2f> p2d;
    p2d.reserve(MarkerCanditates.size() * 4);
    for (auto &m : MarkerCanditates)
      for (auto &point : m)
      {
        p2d.push_back(point);
      }

    cv::cornerSubPix(imagePyramid[curpyr], p2d, cv::Size(halfwsize, halfwsize), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::MAX_ITER, 4, 0.5));
    int cidx = 0;
    for (auto &m : MarkerCanditates)
      for (auto &point : m)
      {
        point = p2d[cidx++];
      }

    prevLowResSize = imagePyramid[curpyr].size();
  }
}

/**
 *
 *
 *
 *
 */
void MarkerDetector::drawAllContours(cv::Mat input, std::vector<std::vector<cv::Point>>& contours)
{
  drawContours(input, contours, -1, cv::Scalar(255, 0, 255));
}

/**
 *
 *
 *
 *
 */
void MarkerDetector::drawContour(cv::Mat& in, std::vector<cv::Point>& contour, cv::Scalar color)
{
  for (unsigned int i = 0; i < contour.size(); i++)
  {
    cv::rectangle(in, contour[i], contour[i], color);
  }
}

void MarkerDetector::drawApproxCurve(cv::Mat& in, std::vector<cv::Point>& contour, cv::Scalar color, int thickness)
{
  for (unsigned int i = 0; i < contour.size(); i++)
  {
    cv::line(in, contour[i], contour[(i + 1) % contour.size()], color, thickness);
  }
}

/**
 *
 *
 *
 *
 */
void MarkerDetector::draw(cv::Mat out, const std::vector<Marker>& markers)
{
  for (unsigned int i = 0; i < markers.size(); i++)
  {
    cv::line(out, markers[i][0], markers[i][1], cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
    cv::line(out, markers[i][1], markers[i][2], cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
    cv::line(out, markers[i][2], markers[i][3], cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
    cv::line(out, markers[i][3], markers[i][0], cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
  }
}

void MarkerDetector::setMarkerLabeler(cv::Ptr<MarkerLabeler> detector)
{
  markerIdDetector = detector;
}

void MarkerDetector::setDictionary(int dict_type, float error_correction_rate)
{
  markerIdDetector = MarkerLabeler::create((Dictionary::DICT_TYPES)dict_type, error_correction_rate);
}

void MarkerDetector::setDictionary(std::string dict_type, float error_correction_rate)
{
  markerIdDetector = MarkerLabeler::create(dict_type, std::to_string(error_correction_rate));
}

cv::Mat MarkerDetector::getThresholdedImage(std::uint32_t idx)
{
  if (_thres_Images.size() == 0)
    return cv::Mat();
  if (idx >= _thres_Images.size())
    idx = _thres_Images.size() - 1; // last one is the original image
  return _thres_Images[idx];
}

} // namespace aruco
