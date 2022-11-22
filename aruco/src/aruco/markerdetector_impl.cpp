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

#include "markerdetector_impl.h"
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
//#include "picoflann.h"

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
MarkerDetector_Impl::MarkerDetector_Impl()
{
  markerIdDetector = aruco::MarkerLabeler::create(Dictionary::ALL_DICTS);
  setDetectionMode(DM_NORMAL);
}
/************************************
 *
 *
 *
 *
 ************************************/
MarkerDetector_Impl::MarkerDetector_Impl(int dict_type, float error_correction_rate)
{
  setDictionary(dict_type, error_correction_rate);
  setDetectionMode(DM_NORMAL);
}
/************************************
 *
 *
 *
 *
 ************************************/
MarkerDetector_Impl::MarkerDetector_Impl(std::string dict_type, float error_correction_rate)
{
  setDictionary(dict_type, error_correction_rate);
  setDetectionMode(DM_NORMAL);
}
/************************************
 *
 *
 *
 *
 ************************************/

MarkerDetector_Impl::~MarkerDetector_Impl()
{
}

void MarkerDetector_Impl::setParameters(const MarkerDetector::Params &params)
{
  _params = params;
  setDictionary(_params.dictionary, _params.error_correction_rate);
}

/************************************
 *
 *
 *
 *
 ************************************/
void MarkerDetector_Impl::setDetectionMode(DetectionMode dm, float minMarkerSize)
{
  _params.setDetectionMode(dm, minMarkerSize);
}

DetectionMode MarkerDetector_Impl::getDetectionMode()
{
  return _params.detectMode;
}



/************************************
 *
 *
 *
 *
 ************************************/

std::vector<aruco::Marker> MarkerDetector_Impl::detect(const cv::Mat &input)
{
  std::vector<Marker> detectedMarkers;
  detect(input, detectedMarkers);
  return detectedMarkers;
}

std::vector<aruco::Marker> MarkerDetector_Impl::detect(const cv::Mat &input,
                                                       const CameraParameters &camParams,
                                                       float markerSizeMeters,
                                                       bool setYPerperdicular, bool correctFisheye)
{
  std::vector<Marker> detectedMarkers;
  detect(input, detectedMarkers, camParams, markerSizeMeters, setYPerperdicular, correctFisheye);
  return detectedMarkers;
}

/************************************
 *
 *
 *
 *
 ************************************/
void MarkerDetector_Impl::detect(const cv::Mat &input, std::vector<Marker> &detectedMarkers,
                                 CameraParameters camParams, float markerSizeMeters,
                                 bool setYPerpendicular, bool correctFisheye)
{
  if (camParams.CamSize != input.size() && camParams.isValid() && markerSizeMeters > 0)
  {
    // must resize camera parameters if we want to compute properly marker poses
    CameraParameters cp_aux = camParams;
    cp_aux.resize(input.size());
    detect(input, detectedMarkers, cp_aux.CameraMatrix, cp_aux.Distorsion,
           cp_aux.ExtrinsicMatrix, markerSizeMeters, setYPerpendicular, correctFisheye);
  }
  else
  {
    detect(input, detectedMarkers, camParams.CameraMatrix, camParams.Distorsion,
           camParams.ExtrinsicMatrix, markerSizeMeters, setYPerpendicular, correctFisheye);
  }
}
int MarkerDetector_Impl::getMarkerWarpSize()
{
  auto bis = markerIdDetector->getBestInputSize();
  if (bis != -1)
    return bis;

  int ndiv = markerIdDetector->getNSubdivisions();
  if (ndiv == -1)
    ndiv = 7;  // set any possible value(it is used for non dictionary based labelers)
  return _params.markerWarpPixSize *
         ndiv;  // this is the minimum size that the smallest marker will have
}



void MarkerDetector_Impl::buildPyramid(vector<cv::Mat> &ImagePyramid, const cv::Mat &grey,
                                       int minSize)
{
  // determine numbre of pyramid images
  int npyrimg = 1;
  cv::Size imgpsize = grey.size();
  while (imgpsize.width > minSize)
  {
    imgpsize = cv::Size(imgpsize.width / _params.pyrfactor, imgpsize.height / _params.pyrfactor);
    npyrimg++;
  }

  ImagePyramid.resize(npyrimg);
  imagePyramid[0] = grey;
  // now, create pyramid images
  imgpsize = grey.size();
  for (int i = 1; i < npyrimg; i++)
  {
    cv::Size nsize(ImagePyramid[i - 1].cols / _params.pyrfactor,
                   ImagePyramid[i - 1].rows / _params.pyrfactor);
    cv::resize(ImagePyramid[i - 1], ImagePyramid[i], nsize);
  }
}

void impl_assignClass_fast(const cv::Mat &im, std::vector<cv::KeyPoint> &kpoints,
                           bool norm, int wsize)
{
  if (im.type() != CV_8UC1)
    throw std::runtime_error("assignClass_fast Input image must be 8UC1");
  int wsizeFull = wsize * 2 + 1;

  cv::Mat labels = cv::Mat::zeros(wsize * 2 + 1, wsize * 2 + 1, CV_8UC1);
  cv::Mat thresIm = cv::Mat(wsize * 2 + 1, wsize * 2 + 1, CV_8UC1);

  for (auto &kp : kpoints)
  {
    int x = kp.pt.x + 0.5f;
    int y = kp.pt.y + 0.5f;

    cv::Rect r = cv::Rect(x - wsize, y - wsize, wsize * 2 + 1, wsize * 2 + 1);
    // Check boundaries
    if (r.x < 0 || r.x + r.width > im.cols || r.y < 0 || r.y + r.height > im.rows)
      continue;


    int endX = r.x + r.width;
    int endY = r.y + r.height;
    uchar minV = 255, maxV = 0;
    for (int y = r.y; y < endY; y++)
    {
      const uchar *ptr = im.ptr<uchar>(y);
      for (int x = r.x; x < endX; x++)
      {
        if (minV > ptr[x])
          minV = ptr[x];
        if (maxV < ptr[x])
          maxV = ptr[x];
      }
    }

    if ((maxV - minV) < 25)
    {
      kp.class_id = 0;
      continue;
    }

    double thres = (maxV + minV) / 2.0;

    unsigned int nZ = 0;
    // count non zero considering the threshold
    for (int y = 0; y < wsizeFull; y++)
    {
      const uchar *ptr = im.ptr<uchar>(r.y + y) + r.x;
      uchar *thresPtr = thresIm.ptr<uchar>(y);
      for (int x = 0; x < wsizeFull; x++)
      {
        if (ptr[x] > thres)
        {
          nZ++;
          thresPtr[x] = 255;
        }
        else
          thresPtr[x] = 0;
      }
    }
    // set all to zero            labels.setTo(cv::Scalar::all(0));
    for (int y = 0; y < thresIm.rows; y++)
    {
      uchar *labelsPtr = labels.ptr<uchar>(y);
      for (int x = 0; x < thresIm.cols; x++)
        labelsPtr[x] = 0;
    }

    uchar newLab = 1;
    std::map<uchar, uchar> unions;
    for (int y = 0; y < thresIm.rows; y++)
    {
      uchar *thresPtr = thresIm.ptr<uchar>(y);
      uchar *labelsPtr = labels.ptr<uchar>(y);
      for (int x = 0; x < thresIm.cols; x++)
      {
        uchar reg = thresPtr[x];
        uchar lleft_px = 0;
        uchar ltop_px = 0;

        if (x - 1 > -1)
        {
          if (reg == thresPtr[x - 1])
            lleft_px = labelsPtr[x - 1];
        }

        if (y - 1 > -1)
        {
          if (reg == thresIm.ptr<uchar>(y - 1)[x])  // thresIm.at<uchar>(y-1, x)
            ltop_px = labels.at<uchar>(y - 1, x);
        }

        if (lleft_px == 0 && ltop_px == 0)
          labelsPtr[x] = newLab++;

        else if (lleft_px != 0 && ltop_px != 0)
        {
          if (lleft_px < ltop_px)
          {
            labelsPtr[x] = lleft_px;
            unions[ltop_px] = lleft_px;
          }
          else if (lleft_px > ltop_px)
          {
            labelsPtr[x] = ltop_px;
            unions[lleft_px] = ltop_px;
          }
          else
          {  // IGuales
            labelsPtr[x] = ltop_px;
          }
        }
        else
        {
          if (lleft_px != 0)
            labelsPtr[x] = lleft_px;
          else
            labelsPtr[x] = ltop_px;
        }
      }
    }

    int nc = newLab - 1 - unions.size();
    if (nc == 2)
    {
      if (nZ > thresIm.total() - nZ)
        kp.class_id = 0;
      else
        kp.class_id = 1;
    }
    else if (nc > 2)
    {
      kp.class_id = 2;
    }
  }
}

// void fastCorners{
//     vector<cv::KeyPoint> kpts;
//    cv::FAST(input,kpts,7);

//   // Adapter.
//   // Given an Point2f element, it returns the element of the dimension specified such
//   that dim=0 is x and dim=1 is y struct PicoFlann_Point2fAdapter{
//       inline  float operator( )(const cv::KeyPoint &kp, int dim)const { return
//       dim==0?kp.pt.x:kp.pt.y; }
//   };

//   picoflann::KdTreeIndex<2,PicoFlann_Point2fAdapter>  kdtree;//2 is the number of
//   dimensions kdtree.build(kpts);
//   //search 10 nearest neibors to point (0,0)
//   vector<bool> maxima(kpts.size(),true);
//   for(auto &kpt:kpts){
//       kpt.size=1;//used
//       kpt.class_id=-1;//not yet
//   }

//   int classIdx=0;
//   vector<pair< cv::Point2f,double> > center_weight;
//   center_weight.reserve(kpts.size());

//   for(size_t i=0;i<kpts.size();i++){
//       if( kpts[i].size){
//           size_t maxResponseIdx =i;
//           std::vector<std::pair<uint32_t,double> > res=kdtree.radiusSearch(kpts,kpts[i],7);
//           //compute the one with max response
//           for(auto p:res){
//               if( kpts[p.first].response>= kpts[maxResponseIdx].response && kpts[p.first].size)
//                   maxResponseIdx= p.first ;
//           }
//           auto &maxKp=kpts[ maxResponseIdx];

//           if(maxKp.class_id==-1){
//               maxKp.class_id=int(center_weight.size());
//               center_weight.push_back( { maxKp.pt*maxKp.response,maxKp.response});
//           }

//           //the others are non maxima
//           for(auto p:res){
//               if(p.first==maxResponseIdx) continue;
//               auto  &kp=kpts[p.first];
//               if( kp.size!=0 &&  kp.class_id==-1){
//                   kp.size=0;//sets as non maxiima
//                   center_weight[maxKp.class_id].first+=kp.pt*kp.response;
//                   center_weight[maxKp.class_id].second+=kp.response;
//               }
//           }
//       }
//   }

//   //take only the selected ones
//   kpts.clear();
//   for(auto cw:center_weight){
//       cv::KeyPoint kp;
//       kp.pt= cw.first*(1./cw.second);
//       kpts.push_back(kp);
//   }

////   kpts.erase(std::remove_if(kpts.begin(),kpts.end(), [ ](const cv::KeyPoint
///&kp){return kp.class_id==0;}), kpts.end());

//   assignClass_fast(input,kpts,false,5);


//    cv::Mat im2;
//    cv::cvtColor(input,im2,CV_GRAY2BGR);
//     for(int i=0;i< kpts.size();i++){
//        auto & kp=kpts[i];

//        if(kp.class_id==2){

//            cv::rectangle(auxThresImage,kp.pt-cv::Point2f(2,2),kp.pt+cv::Point2f(2,2),cv::Scalar(0,0,0),-1);
//            cv::rectangle(im2,kp.pt-cv::Point2f(2,2),kp.pt+cv::Point2f(2,2),cv::Scalar(255,0,123));
//        }
//        else
//            cv::rectangle(im2,kp.pt-cv::Point2f(2,2),kp.pt+cv::Point2f(2,2),cv::Scalar(125,125,125));
//    }

//}
/**************************************************
 *
 */

vector<MarkerDetector_Impl::MarkerCandidate> MarkerDetector_Impl::thresholdAndDetectRectangles(
    const cv::Mat &input, int thres_param1, int thres_param2, bool enclosed, cv::Mat &auxThresImage)
{
  // ensure that _thresParam1%2==1
  __ARUCO_ADDTIMER__;
  if (thres_param1 < 3)
    thres_param1 = 3;
  else if (((int)thres_param1) % 2 != 1)
    thres_param1 = (int)(thres_param1 + 1);

  int enclosedOffset = -1;
  cv::Mat auxImage;
  // if ( !erode)
  auxImage = auxThresImage;
  if (_params.thresMethod == MarkerDetector::THRES_AUTO_FIXED)
  {
    cv::threshold(input, auxImage, static_cast<int>(thres_param2), 255, THRESH_BINARY_INV);
    if (enclosed)
    {
      cv::Mat aux1;
      enclosedOffset = int(std::max(3.0, 3. / 1920. * float(auxImage.cols)));
      if (enclosedOffset % 2 == 0)
        enclosedOffset++;
      cv::erode(auxImage, aux1,
                getStructuringElement(MORPH_CROSS, cv::Size(enclosedOffset, enclosedOffset),
                                      cv::Point(enclosedOffset / 2, enclosedOffset / 2)));
      cv::bitwise_xor(aux1, auxImage, auxImage);

      __ARUCO_TIMER_EVENT__("erode");
    }
  }
  else
  {
    cv::adaptiveThreshold(input, auxImage, 255., ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV,
                          static_cast<int>(thres_param1), static_cast<int>(thres_param2));
    if (_params.closingSize > 0)
    {
      int p = _params.closingSize * 2 + 1;
      cv::Mat im2;
      cv::Mat ker = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(p, p));
      cv::morphologyEx(auxImage, im2, cv::MORPH_OPEN, ker);
      auxThresImage = im2;
    }
    enclosedOffset = thres_param1;
  }
  __ARUCO_TIMER_EVENT__("thres");


  // fastCorners ()


  vector<MarkerCandidate> MarkerCanditates;
  // calcualte the min_max contour sizes
  int thisImageMinSize = int(3.5 * float(_params.lowResMarkerSize));
  // if image is eroded, minSize must be adapted
  std::vector<cv::Vec4i> hierarchy;
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(auxThresImage, contours, cv::noArray(), cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
  __ARUCO_TIMER_EVENT__("find-cont");
  vector<Point> approxCurve;
//#define _aruco_debug_detectrectangles
#ifdef _aruco_debug_detectrectangles
  cv::Mat simage;
  cv::cvtColor(input, simage, CV_GRAY2BGR);
#endif

  /// for each contour, analyze if it is a paralelepiped likely to be the marker
  for (unsigned int i = 0; i < contours.size(); i++)
  {
#ifdef _aruco_debug_detectrectangles
    drawContour(simage, contours[i], Scalar(125, 125, 255));
#endif
    // check it is a possible element by first checking that is is large enough
    if (thisImageMinSize < int(contours[i].size()))
    {
      // can approximate to a convex rect?
      cv::approxPolyDP(contours[i], approxCurve, double(contours[i].size()) * 0.05, true);

      if (approxCurve.size() == 4 && cv::isContourConvex(approxCurve))
      {
#ifdef _aruco_debug_detectrectangles
        drawApproxCurve(simage, approxCurve, Scalar(255, 0, 255), 1);
#endif
        // ensure that the   distace between consecutive points is large enough
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
          MarkerCanditates.back().push_back(Point2f(static_cast<float>(approxCurve[j].x),
                                                    static_cast<float>(approxCurve[j].y)));
        // now, if it is eroded, must enalrge 1 bit the corners to go to the real location
        // for each opposite pair, take the line joining them and move one pixel apart
        // ideally, Bresenham's algorithm should be used
        if (enclosed)
        {
          enlargeMarkerCandidate(MarkerCanditates.back(), float(enclosedOffset) / 2.);
        }
#ifdef _aruco_debug_detectrectangles
        MarkerCanditates.back().draw(simage, Scalar(255, 255, 0), 1, false);
#endif
        MarkerCanditates.back().contourPoints = contours[i];
      }
    }
  }
#ifdef _aruco_debug_detectrectangles
  cv::imshow("contours", simage);
#endif
  return MarkerCanditates;
}


void MarkerDetector_Impl::thresholdAndDetectRectangles_thread()
{
  while (true)
  {
    bool enclosed = false;
    auto tad = _tasks.pop();
    if (tad.task == EXIT_TASK)
      return;
    else if (tad.task == ENCLOSE_TASK)
      enclosed = true;
    _vcandidates[tad.outIdx] =
        thresholdAndDetectRectangles(_thres_Images[tad.inIdx], tad.param1, tad.param2,
                                     enclosed, _thres_Images[tad.outIdx]);
  };
}

vector<aruco::MarkerDetector_Impl::MarkerCandidate> MarkerDetector_Impl::thresholdAndDetectRectangles(
    const cv::Mat &image)
{
  // compute the different values of param1

  int adaptiveWindowSize = _params.AdaptiveThresWindowSize;
  if (_params.AdaptiveThresWindowSize == -1)
    adaptiveWindowSize = max(int(3), int(15 * float(image.cols) / 1920.));
  if (adaptiveWindowSize % 2 == 0)
    adaptiveWindowSize++;

  // adaptiveWindowSize=15;

  vector<int> p1_values;
  for (int i = static_cast<int>(
           std::max(3., adaptiveWindowSize - 2. * _params.AdaptiveThresWindowSize_range));
       i <= adaptiveWindowSize + 2 * _params.AdaptiveThresWindowSize_range; i += 2)
    p1_values.push_back(i);

  size_t nimages = p1_values.size();
  _tooNearDistance = p1_values.back();
  _vcandidates.resize(nimages);
  _thres_Images.resize(nimages + 1);
  _thres_Images.back() = image;  // add at the end the original image
  // first, thresholded images
  ThresAndDetectRectTASK tad;
  vector<ThresAndDetectRectTASK> vtad;


  ThreadTasks task = THRESHOLD_TASK;
  if (_params.enclosedMarker)
    task = ENCLOSE_TASK;
  for (size_t i = 0; i < p1_values.size(); i++)
  {
    tad.inIdx = int(_thres_Images.size() - 1);
    tad.param1 = p1_values[i];
    tad.param2 = _params.ThresHold;
    tad.outIdx = i;
    tad.task = task;
    _tasks.push(tad);
    vtad.push_back(tad);
  }


  // reserve images
  for (size_t i = 0; i < nimages; i++)
    _thres_Images[i].create(image.size(), CV_8UC1);


  // how many threads will be used?
  int nthreads = 0;
  if (_params.maxThreads <= 0)  // if allowed to use all , take max()-1, since the
                                // buildpyramid must be working at this moment
    nthreads = std::thread::hardware_concurrency() - 1;
  else
    nthreads = max(1, _params.maxThreads - 1);

  tad.task = EXIT_TASK;
  for (int i = 0; i < nthreads; i++)
    _tasks.push(tad);

  if (nthreads == 1)
  {  // no threads
    ScopeTimer Timer("non-parallel");
    thresholdAndDetectRectangles_thread();
  }
  else
  {  // parallell mode
    // add the final task END
    ScopeTimer Timer("parallel");

    // run the tasks (in parallel)
    vector<std::thread> threads;
    for (int i = 0; i < nthreads; i++)
      threads.push_back(
          std::thread(&MarkerDetector_Impl::thresholdAndDetectRectangles_thread, this));
    // wait for them to finish
    for (auto &th : threads)
      th.join();
  }
  vector<MarkerCandidate> joined;
  joinVectors(_vcandidates, joined);
  return joined;
}

vector<MarkerDetector_Impl::MarkerCandidate> MarkerDetector_Impl::prefilterCandidates(
    vector<aruco::MarkerDetector_Impl::MarkerCandidate> &MarkerCanditates, cv::Size imgSize)
{
  /////////////////////////////////////////////////////////////////////////////////////
  /// CANDIDATE PREFILTERING- Merge and Remove candidates so that only reliable ones are returned
  //////////////////////////////////////////////////////////////////////////////////
  /// sort the points in anti-clockwise order
  valarray<bool> swapped(false, MarkerCanditates.size());  // used later
  for (unsigned int i = 0; i < MarkerCanditates.size(); i++)
  {
    // trace a line between the first and second point.
    // if the thrid point is at the right side, then the points are anti-clockwise
    double dx1 = MarkerCanditates[i][1].x - MarkerCanditates[i][0].x;
    double dy1 = MarkerCanditates[i][1].y - MarkerCanditates[i][0].y;
    double dx2 = MarkerCanditates[i][2].x - MarkerCanditates[i][0].x;
    double dy2 = MarkerCanditates[i][2].y - MarkerCanditates[i][0].y;
    double o = (dx1 * dy2) - (dy1 * dx2);

    if (o < 0.0)
    {  // if the third point is in the left side, then sort in anti-clockwise order
      swap(MarkerCanditates[i][1], MarkerCanditates[i][3]);
      swapped[i] = true;
    }
  }

  if (0)
  {
    /// remove these elements which corners are too close to each other
    // first detect candidates to be removed
    vector<pair<int, int>> TooNearCandidates;
    for (unsigned int i = 0; i < MarkerCanditates.size(); i++)
    {
      // calculate the average distance of each corner to the nearest corner of the other marker candidate
      for (unsigned int j = i + 1; j < MarkerCanditates.size(); j++)
      {
        valarray<float> vdist(4);
        for (int c = 0; c < 4; c++)
          vdist[c] = cv::norm(MarkerCanditates[i][c] - MarkerCanditates[j][c]);
        // if distance is too small
        if (vdist[0] < _tooNearDistance && vdist[1] < _tooNearDistance &&
            vdist[2] < _tooNearDistance && vdist[3] < _tooNearDistance)
          TooNearCandidates.push_back(pair<int, int>(i, j));
      }
    }

    /// mark for removal the element of  the pair with smaller perimeter
    vector<bool> toRemove(MarkerCanditates.size(), false);
    for (unsigned int i = 0; i < TooNearCandidates.size(); i++)
    {
      if (perimeter(MarkerCanditates[TooNearCandidates[i].first]) >
          perimeter(MarkerCanditates[TooNearCandidates[i].second]))
        toRemove[TooNearCandidates[i].second] = true;
      else
        toRemove[TooNearCandidates[i].first] = true;
    }

    /// find these too near borders  and remove them
    // remove markers with corners too near the image limits
    int borderDistThresX = static_cast<int>(_params.borderDistThres * float(imgSize.width));
    int borderDistThresY = static_cast<int>(_params.borderDistThres * float(imgSize.height));
    for (size_t i = 0; i < MarkerCanditates.size(); i++)
    {
      // delete if any of the corners is too near image border
      for (size_t c = 0; c < MarkerCanditates[i].size(); c++)
      {
        if (MarkerCanditates[i][c].x < borderDistThresX ||
            MarkerCanditates[i][c].y < borderDistThresY ||
            MarkerCanditates[i][c].x > imgSize.width - borderDistThresX ||
            MarkerCanditates[i][c].y > imgSize.height - borderDistThresY)
        {
          toRemove[i] = true;
        }
      }
    }

    // move to output only valid ones
    vector<MarkerCandidate> finalCandidates;
    finalCandidates.reserve(MarkerCanditates.size());
    for (size_t i = 0; i < MarkerCanditates.size(); i++)
      if (!toRemove[i])
        finalCandidates.push_back(MarkerCanditates[i]);
    return finalCandidates;
  }
  else
    return MarkerCanditates;
}


void MarkerDetector_Impl::addToImageHist(cv::Mat &im, std::vector<float> &hist)
{
  for (int y = 0; y < im.rows; y++)
  {
    uchar *ptr = im.ptr<uchar>(y);
    for (int x = 0; x < im.cols; x++)
      hist[ptr[x]]++;
  }
}

int MarkerDetector_Impl::Otsu(std::vector<float> &hist)
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
      // cout<<t<<" : "<<var<<"|"<<w0<<" "<<w1<<" "<<mean0<<" "<<mean1<<endl;
      if (var > maxVar)
      {
        maxVar = var;
        bestT = t;
      }
    }
  }
  return bestT;
}
/************************************
 * Main detection function. Performs all steps
 ************************************/
void MarkerDetector_Impl::detect(const cv::Mat &input, std::vector<Marker> &detectedMarkers,
                                 cv::Mat camMatrix, cv::Mat distCoeff, cv::Mat extrinsics,
                                 float markerSizeMeters, bool setYPerpendicular,
                                 bool correctFisheye)
{
  // clear input data
  detectedMarkers.clear();
  _vcandidates.clear();
  _candidates_wcontour.clear();
  __ARUCO_ADDTIMER__;


  // it must be a 3 channel image
  if (input.type() == CV_8UC3)
    cv::cvtColor(input, grey, CV_BGR2GRAY);
  //  convertToGray(input, grey);
  else
    grey = input;
  __ARUCO_TIMER_EVENT__("ConvertGrey");

  //////////////////////////////////////////////////////////////////////
  /// CREATE LOW RESOLUTION IMAGE IN WHICH MARKERS WILL BE DETECTED
  //////////////////////////////////////////////////////////////////////
  float ResizeFactor = 1;
  // use the minimum and markerWarpSize to determine the optimal image size on which to do rectangle detection
  cv::Mat imgToBeThresHolded;
  cv::Size maxImageSize = grey.size();
  auto minpixsize =
      getMinMarkerSizePix(input.size());  // min pixel size of the marker in the original image
  if (_params.lowResMarkerSize < minpixsize)
  {
    ResizeFactor = float(_params.lowResMarkerSize) / float(minpixsize);
    if (ResizeFactor < 0.9)
    {  // do not waste time if smaller than this
      _debug_msg("Scale factor=" << ResizeFactor, 1);
      maxImageSize.width = float(grey.cols) * ResizeFactor + 0.5;
      maxImageSize.height = float(grey.rows) * ResizeFactor + 0.5;
      if (maxImageSize.width % 2 != 0)
        maxImageSize.width++;
      if (maxImageSize.height % 2 != 0)
        maxImageSize.height++;
      cv::resize(grey, imgToBeThresHolded, maxImageSize, 0, 0, cv::INTER_NEAREST);
      //                cv::resize(grey,imgToBeThresHolded,maxImageSize,0,0,cv::INTER_LINEAR);
    }
  }

  if (imgToBeThresHolded.empty())  // if not set in previous step, add original now
    imgToBeThresHolded = grey;

  __ARUCO_TIMER_EVENT__("CreateImageToTheshold");
  bool needPyramid =
      true;  // ResizeFactor< 1/_params.pyrfactor;//only use pyramid if working on a big image.
  std::thread buildPyramidThread;
  if (needPyramid)
  {
    if (_params.maxThreads > 1)
      buildPyramidThread =
          std::thread([&] { buildPyramid(imagePyramid, grey, 2 * getMarkerWarpSize()); });
    else
      buildPyramid(imagePyramid, grey, 2 * getMarkerWarpSize());
    __ARUCO_TIMER_EVENT__("BuildPyramid");
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
    ///////////////////////////////////////////////////////////////////////////
    /// THRESHOLD IMAGES AND DETECT INITIAL RECTANGLES
    ///////////////////////////////////////////////////////////////////////////
    vector<MarkerCandidate> MarkerCanditates;
    MarkerCanditates = thresholdAndDetectRectangles(imgToBeThresHolded);
    thres = _thres_Images[0];

    //        _debug_exec(10,
    //        {//only executes when compiled in DEBUG mode if debug level is at least 10
    //         //show the thresholded images
    //         for (size_t i = 0; i < _thres_Images.size(); i++) {
    //             stringstream sstr; sstr << "thres-" << i;
    //             cv::namedWindow(sstr.str(),cv::WINDOW_NORMAL);
    //             cv::imshow(sstr.str(),_thres_Images[i]);
    //         }});


    __ARUCO_TIMER_EVENT__("Threshold and Detect rectangles");
    // prefilter candidates
    //        _debug_exec(10,//only executes when compiled in DEBUG mode if debug level is at least 10
    //                    //show the thresholded images
    //                    cv::Mat imrect;
    //                cv::cvtColor(imgToBeThresHolded,imrect,CV_GRAY2BGR);
    //        for(auto m: MarkerCanditates )
    //            m.draw(imrect,cv::Scalar(0,245,0));
    //        cv::imshow("rect-nofiltered",imrect);
    //        );

    MarkerCanditates = prefilterCandidates(MarkerCanditates, imgToBeThresHolded.size());

    __ARUCO_TIMER_EVENT__("prefilterCandidates");

    //        _debug_exec(10,//only executes when compiled in DEBUG mode if debug level is at least 10
    //                    //show the thresholded images
    //                    cv::Mat imrect;
    //                cv::cvtColor(imgToBeThresHolded,imrect,CV_GRAY2BGR);
    //        for(auto m: MarkerCanditates)
    //            m.draw(imrect,cv::Scalar(0,245,0));
    //        cv::imshow("rect-filtered",imrect);
    //        );
    // before going on, make sure the piramid is built
    if (buildPyramidThread.joinable())
      buildPyramidThread.join();


    ///////////////////////////////////////////////////////////////////////////
    /// CANDIDATE CLASSIFICATION: Decide which candidates are really markers
    ///////////////////////////////////////////////////////////////////////////

    // Debug::setLevel(10);
    auto markerWarpSize = getMarkerWarpSize();

    detectedMarkers.clear();
    _candidates_wcontour.clear();
    for (auto &b : hist)
      b = 0;
    float desiredarea = std::pow(static_cast<float>(markerWarpSize), 2.f);
    for (size_t i = 0; i < MarkerCanditates.size(); i++)
    {
      // Find proyective homography
      cv::Mat canonicalMarker, canonicalMarkerAux;

      cv::Mat inToWarp = imgToBeThresHolded;
      MarkerCandidate points2d_pyr = MarkerCanditates[i];
      if (needPyramid)
      {
        // warping is one of the most time consuming operations, especially when the
        // region is large. To reduce computing time, let us find in the image pyramid,
        // the best configuration to save time indicates how much bigger observation is
        // wrt to desired patch
        size_t imgPyrIdx = 0;
        for (size_t p = 1; p < imagePyramid.size(); p++)
        {
          if (MarkerCanditates[i].getArea() / pow(4, p) >= desiredarea)
            imgPyrIdx = p;
          else
            break;
        }
        inToWarp = imagePyramid[imgPyrIdx];
        // move points to the image level p
        float ratio = float(inToWarp.cols) / float(imgToBeThresHolded.cols);
        for (auto &p : points2d_pyr)
          p *= ratio;  // 1. / pow(2, imgPyrIdx);
      }
      warp(inToWarp, canonicalMarker, Size(markerWarpSize, markerWarpSize), points2d_pyr);
      int id, nRotations;
      double min, Max;
      cv::minMaxIdx(canonicalMarker, &min, &Max);
      canonicalMarker.copyTo(canonicalMarkerAux);
      string additionalInfo;
      //            _debug_exec(10,//only executes when compiled in DEBUG mode if debug
      //            level is at least 10
      //                        //show the thresholded images
      //                        stringstream sstr;sstr<<"test-"<<i;
      //            cout  <<"test"<<i<<endl;
      //            cv::namedWindow(sstr.str(),cv::WINDOW_NORMAL);
      //            cv::imshow(sstr.str(),canonicalMarkerAux);
      //            cv::waitKey(0);
      //             );
      if (markerIdDetector->detect(canonicalMarkerAux, id, nRotations, additionalInfo))
      {
        detectedMarkers.push_back(MarkerCanditates[i]);
        detectedMarkers.back().id = id;
        detectedMarkers.back().dict_info = additionalInfo;
        detectedMarkers.back().contourPoints = MarkerCanditates[i].contourPoints;
        // sort the points so that they are always in the same order no matter the camera orientation
        std::rotate(detectedMarkers.back().begin(), detectedMarkers.back().begin() + 4 - nRotations,
                    detectedMarkers.back().end());
        //                _debug_exec(10,//only executes when compiled in DEBUG mode if
        //                debug level is at least 10
        //                            //show the thresholded images
        //                            stringstream
        //                            sstr;sstr<<"can-"<<detectedMarkers.back().id;
        //                cv::namedWindow(sstr.str(),cv::WINDOW_NORMAL);
        //                cv::imshow(sstr.str(),canonicalMarker);
        //                cout<<"ID="<<id<<" "<< detectedMarkers.back()<<endl;
        //                );
        if (_params.thresMethod == MarkerDetector::THRES_AUTO_FIXED)
          addToImageHist(canonicalMarker, hist);
      }
      else
      {
        _candidates_wcontour.push_back(MarkerCanditates[i]);
      }
    }
    __ARUCO_TIMER_EVENT__("Marker classification. ");
    if (detectedMarkers.size() == 0 && _params.thresMethod == MarkerDetector::THRES_AUTO_FIXED &&
        ++nAttemptsAutoFix < _params.NAttemptsAutoThresFix)
    {
      _params.ThresHold = 10 + rand() % 230;
      keepLookingFor = true;
    }
    else
      keepLookingFor = false;
  } while (keepLookingFor);

  //  Debug::setLevel(5);


  if (_params.thresMethod == MarkerDetector::THRES_AUTO_FIXED)
  {
    int newThres = Otsu(hist);
    ;
    if (newThres > 0)
      _params.ThresHold = float(newThres);
  }

#ifdef debug_lines
  cv::imshow("image-lines", image);
  cv::waitKey(10);
#endif
  // now, move the points to the original image (upsample corners)
  if (input.cols != imgToBeThresHolded.cols)
  {
    cornerUpsample(detectedMarkers, imgToBeThresHolded.size());
    __ARUCO_TIMER_EVENT__("Corner Upsample");
  }



  //////////////////////////////////////////////////////////////////////
  /// MARKER Tracking
  //////////////////////////////////////////////////////////////////////


  if (_params.trackingMinDetections > 0)
  {
    // update the marker_ndetections
    // decrement unseen ones
    for (auto &md : marker_ndetections)
    {
      if (std::find(detectedMarkers.begin(), detectedMarkers.end(), Marker(md.first)) ==
          detectedMarkers.end())
        md.second = std::max(md.second - 1, 0);
    }

    // Identify the markers in the prev frame not yet tracked
    std::vector<Marker> needsTrack;
    for (auto m : _prevMarkers)
    {
      if (std::find(detectedMarkers.begin(), detectedMarkers.end(), Marker(m.id)) ==
          detectedMarkers.end())
      {
        if (marker_ndetections.count(m.id) != 0)
        {
          if (marker_ndetections.at(m.id) >= _params.trackingMinDetections)
            needsTrack.push_back(m);
        }
      }
    }

    // look for them in the candidates
    if (needsTrack.size() > 0)
    {
      struct trackInfo
      {
        trackInfo(const Marker &m)
        {
          ma.setParams(m);
        }
        marker_analyzer ma;
        int candidate = -1;
        double dist = std::numeric_limits<double>::max();
      };
      std::map<int, trackInfo> marker_trackMatches;
      for (auto &mnt : needsTrack)
        marker_trackMatches.insert({ mnt.id, trackInfo(mnt) });

      for (size_t cand = 0; cand < _candidates_wcontour.size(); cand++)
      {
        auto &mcnd = _candidates_wcontour[cand];
        marker_analyzer can_ma;
        can_ma.setParams(mcnd);
        for (auto &mtm : marker_trackMatches)
        {
          if (mtm.second.ma.isInto(can_ma.getCenter()))
          {
            auto dist = cv::norm(mtm.second.ma.getCenter() - can_ma.getCenter());
            auto sizedif =
                fabs(mtm.second.ma.getArea() - can_ma.getArea()) / mtm.second.ma.getArea();
            if (sizedif < 0.3f && dist < mtm.second.dist)
            {
              mtm.second.candidate = static_cast<int>(cand);
              mtm.second.dist = dist;
            }
          }
        }
      }

      for (auto &mtm : marker_trackMatches)
      {
        if (mtm.second.candidate == -1)
          continue;
        auto it = find(_prevMarkers.begin(), _prevMarkers.end(), Marker(mtm.first));
        assert(it != _prevMarkers.end());
        detectedMarkers.push_back(_candidates_wcontour[mtm.second.candidate]);
        detectedMarkers.back().id = mtm.first;
        detectedMarkers.back().dict_info = it->dict_info;
        detectedMarkers.back().contourPoints =
            _candidates_wcontour[mtm.second.candidate].contourPoints;
        // find best rotatation

        auto rotError = [](const vector<cv::Point2f> &c1, const vector<cv::Point2f> &c2)
        {
          cv::Point2f direction1 = c1[1] - c1[0];
          direction1 *= 1. / cv::norm(direction1);
          cv::Point2f direction2 = c2[1] - c2[0];
          direction2 *= 1. / cv::norm(direction2);
          return direction1.dot(direction2);
        };
        auto prev = find(_prevMarkers.begin(), _prevMarkers.end(), Marker(mtm.first));
        pair<int, double> rot_error(-1, -1);
        for (int r = 0; r < 4; r++)
        {
          vector<cv::Point2f> aux = detectedMarkers.back();
          std::rotate(aux.begin(), aux.begin() + r, aux.end());
          auto err = rotError(*prev, aux);
          if (err > rot_error.second)
          {
            rot_error = { r, err };
          }
        }
        std::rotate(detectedMarkers.back().begin(),
                    detectedMarkers.back().begin() + rot_error.first,
                    detectedMarkers.back().end());
        // mark for removal
        _candidates_wcontour[mtm.second.candidate].clear();
      }
      // remove the used candidates
      _candidates_wcontour.erase(
          std::remove_if(_candidates_wcontour.begin(), _candidates_wcontour.end(),
                         [](const vector<cv::Point2f> &m) { return m.size() == 0; }),
          _candidates_wcontour.end());
    }
    // update for next
    for (auto m : detectedMarkers)
    {
      if (marker_ndetections.count(m.id) == 0)
        marker_ndetections[m.id] = 1;
      else
        marker_ndetections[m.id]++;
    }
  }


  //////////////////////////////////////////////////////////////////////
  /// REMOVAL OF DUPLICATED
  //////////////////////////////////////////////////////////////////////

  if (1)
  {
    // sort by id
    std::sort(detectedMarkers.begin(), detectedMarkers.end());

    // there might be still the case that a marker is detected twice because of the double
    // border indicated earlier, detect and remove these cases
    vector<bool> toRemove(detectedMarkers.size(), false);

    for (int i = 0; i < int(detectedMarkers.size()) - 1; i++)
    {
      for (int j = i + 1; j < int(detectedMarkers.size()) && !toRemove[i]; j++)
      {
        if (detectedMarkers[i].id == detectedMarkers[j].id &&
            detectedMarkers[i].dict_info == detectedMarkers[j].dict_info)
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
  }


  //////////////////////////////////////////////////////////////////////
  /// CORNER REFINEMENT IF REQUIRED
  //////////////////////////////////////////////////////////////////////
  /// refine the corner location if enclosed markers and we did not do it via upsampling
  if (detectedMarkers.size() > 0 && input.size() == imgToBeThresHolded.size())
  {
    if (_params.cornerRefinementM == CORNER_SUBPIX)
    {
      int halfwsize = 4 * float(input.cols) / float(imgToBeThresHolded.cols) + 0.5;

      vector<Point2f> Corners;
      for (unsigned int i = 0; i < detectedMarkers.size(); i++)
        for (int c = 0; c < 4; c++)
          Corners.push_back(detectedMarkers[i][c]);
      cornerSubPix(grey, Corners, cv::Size(halfwsize, halfwsize), cv::Size(-1, -1),
                   cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 12, 0.005));
      // copy back
      for (unsigned int i = 0; i < detectedMarkers.size(); i++)
        for (int c = 0; c < 4; c++)
          detectedMarkers[i][c] = Corners[i * 4 + c];
    }
    else if (_params.cornerRefinementM == CORNER_LINES)
    {
      // use the lines method for estimation of the corners
      for (unsigned int i = 0; i < detectedMarkers.size(); i++)
        refineCornerWithContourLines(detectedMarkers[i]);
    }
  }
  __ARUCO_TIMER_EVENT__("Corner Refinement");
  ///



  //////////////////////////////////////////////////////////////////////
  /// MARKER POSE ESTIMATION
  //////////////////////////////////////////////////////////////////////

  /// detect the position of detected markers if desired
  if (camMatrix.rows != 0 && markerSizeMeters > 0)
  {
    for (unsigned int i = 0; i < detectedMarkers.size(); i++)
      detectedMarkers[i].calculateExtrinsics(markerSizeMeters, camMatrix, distCoeff,
                                             extrinsics, setYPerpendicular, correctFisheye);
    __ARUCO_TIMER_EVENT__("Pose Estimation");
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
  if (_params.autoSize)
  {
    _params.minSize = markerMinSize * (1 - _params.ts);
  }


  /// save for tracking
  _prevMarkers = detectedMarkers;
}
void MarkerDetector_Impl::refineCornerWithContourLines(aruco::Marker &marker,
                                                       cv::Mat camMatrix, cv::Mat distCoeff)
{
  // search corners on the contour vector

  std::vector<cv::Point> &contour = marker.contourPoints;
  vector<int> cornerIndex(4, -1);
  vector<float> dist(4, std::numeric_limits<float>::max());
  for (unsigned int j = 0; j < contour.size(); j++)
  {
    for (unsigned int k = 0; k < 4; k++)
    {
      float d = (contour[j].x - marker[k].x) * (contour[j].x - marker[k].x) +
                (contour[j].y - marker[k].y) * (contour[j].y - marker[k].y);
      if (d < dist[k])
      {
        cornerIndex[k] = j;
        dist[k] = d;
      }
    }
  }

  // if any cornerIndex


  // contour pixel in inverse order or not?
  bool inverse;
  if ((cornerIndex[1] > cornerIndex[0]) &&
      (cornerIndex[2] > cornerIndex[1] || cornerIndex[2] < cornerIndex[0]))
    inverse = false;
  else if (cornerIndex[2] > cornerIndex[1] && cornerIndex[2] < cornerIndex[0])
    inverse = false;
  else
    inverse = true;


  // get pixel vector for each line of the marker
  int inc = 1;
  if (inverse)
    inc = -1;

  // undistort contour
  vector<Point2f> contour2f;
  if (!camMatrix.empty() && !distCoeff.empty())
  {
    for (unsigned int i = 0; i < contour.size(); i++)
      contour2f.push_back(cv::Point2f(contour[i].x, contour[i].y));
    if (!camMatrix.empty() && !distCoeff.empty())
      cv::undistortPoints(contour2f, contour2f, camMatrix, distCoeff, cv::Mat(), camMatrix);
  }
  else
  {
    contour2f.reserve(contour.size());
    for (auto p : contour)
      contour2f.push_back(cv::Point2f(p.x, p.y));
  }

  vector<std::vector<cv::Point2f>> contourLines;
  contourLines.resize(4);
  for (unsigned int l = 0; l < 4; l++)
  {
    for (int j = (int)cornerIndex[l]; j != (int)cornerIndex[(l + 1) % 4]; j += inc)
    {
      if (j == (int)contour.size() && !inverse)
        j = 0;
      else if (j == 0 && inverse)
        j = contour.size() - 1;
      contourLines[l].push_back(contour2f[j]);
      if (j == (int)cornerIndex[(l + 1) % 4])
        break;  // this has to be added because of the previous ifs
    }
  }

  // interpolate marker lines
  vector<Point3f> lines;
  lines.resize(4);
  for (unsigned int j = 0; j < lines.size(); j++)
    interpolate2Dline(contourLines[j], lines[j]);

  // get cross points of lines
  vector<Point2f> crossPoints;
  crossPoints.resize(4);
  for (unsigned int i = 0; i < 4; i++)
    crossPoints[i] = getCrossPoint(lines[(i - 1) % 4], lines[i]);

  // distort corners again if undistortion was performed
  if (!camMatrix.empty() && !distCoeff.empty())
    distortPoints(crossPoints, crossPoints, camMatrix, distCoeff);

  // reassing points
  for (unsigned int j = 0; j < 4; j++)
  {
    // cout<<marker[j]<<" "<<crossPoints[j]<<endl;
    marker[j] = crossPoints[j];
  }
}



// expands the corners of the candidate to reach the real locations
// used in eroded images
void MarkerDetector_Impl::enlargeMarkerCandidate(MarkerCandidate &cand, int fact)
{
  for (int j = 0; j < 2; j++)
  {
    auto startp = j;
    auto endp = (j + 2) % 4;
    // sort so that the nearest to x is first
    if (cand[startp].x > cand[endp].x)
    {
      swap(startp, endp);
    }
    const float _180 = 3.14159f;

    const float _22 = 3.14159 / 8.f;
    const float _3_22 = 3. * 3.14159f / 8.f;
    const float _5_22 = 5.f * 3.14159f / 8.f;
    const float _7_22 = 7.f * 3.14159f / 8.f;

    int incx = 0, incy = 0;
    // compute the angle
    auto v1 = cand[endp] - cand[startp];
    float angle = atan2(v1.y, v1.x);
    if (_22 < angle && angle < 3 * _22)
    {  // a
      incx = incy = fact;
    }
    else if (-_22 < angle && angle < _22)
    {  // b
      incx = fact;
      incy = 0;
    }
    else if (-_3_22 < angle && angle < -_22)
    {  // c
      incx = fact;
      incy = -fact;
    }
    else if (-_5_22 < angle && angle < -_3_22)
    {  // D
      incx = 0;
      incy = -fact;
    }
    else if (-_7_22 < angle && angle < -_5_22)
    {  // E
      incx = -fact;
      incy = -fact;
    }
    else if ((-_180 < angle && angle < -_7_22) || (_7_22 < angle && angle < _180))
    {  // f
      incx = -fact;
      incy = 0;
    }
    else if ((_5_22 < angle && angle < _7_22))
    {  // g
      incx = -fact;
      incy = fact;
    }
    else if ((_3_22 < angle && angle < _5_22))
    {  // h
      incx = fact;
      incy = fact;
    }
    cand[endp].x += incx;
    cand[endp].y += incy;
    cand[startp].x -= incx;
    cand[startp].y -= incy;
  }
}



int MarkerDetector_Impl::getMinMarkerSizePix(cv::Size orginput_imageSize) const
{
  if (_params.minSize == -1 && _params.minSize_pix == -1)
    return 0;
  // calcualte the min_max contour sizes
  int maxDim = std::max(orginput_imageSize.width, orginput_imageSize.height);
  int minSize = 0;
  if (_params.minSize != -1)
    minSize = static_cast<float>(_params.minSize) * static_cast<float>(maxDim);
  if (_params.minSize_pix != -1)
    minSize = std::min(_params.minSize_pix, minSize);
  return minSize;
}
/************************************
 *
 *
 *
 *
 ************************************/
bool MarkerDetector_Impl::warp(Mat &in, Mat &out, Size size, vector<Point2f> points)
{
  if (points.size() != 4)
    throw cv::Exception(9001, "point.size()!=4", "MarkerDetector_Impl::warp", __FILE__, __LINE__);
  // obtain the perspective transform
  Point2f pointsRes[4], pointsIn[4];
  for (int i = 0; i < 4; i++)
    pointsIn[i] = points[i];
  pointsRes[0] = (Point2f(0, 0));
  pointsRes[1] = Point2f(static_cast<float>(size.width - 1), 0.f);
  pointsRes[2] =
      Point2f(static_cast<float>(size.width - 1), static_cast<float>(size.height - 1));
  pointsRes[3] = Point2f(0.f, static_cast<float>(size.height - 1));
  Mat M = getPerspectiveTransform(pointsIn, pointsRes);
  cv::warpPerspective(in, out, M, size, cv::INTER_LINEAR);
  //  cv::warpPerspective(in, out, M, size, cv::INTER_NEAREST);
  return true;
}


/************************************
 *
 *
 *
 *
 ************************************/
int MarkerDetector_Impl::perimeter(const vector<Point2f> &a)
{
  int sum = 0;
  for (unsigned int i = 0; i < a.size(); i++)
  {
    int i2 = (i + 1) % a.size();
    sum += static_cast<int>(sqrt((a[i].x - a[i2].x) * (a[i].x - a[i2].x) +
                                 (a[i].y - a[i2].y) * (a[i].y - a[i2].y)));
  }
  return sum;
}


/**
 */
void MarkerDetector_Impl::interpolate2Dline(const std::vector<cv::Point2f> &inPoints,
                                            cv::Point3f &outLine)
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
  Mat A(pointsCount, 2, CV_32FC1, Scalar(0));
  Mat B(pointsCount, 1, CV_32FC1, Scalar(0));
  Mat X;

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
    solve(A, B, X, DECOMP_SVD);
    // return Ax + By + C
    outLine = Point3f(X.at<float>(0, 0), -1., X.at<float>(1, 0));
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
    solve(A, B, X, DECOMP_SVD);
    // return Ax + By + C
    outLine = Point3f(-1., X.at<float>(0, 0), X.at<float>(1, 0));
  }
}

/**
 */
Point2f MarkerDetector_Impl::getCrossPoint(const cv::Point3f &line1, const cv::Point3f &line2)
{
  // create matrices of equation system
  Mat A(2, 2, CV_32FC1, Scalar(0));
  Mat B(2, 1, CV_32FC1, Scalar(0));
  Mat X;

  A.at<float>(0, 0) = line1.x;
  A.at<float>(0, 1) = line1.y;
  B.at<float>(0, 0) = -line1.z;

  A.at<float>(1, 0) = line2.x;
  A.at<float>(1, 1) = line2.y;
  B.at<float>(1, 0) = -line2.z;

  // solve system
  solve(A, B, X, DECOMP_SVD);
  return Point2f(X.at<float>(0, 0), X.at<float>(1, 0));
}

// template <typename T>
// void  MarkerDetector_Impl::cornerUpsample(vector<T>& MarkerCanditates, cv::Size
// lowResImageSize ){
//      cornerUpsample_SUBP(MarkerCanditates,lowResImageSize);
// }

// template <typename T>
// void MarkerDetector_Impl::cornerUpsample_SUBP(vector<T>& MarkerCanditates,   cv::Size
// lowResImageSize ){
//     if (MarkerCanditates.size()==0)return;
//     //first, determine the image in the pyramid nearest to this one
//     int startPyrImg=0;

//    for(size_t i=0;i<imagePyramid.size();i++){
//        if ( lowResImageSize.width < imagePyramid[i].cols) startPyrImg=i;
//        else break;
//    }
////#define _aruco_marker_detector_fast

//     cv::Size prevLowResSize=lowResImageSize;
//    for(int curpyr=startPyrImg;curpyr>=0;curpyr--){
//        float factor= float(imagePyramid[curpyr].cols)/float(prevLowResSize.width) ;
//        //upsample corner locations
//        for(auto &m:MarkerCanditates)
//            for(auto &point:m) {point*=factor;}
//        int halfwsize=  0.5+2.5*factor;
//             vector<cv::Point2f> p2d;//p2d.reserve(MarkerCanditates.size()*4);
//            for(auto &m:MarkerCanditates)
//                for(auto &point:m) { p2d.push_back(point);}
//              cv::cornerSubPix(
//              imagePyramid[curpyr],p2d,cv::Size(halfwsize,halfwsize),cv::Size(-1,-1),cv::TermCriteria(cv::TermCriteria::MAX_ITER
//              , 4,0.5));
//            int cidx=0;
//            for(auto &m:MarkerCanditates)
//                for(auto &point:m) {point =p2d[cidx++];}

//        prevLowResSize=imagePyramid[curpyr].size();
//    }
//}


/************************************
 *
 *
 *
 *
 ************************************/
void MarkerDetector_Impl::drawAllContours(Mat input, std::vector<std::vector<cv::Point>> &contours)
{
  drawContours(input, contours, -1, Scalar(255, 0, 255));
}

/************************************
 *
 *
 *
 *
 ************************************/
void MarkerDetector_Impl::drawContour(Mat &in, vector<Point> &contour, Scalar color)
{
  for (unsigned int i = 0; i < contour.size(); i++)
  {
    cv::rectangle(in, contour[i], contour[i], color);
  }
}

void MarkerDetector_Impl::drawApproxCurve(Mat &in, vector<Point> &contour, Scalar color, int thickness)
{
  for (unsigned int i = 0; i < contour.size(); i++)
  {
    cv::line(in, contour[i], contour[(i + 1) % contour.size()], color, thickness);
  }
}
/************************************
 *
 *
 *
 *
 ************************************/

void MarkerDetector_Impl::draw(Mat out, const vector<Marker> &markers)
{
  for (unsigned int i = 0; i < markers.size(); i++)
  {
    cv::line(out, markers[i][0], markers[i][1], cv::Scalar(255, 0, 0), 2);
    cv::line(out, markers[i][1], markers[i][2], cv::Scalar(255, 0, 0), 2);
    cv::line(out, markers[i][2], markers[i][3], cv::Scalar(255, 0, 0), 2);
    cv::line(out, markers[i][3], markers[i][0], cv::Scalar(255, 0, 0), 2);
  }
}


void MarkerDetector_Impl::setMarkerLabeler(cv::Ptr<MarkerLabeler> detector)
{
  markerIdDetector = detector;
}

void MarkerDetector_Impl::setDictionary(int dict_type, float error_correction_rate)
{
  markerIdDetector =
      MarkerLabeler::create((Dictionary::DICT_TYPES)dict_type, error_correction_rate);
  _params.error_correction_rate = error_correction_rate;
  _params.dictionary = aruco::Dictionary::getTypeString((Dictionary::DICT_TYPES)dict_type);
}



void MarkerDetector_Impl::setDictionary(string dict_type, float error_correction_rate)
{
  auto _to_string = [](float i)
  {
    std::stringstream str;
    str << i;
    return str.str();
  };
  _params.dictionary = dict_type;
  markerIdDetector = MarkerLabeler::create(dict_type, _to_string(error_correction_rate));
  _params.error_correction_rate = error_correction_rate;
}


cv::Mat MarkerDetector_Impl::getThresholdedImage(uint32_t idx)
{
  if (_thres_Images.size() == 0)
    return cv::Mat();
  if (idx >= _thres_Images.size())
    idx = _thres_Images.size() - 1;  // last one is the original image
  return _thres_Images[idx];
}



/**
 */
void MarkerDetector_Impl::distortPoints(vector<cv::Point2f> in, vector<cv::Point2f> &out,
                                        const Mat &camMatrix, const Mat &distCoeff)
{
  // trivial extrinsics
  cv::Mat Rvec = cv::Mat(3, 1, CV_32FC1, cv::Scalar::all(0));
  cv::Mat Tvec = Rvec.clone();
  // calculate 3d points and then reproject, so opencv makes the distortion internally
  vector<cv::Point3f> cornersPoints3d;
  for (unsigned int i = 0; i < in.size(); i++)
    cornersPoints3d.push_back(
        cv::Point3f((in[i].x - camMatrix.at<float>(0, 2)) / camMatrix.at<float>(0, 0),  // x
                    (in[i].y - camMatrix.at<float>(1, 2)) / camMatrix.at<float>(1, 1),  // y
                    1));  // z
  cv::projectPoints(cornersPoints3d, Rvec, Tvec, camMatrix, distCoeff, out);
}


/**Saves the configuration of the detector to a file
 */
void MarkerDetector_Impl::saveParamsToFile(const std::string &path) const
{
  cv::FileStorage fs(path, cv::FileStorage::WRITE);
  if (!fs.isOpened())
    throw std::runtime_error("Could not open " + path);
  _params.save(fs);
}

/**Loads the configuration from a file
 */
void MarkerDetector_Impl::loadParamsFromFile(const std::string &path)
{
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened())
    throw std::runtime_error("Could not open " + path);
  _params.load(fs);
  setDictionary(_params.dictionary, _params.error_correction_rate);
}

void MarkerDetector_Impl::toStream(std::ostream &str) const
{
  uint64_t sig = 13213;
  str.write((char *)&sig, sizeof(sig));
  _params.toStream(str);
}

void MarkerDetector_Impl::fromStream(std::istream &str)
{
  uint64_t sig = 13213;
  str.read((char *)&sig, sizeof(sig));
  if (sig != 13213)
    throw std::runtime_error("MarkerDetector_Impl::fromStream invalid signature");
  _params.fromStream(str);
  setDictionary(_params.dictionary, _params.error_correction_rate);
}



void MarkerDetector_Impl::filter_ambiguous_query(std::vector<cv::DMatch> &matches)
{
  if (matches.size() == 0)
    return;
  // determine maximum values of queryIdx
  int maxT = -1;
  for (auto m : matches)
    maxT = std::max(maxT, m.queryIdx);

  // now, create the vector with the elements
  vector<int> used(maxT + 1, -1);
  vector<cv::DMatch> best_matches(maxT);
  int idx = 0;
  bool needRemove = false;

  for (auto &match : matches)
  {
    if (used[match.queryIdx] == -1)
    {
      used[match.queryIdx] = idx;
    }
    else
    {
      if (matches[used[match.queryIdx]].distance > match.distance)
      {
        matches[used[match.queryIdx]].queryIdx = -1;  // annulate the other match
        used[match.queryIdx] = idx;
        needRemove = true;
      }
      else
      {
        match.queryIdx = -1;  // annulate this match
        needRemove = true;
      }
    }
    idx++;
  }


  if (needRemove)
    matches.erase(std::remove_if(matches.begin(), matches.end(),
                                 [](const cv::DMatch &m)
                                 { return m.trainIdx == -1 || m.queryIdx == -1; }),
                  matches.end());
}

void MarkerDetector_Impl::cornerUpsample(std::vector<Marker> &MarkerCanditates,
                                         cv::Size lowResImageSize)
{
  if (MarkerCanditates.size() == 0)
    return;
  // first, determine the image in the pyramid nearest to this one
  int startPyrImg = 0;

  for (size_t i = 0; i < imagePyramid.size(); i++)
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
    std::vector<cv::Point2f> p2d;  // p2d.reserve(MarkerCanditates.size()*4);
    for (auto &m : MarkerCanditates)
      for (auto &point : m)
      {
        p2d.push_back(point);
      }
    cv::cornerSubPix(imagePyramid[curpyr], p2d, cv::Size(halfwsize, halfwsize),
                     cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::MAX_ITER, 4, 0.5));
    int cidx = 0;
    for (auto &m : MarkerCanditates)
      for (auto &point : m)
      {
        point = p2d[cidx++];
      }

    prevLowResSize = imagePyramid[curpyr].size();
  }
}


void MarkerDetector_Impl::cornerUpsample(std::vector<std::vector<cv::Point2f>> &MarkerCanditates,
                                         cv::Size lowResImageSize)
{
  std::vector<Marker> markers;
  markers.reserve(MarkerCanditates.size());
  std::copy(MarkerCanditates.begin(), MarkerCanditates.end(), std::back_inserter(markers));

  cornerUpsample(markers, lowResImageSize);

  for (std::size_t i = 0; i < markers.size(); i++)
  {
    MarkerCanditates[i] = markers[i];
  }
}
};  // namespace aruco
