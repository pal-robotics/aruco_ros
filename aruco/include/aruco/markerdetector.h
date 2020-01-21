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

#ifndef _ARUCO_MarkerDetector_H
#define _ARUCO_MarkerDetector_H

#include "aruco_export.h"
#include <opencv2/core.hpp>
#include <cstdio>
#include <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <vector>
#include "marker.h"

namespace aruco
{

/**
 * @brief The DetectionMode enum defines the different possibilities for detection
 *
 */
enum DetectionMode
  : int
  { DM_NORMAL = 0, DM_FAST = 1, DM_VIDEO_FAST = 2
};

class CameraParameters;
class MarkerLabeler;

/**
 * \brief Main class for marker detection
 */
class ARUCO_EXPORT MarkerDetector
{
public:
  enum ThresMethod
    : int
    { THRES_ADAPTIVE = 0, THRES_AUTO_FIXED = 1
  };

  /**
   * Operating params
   */
  struct Params
  {

    // maximum number of parallel threads
    int maxThreads = 1; // -1 means all

    // border around image limits in which corners are not allowed to be detected. (0,1)
    float borderDistThres = 0.015f;
    int lowResMarkerSize = 20; // minimum size of a marker in the low resolution image

    // minimum  size of a contour length. We use the following formula
    // minLenght = min(_minSize_pix, _minSize * Is) * 4
    // being Is = max(imageWidth, imageHeight)
    // the value _minSize are normalized, thus, depends on camera image size
    // however, _minSize_pix is expressed in pixels (you can use the one you prefer)
    float minSize = -1; // tau_i in paper
    int minSize_pix = -1;
    bool enclosedMarker = false; // special treatment for enclosed markers


    void setThresholdMethod(ThresMethod method, int thresHold = -1, int wsize = 15, int wsize_range = 0)
    {
      _AdaptiveThresWindowSize = wsize;
      _thresMethod = method;
      if (thresHold == -1)
      {
        if (method == THRES_AUTO_FIXED)
          _ThresHold = 100;
        else
          _ThresHold = 10;
      }
      else
        _ThresHold = thresHold;
      _AdaptiveThresWindowSize_range = wsize_range;
    }

    // threshold methods
    ThresMethod _thresMethod = THRES_ADAPTIVE;
    int NAttemptsAutoThresFix = 3; // number of times that tries a random threshold in case of THRES_AUTO_FIXED

    // threshold parameters
    int _AdaptiveThresWindowSize = 15, _ThresHold = 10, _AdaptiveThresWindowSize_range = 0;
    // size of the image passed to the MarkerLabeler
    int _markerWarpPixSize = 5; // tau_c in paper

    // enable/disables the method for automatic size estimation for speed up
    bool _autoSize = false;
    float _ts = 0.25f; // $\tau_s$ is a factor in the range $(0,1]$ that accounts for the camera motion speed. For instance, when $\tau_s=0.1$, it means that in the next frame, $\tau_i$ is such that markers $10\%$ smaller than the smallest marker in the current image  will be seek. To avoid loosing track of the markers. If no markers are detected in a frame, $\tau_i$ is set to zero for the next frame so that markers of any size can be detected.

    /**
     * Enables automatic image resize according to elements detected in previous frame
     * @param v
     * @param ts is a factor in the range $(0,1]$ that accounts for the camera motion speed. For instance, when ts=0.1, it means that in the next frame, $\tau_i$ is such that markers $10\%$ smaller than the smallest marker in the current image  will be seek. To avoid loosing track of the markers.
     */
    void setAutoSizeSpeedUp(bool v, float ts = 0.25)
    {
      _autoSize = v;
      _ts = ts;
    }

    bool getAutoSizeSpeedUp() const
    {
      return _autoSize;
    }

    float pyrfactor = 2;
  };

  /**
   * See
   */
  MarkerDetector();

  /**
   * Creates indicating the dictionary. See @see  setDictionary for further details
   * @param dict_type Dictionary employed. See @see  setDictionary for further details
   * @param error_correction_rate value indicating the correction error allowed. Is in range [0,1]. 0 means no
   * correction at all. So
   * an erroneous bit will result in discarding the marker. 1, mean full correction. The maximum number of bits
   * that can be corrected depends on each dictionary.
   * We recommend using values from 0 to 0.5. (in general, this will allow up to 3 bits or correction).       */
  MarkerDetector(int dict_type, float error_correction_rate = 0);
  MarkerDetector(std::string dict_type, float error_correction_rate = 0);

  /**
   */
  ~MarkerDetector();

  /**
   * Specifies the detection mode. We have preset three types of detection modes. These are
   * ways to configure the internal parameters for the most typical situations. The modes are:
   * - DM_NORMAL: In this mode, the full resolution image is employed for detection and slow threshold method. Use this method when
   * you process individual images that are not part of a video sequence and you are not interested in speed.
   *
   * - DM_FAST: In this mode, there are two main improvements. First, image is threshold using a faster method using a global threshold.
   * Also, the full resolution image is employed for detection, but, you could speed up detection even more by indicating a minimum size of the
   * markers you will accept. This is set by the variable minMarkerSize which should be in range [0,1]. When it is 0, means that you do not set
   * a limit in the size of the accepted markers. However, if you set 0.1, it means that markers smaller than 10% of the total image area, will not
   * be detected. Then, the detection can be accelerated up to orders of magnitude compared to the normal mode.
   *
   * - DM_VIDEO_FAST: This is similar to DM_FAST, but specially adapted to video processing. In that case, we assume that the observed markers
   * when you call to detect() have a size similar to the ones observed in the previous frame. Then, the processing can be speeded up by employing smaller versions
   * of the image automatically calculated.
   */
  void setDetectionMode(DetectionMode dm, float minMarkerSize = 0);

  /**
   * Returns current detection mode
   */
  DetectionMode getDetectionMode();

  /**
   * Detects the markers in the image passed
   *
   * If you provide information about the camera parameters and the size of the marker, then, the extrinsics of
   * the markers are detected
   *
   * @param input input color image
   * @param camMatrix intrinsic camera information.
   * @param distCoeff camera distortion coefficient. If set Mat() if is assumed no camera distortion
   * @param markerSizeMeters size of the marker sides expressed in meters. If not specified this value, the
   * extrinsics of the markers are not detected.
   * @param setYPerperdicular If set the Y axis will be perpendicular to the surface. Otherwise, it will be the Z
   * axis
   * @param correctFisheye Correct fisheye distortion
   * @return vector with the detected markers
   */
  std::vector<aruco::Marker> detect(const cv::Mat& input);
  std::vector<aruco::Marker> detect(const cv::Mat& input, const CameraParameters& camParams, float markerSizeMeters,
                                    bool setYPerperdicular = false, bool correctFisheye = false);

  /**
   * Detects the markers in the image passed
   *
   * If you provide information about the camera parameters and the size of the marker, then, the extrinsics of
   * the markers are detected
   *
   * @param input input color image
   * @param detectedMarkers output vector with the markers detected
   * @param camParams Camera parameters
   * @param markerSizeMeters size of the marker sides expressed in meters
   * @param setYPerperdicular If set the Y axis will be perpendicular to the surface. Otherwise, it will be the Z axis
   * @param correctFisheye Correct fisheye distortion
   */
  void detect(const cv::Mat& input, std::vector<Marker>& detectedMarkers, CameraParameters camParams,
              float markerSizeMeters = -1, bool setYPerperdicular = false, bool correctFisheye = false);

  /**
   * Detects the markers in the image passed
   *
   * If you provide information about the camera parameters and the size of the marker, then, the extrinsics of
   * the markers are detected
   *
   * NOTE: be sure that the camera matrix is for this image size. If you do not know what I am talking about, use
   * functions above and not this one
   * @param input input color image
   * @param detectedMarkers output vector with the markers detected
   * @param camMatrix intrinsic camera information.
   * @param distCoeff camera distortion coefficient. If set Mat() if is assumed no camera distortion
   * @param extrinsics translation (tx,ty,tz) from right stereo camera to left. Empty if no stereo or left camera
   * @param markerSizeMeters size of the marker sides expressed in meters
   * @param setYPerperdicular If set the Y axis will be perpendicular to the surface. Otherwise, it will be the Z axis
   * @param correctFisheye Correct fisheye distortion
   */
  void detect(const cv::Mat& input, std::vector<Marker>& detectedMarkers, cv::Mat camMatrix = cv::Mat(),
              cv::Mat distCoeff = cv::Mat(), cv::Mat extrinsics = cv::Mat(), float markerSizeMeters = -1, bool setYPerperdicular = false, bool correctFisheye = false);

//  /**
//   * Sets operating params
//   */
//  void setParams(Params p);

  /**
   * Returns operating params
   */
  Params getParameters() const
  {
    return _params;
  }

  /**
   * Returns operating params
   */
  Params & getParameters()
  {
    return _params;
  }

  /**
   * Sets the dictionary to be employed.
   * You can choose:ARUCO, //original aruco dictionary. By default
   * ARUCO_MIP_25h7,
   * ARUCO_MIP_16h3,
   * ARUCO_MIP_36h12, **** recommended
   * ARTAG, //
   * ARTOOLKITPLUS,
   * ARTOOLKITPLUSBCH, //
   * TAG16h5,TAG25h7,TAG25h9,TAG36h11,TAG36h10//APRIL TAGS DICIONARIES
   * CHILITAGS,//chili tags dictionary . NOT RECOMMENDED. It has distance 0. Markers 806 and 682
   * should not be used!!!
   *
   * If dict_type is none of the above ones, it is assumed you mean a CUSTOM dictionary saved in a file @see Dictionary::loadFromFile
   * Then, it tries to open it
   */
  void setDictionary(std::string dict_type, float error_correction_rate = 0);

  /**
   * @brief setDictionary Specifies the dictionary you want to use for marker decoding
   * @param dict_type dictionary employed for decoding markers @see Dictionary
   * @param error_correction_rate value indicating the correction error allowed. Is in range [0,1]. 0 means no
   * correction at all. So
   * an erroneous bit will result in discarding the marker. 1, mean full correction. The maximum number of bits
   * that can be corrected depends on each dictionary.
   * We recommend using values from 0 to 0.5. (in general, this will allow up to 3 bits or correction).
   */
  void setDictionary(int dict_type, float error_correction_rate = 0);

  /**
   * Returns a reference to the internal image thresholded. Since there can be generated many of them, specify which
   */
  cv::Mat getThresholdedImage(uint32_t idx = 0);

  ///-------------------------------------------------
  /// Methods you may not need
  /// These methods do the hard work. They have been set public in case you want to do customizations
  ///-------------------------------------------------

  /**
   * @brief setMakerLabeler sets the labeler employed to analyze the squares and extract the inner binary code
   * @param detector
   */
  void setMarkerLabeler(cv::Ptr<MarkerLabeler> detector);
  cv::Ptr<MarkerLabeler> getMarkerLabeler()
  {
    return markerIdDetector;
  }
  // Represent a candidate to be a maker
  class MarkerCandidate : public Marker
  {
  public:
    MarkerCandidate()
    {
    }

    MarkerCandidate(const Marker& M) :
        Marker(M)
    {
    }

    MarkerCandidate(const MarkerCandidate& M) :
        Marker(M)
    {
      contour = M.contour;
      idx = M.idx;
    }

    MarkerCandidate& operator=(const MarkerCandidate& M)
    {
      (*(Marker*)this) = (*(Marker*)&M);
      contour = M.contour;
      idx = M.idx;
      return *this;
    }

    vector<cv::Point> contour; // all the points of its contour
    int idx; // index position in the global contour list
  };

//  /**
//   * Detection of candidates to be markers, i.e., rectangles.
//   * This function returns in candidates all the rectangles found in a thresholded image
//   */
//  void detectRectangles(const ThresImage &thresImg, vector<std::vector<cv::Point2f>>& candidates);

  /**
   * Returns a list candidates to be markers (rectangles), for which no valid id was found after calling
   * detectRectangles
   */
  std::vector<std::vector<cv::Point2f>> getCandidates() const
  {
    return _candidates;
  }

  /**
   * Given the input image with markers, creates an output image with it in the canonical position
   * @param in input image
   * @param out image with the marker
   * @param size of out
   * @param points 4 corners of the marker in the image in
   * @return true if the operation succeed
   */
  bool warp(cv::Mat& in, cv::Mat& out, cv::Size size, std::vector<cv::Point2f> points);

private:
  // operating params
  Params _params;
  DetectionMode _detectMode = DM_NORMAL;

  // images
  cv::Mat grey, thres;

  // pointer to the function that analyzes a rectangular region so as to detect its internal marker
  cv::Ptr<MarkerLabeler> markerIdDetector;

  /**
   */
  int perimeter(const std::vector<cv::Point2f> &a);

  // auxiliary functions to perform LINES refinement
  void interpolate2Dline(const std::vector<cv::Point2f>& inPoints, cv::Point3f& outLine);
  cv::Point2f getCrossPoint(const cv::Point3f& line1, const cv::Point3f& line2);
  void distortPoints(std::vector<cv::Point2f> in, std::vector<cv::Point2f>& out, const cv::Mat& camMatrix,
                     const cv::Mat& distCoeff);

  // returns the number of pixels that the smallest and largest allowed markers have
  int getMinMarkerSizePix(cv::Size orginput_imageSize) const;

  // returns the markerWarpSize
  int getMarkerWarpSize();

  /**
   * Given a vector vinout with elements and a boolean vector indicating the elements from it to remove,
   * this function remove the elements
   * @param vinout
   * @param toRemove
   */
  template<typename T>
  void removeElements(std::vector<T>& vinout, const std::vector<bool>& toRemove)
  {
    // remove the invalid ones by setting the valid in the positions left by the invalids
    std::size_t indexValid = 0;
    for (std::size_t i = 0; i < toRemove.size(); i++)
    {
      if (!toRemove[i])
      {
        if (indexValid != i)
          vinout[indexValid] = vinout[i];
        indexValid++;
      }
    }
    vinout.resize(indexValid);
  }

  template<typename T>
  void joinVectors(std::vector<std::vector<T>>& vv, std::vector<T>& v, bool clearv = false)
  {
    if (clearv)
      v.clear();
    for (std::size_t i = 0; i < vv.size(); i++)
      for (std::size_t j = 0; j < vv[i].size(); j++)
        v.push_back(vv[i][j]);
  }

  std::vector<cv::Mat> imagePyramid;
  void enlargeMarkerCandidate(MarkerCandidate &cand, int fact = 1);

  void cornerUpsample(std::vector<Marker>& MarkerCanditates, cv::Size lowResImageSize);
  void cornerUpsample_SUBP(std::vector<Marker>& MarkerCanditates, cv::Size lowResImageSize);

  void buildPyramid(std::vector<cv::Mat> &imagePyramid, const cv::Mat &grey, int minSize);

  std::vector<aruco::MarkerDetector::MarkerCandidate> thresholdAndDetectRectangles(const cv::Mat & input,
                                                                                   int thres_param1, int thres_param2,
                                                                                   bool erode, cv::Mat &auxThresImage);
  std::vector<aruco::MarkerDetector::MarkerCandidate> thresholdAndDetectRectangles(const cv::Mat &image);
  std::vector<aruco::MarkerDetector::MarkerCandidate> prefilterCandidates(std::vector<MarkerCandidate> &candidates,
                                                                          cv::Size orgImageSize);

  std::vector<cv::Mat> _thres_Images;
  std::vector<std::vector<MarkerCandidate> > _vcandidates;
  std::vector<std::vector<cv::Point2f> > _candidates;

  // graphical debug
  void drawApproxCurve(cv::Mat& in, std::vector<cv::Point>& approxCurve, cv::Scalar color, int thickness = 1);
  void drawContour(cv::Mat& in, std::vector<cv::Point>& contour, cv::Scalar);
  void drawAllContours(cv::Mat input, std::vector<std::vector<cv::Point>>& contours);
  void draw(cv::Mat out, const std::vector<Marker>& markers);

  enum ThreadTasks
  {
    THRESHOLD_TASK, ERODE_TASK, EXIT_TASK
  };
  struct ThresAndDetectRectTASK
  {
    int inIdx, outIdx;
    int param1, param2;
    ThreadTasks task;
  };
  void thresholdAndDetectRectangles_thread();

  // thread safe queue to implement producer-consumer
  template<typename T>
  class Queue
  {
  public:
    T pop()
    {
      std::unique_lock<std::mutex> mlock(mutex_);
      while (queue_.empty())
      {
        cond_.wait(mlock);
      }
      auto item = queue_.front();
      queue_.pop();
      return item;
    }

    void push(const T& item)
    {
      std::unique_lock<std::mutex> mlock(mutex_);
      queue_.push(item);
      mlock.unlock();
      cond_.notify_one();
    }

    std::size_t size()
    {
      std::unique_lock<std::mutex> mlock(mutex_);
      std::size_t s = queue_.size();
      return s;
    }

  private:
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
  };

  Queue<ThresAndDetectRectTASK> _tasks;
};

} // namespace aruco

#endif /* _ARUCO_MarkerDetector_H */
