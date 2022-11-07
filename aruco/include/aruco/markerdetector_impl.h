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

#ifndef _ARUCO_MarkerDetector_Impl_H
#define _ARUCO_MarkerDetector_Impl_H

#include "aruco_export.h"
#include <opencv2/core/core.hpp>
#include <cstdio>
#include <iostream>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <map>
#include "marker.h"
#include "markerdetector.h"
#include <opencv2/imgproc/imgproc.hpp>

namespace aruco
{

class CameraParameters;
class MarkerLabeler;
/**\brief Main class for marker detection
 *
 */
class MarkerDetector_Impl
{
  friend class MarkerDetector;

public:
  /**
   * See
   */
  MarkerDetector_Impl();
  /**Creates indicating the dictionary. See @see  setDictionary for further details
   * @param dict_type Dictionary employed. See @see  setDictionary for further details
   * @param error_correction_rate value indicating the correction error allowed. Is in
   * range [0,1]. 0 means no correction at all. So an erroneous bit will result in
   * discarding the marker. 1, mean full correction. The maximum number of bits that can
   * be corrected depends on each ditionary. We recommend using values from 0 to 0.5. (in
   * general, this will allow up to 3 bits or correction).       */
  MarkerDetector_Impl(int dict_type, float error_correction_rate = 0);
  MarkerDetector_Impl(std::string dict_type, float error_correction_rate = 0);

  /**Saves the configuration of the detector to a file.
   */
  void saveParamsToFile(const std::string& path) const;

  /**Loads the configuration from a file.
   */
  void loadParamsFromFile(const std::string& path);


  /**
   */
  ~MarkerDetector_Impl();
  /**Specifies the detection mode. We have preset three types of detection modes. These
   * are ways to configure the internal parameters for the most typical situations. The
   * modes are:
   * - DM_NORMAL: In this mode, the full resolution image is employed for detection and
   * slow threshold method. Use this method when you process individual images that are
   * not part of a video sequence and you are not interested in speed.
   *
   * - DM_FAST: In this mode, there are two main improvements. First, image is threshold
   * using a faster method using a global threshold. Also, the full resolution image is
   * employed for detection, but, you could speed up detection even more by indicating a
   * minimum size of the markers you will accept. This is set by the variable
   * minMarkerSize which shoud be in range [0,1]. When it is 0, means that you do not set
   * a limit in the size of the accepted markers. However, if you set 0.1, it means that
   * markers smaller than 10% of the total image area, will not be detected. Then, the
   * detection can be accelated up to orders of magnitude compared to the normal mode.
   *
   * - DM_VIDEO_FAST: This is similar to DM_FAST, but specially adapted to video
   * processing. In that case, we assume that the observed markers when you call to
   * detect() have a size similar to the ones observed in the previous frame. Then, the
   * processing can be speeded up by employing smaller versions of the image automatically
   * calculated.
   *
   */
  void setDetectionMode(DetectionMode dm, float minMarkerSize = 0);
  /**returns current detection mode
   */
  DetectionMode getDetectionMode();
  /**Detects the markers in the image passed
   *
   * If you provide information about the camera parameters and the size of the marker,
   * then, the extrinsics of the markers are detected
   *
   * @param input input color image
   * @param camMatrix intrinsic camera information.
   * @param distCoeff camera distorsion coefficient. If set Mat() if is assumed no camera
   * distorion
   * @param markerSizeMeters size of the marker sides expressed in meters. If not
   * specified this value, the extrinsics of the markers are not detected.
   * @param setYPerperdicular If set the Y axis will be perpendicular to the surface.
   * Otherwise, it will be the Z axis
   * @param correctFisheye Correct fisheye distortion
   * @return vector with the detected markers
   */
  std::vector<aruco::Marker> detect(const cv::Mat& input);
  std::vector<aruco::Marker> detect(const cv::Mat& input, const CameraParameters& camParams,
                                    float markerSizeMeters, bool setYPerperdicular = false,
                                    bool correctFisheye = false);

  /**Detects the markers in the image passed
   *
   * If you provide information about the camera parameters and the size of the marker,
   * then, the extrinsics of the markers are detected
   *
   * @param input input color image
   * @param detectedMarkers output vector with the markers detected
   * @param camParams Camera parameters
   * @param markerSizeMeters size of the marker sides expressed in meters
   * @param setYPerperdicular If set the Y axis will be perpendicular to the surface.
   * Otherwise, it will be the Z axis
   * @param correctFisheye Correct fisheye distortion
   */
  void detect(const cv::Mat& input, std::vector<Marker>& detectedMarkers,
              CameraParameters camParams, float markerSizeMeters = -1,
              bool setYPerperdicular = false, bool correctFisheye = false);

  /**
   * Detects the markers in the image passed
   *
   * If you provide information about the camera parameters and the size of the marker,
   * then, the extrinsics of the markers are detected
   *
   * NOTE: be sure that the camera matrix is for this image size. If you do not know what
   * I am talking about, use functions above and not this one
   * @param input input color image
   * @param detectedMarkers output vector with the markers detected
   * @param camMatrix intrinsic camera information.
   * @param distCoeff camera distorsion coefficient. If set Mat() if is assumed no camera
   * distorion
   * @param extrinsics translation (tx,ty,tz) from right stereo camera to left. Empty if
   * no stereo or left camera
   * @param markerSizeMeters size of the marker sides expressed in meters
   * @param setYPerperdicular If set the Y axis will be perpendicular to the surface.
   * Otherwise, it will be the Z axis
   * @param correctFisheye Correct fisheye distortion
   */
  void detect(const cv::Mat& input, std::vector<Marker>& detectedMarkers,
              cv::Mat camMatrix = cv::Mat(), cv::Mat distCoeff = cv::Mat(),
              cv::Mat extrinsics = cv::Mat(), float markerSizeMeters = -1,
              bool setYPerperdicular = false, bool correctFisheye = false);


  /**Returns operating params
   */
  MarkerDetector::Params getParameters() const
  {
    return _params;
  }
  /**Returns operating params
   */
  MarkerDetector::Params& getParameters()
  {
    return _params;
  }
  /** Sets the dictionary to be employed.
   * You can choose:ARUCO,//original aruco dictionary. By default
                   ARUCO_MIP_25h7,
                   ARUCO_MIP_16h3,
                   ARUCO_MIP_36h12, **** recommended
                   ARTAG,//
                   ARTOOLKITPLUS,
                   ARTOOLKITPLUSBCH,//
                   TAG16h5,TAG25h7,TAG25h9,TAG36h11,TAG36h10//APRIL TAGS DICIONARIES
                   CHILITAGS,//chili tags dictionary . NOT RECOMMENDED. It has distance 0.
   Markers 806 and 682 should not be used!!!

    If dict_type is none of the above ones, it is assumed you mean a CUSTOM dicionary
   saved in a file @see Dictionary::loadFromFile Then, it tries to open it
  */
  void setDictionary(std::string dict_type, float error_correction_rate = 0);

  /**
   * @brief setDictionary Specifies the dictionary you want to use for marker decoding
   * @param dict_type dictionary employed for decoding markers @see Dictionary
   * @param error_correction_rate value indicating the correction error allowed. Is in
   * range [0,1]. 0 means no correction at all. So an erroneous bit will result in
   * discarding the marker. 1, mean full correction. The maximum number of bits that can
   * be corrected depends on each ditionary. We recommend using values from 0 to 0.5. (in
   * general, this will allow up to 3 bits or correction).
   */
  void setDictionary(int dict_type, float error_correction_rate = 0);

  /**
   * Returns a reference to the internal image thresholded. Since there can be generated
   * many of them, specify which
   */
  cv::Mat getThresholdedImage(uint32_t idx = 0);
  /**returns the number of thresholed images available
   */
  //   size_t getNhresholdedImages()const{return _thres_Images.size();}



  ///-------------------------------------------------
  /// Methods you may not need
  /// Thesde methods do the hard work. They have been set public in case you want to do customizations
  ///-------------------------------------------------

  /**
   * @brief setMakerLabeler sets the labeler employed to analyze the squares and extract
   * the inner binary code
   * @param detector
   */
  void setMarkerLabeler(cv::Ptr<MarkerLabeler> detector);
  cv::Ptr<MarkerLabeler> getMarkerLabeler()
  {
    return markerIdDetector;
  }
  typedef Marker MarkerCandidate;


  /**Returns a list candidates to be markers (rectangles), for which no valid id was found
   * after calling detectRectangles
   */
  std::vector<MarkerCandidate> getCandidates() const
  {
    return _candidates_wcontour;
  }

  /**
   * Given the iput image with markers, creates an output image with it in the canonical position
   * @param in input image
   * @param out image with the marker
   * @param size of out
   * @param points 4 corners of the marker in the image in
   * @return true if the operation succeed
   */
  bool warp(cv::Mat& in, cv::Mat& out, cv::Size size, std::vector<cv::Point2f> points);


  // serialization in binary mode
  void toStream(std::ostream& str) const;
  void fromStream(std::istream& str);
  // configure the detector from a set of parameters
  void setParameters(const MarkerDetector::Params& params);

  std::vector<cv::Mat> getImagePyramid()
  {
    return imagePyramid;
  }



private:
  // obfuscate start

  // operating params
  MarkerDetector::Params _params;

  // Images
  cv::Mat grey, thres;
  // pointer to the function that analizes a rectangular region so as to detect its internal marker
  cv::Ptr<MarkerLabeler> markerIdDetector;
  /**
   */
  int perimeter(const std::vector<cv::Point2f>& a);

  // auxiliar functions to perform LINES refinement
  void interpolate2Dline(const std::vector<cv::Point2f>& inPoints, cv::Point3f& outLine);
  cv::Point2f getCrossPoint(const cv::Point3f& line1, const cv::Point3f& line2);
  void distortPoints(std::vector<cv::Point2f> in, std::vector<cv::Point2f>& out,
                     const cv::Mat& camMatrix, const cv::Mat& distCoeff);

  // returns the number of pixels that the smallest and largest allowed markers have
  int getMinMarkerSizePix(cv::Size orginput_imageSize) const;

  // returns the markerWarpSize
  int getMarkerWarpSize();
  /**Given a vector vinout with elements and a boolean vector indicating the lements from
   * it to remove, this function remove the elements
   * @param vinout
   * @param toRemove
   */
  template <typename T>
  void removeElements(std::vector<T>& vinout, const std::vector<bool>& toRemove)
  {
    // remove the invalid ones by setting the valid in the positions left by the invalids
    size_t indexValid = 0;
    for (size_t i = 0; i < toRemove.size(); i++)
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


  template <typename T>
  void joinVectors(std::vector<std::vector<T>>& vv, std::vector<T>& v, bool clearv = false)
  {
    if (clearv)
      v.clear();
    for (size_t i = 0; i < vv.size(); i++)
      for (size_t j = 0; j < vv[i].size(); j++)
        v.push_back(vv[i][j]);
  }

  std::vector<cv::Mat> imagePyramid;
  void enlargeMarkerCandidate(MarkerCandidate& cand, int fact = 1);

  void cornerUpsample(std::vector<std::vector<cv::Point2f>>& MarkerCanditates,
                      cv::Size lowResImageSize);
  void cornerUpsample(std::vector<Marker>& MarkerCanditates, cv::Size lowResImageSize);



  void buildPyramid(std::vector<cv::Mat>& imagePyramid, const cv::Mat& grey, int minSize);



  std::vector<aruco::MarkerCandidate> thresholdAndDetectRectangles(const cv::Mat& input,
                                                                   int thres_param1,
                                                                   int thres_param2, bool erode,
                                                                   cv::Mat& auxThresImage);
  std::vector<aruco::MarkerCandidate> thresholdAndDetectRectangles(const cv::Mat& image);
  std::vector<aruco::MarkerCandidate> prefilterCandidates(std::vector<MarkerCandidate>& candidates,
                                                          cv::Size orgImageSize);


  std::vector<cv::Mat> _thres_Images;
  std::vector<std::vector<MarkerCandidate>> _vcandidates;
  // std::vector<std::vector<cv::Point2f > > _candidates;
  std::vector<MarkerCandidate> _candidates_wcontour;
  std::vector<Marker> _prevMarkers;       // employed for tracking
  std::map<int, int> marker_ndetections;  // used to keeps track only of markers with a
                                          // minimum number of detections

  // graphical debug
  void drawApproxCurve(cv::Mat& in, std::vector<cv::Point>& approxCurve, cv::Scalar color,
                       int thickness = 1);
  void drawContour(cv::Mat& in, std::vector<cv::Point>& contour, cv::Scalar);
  void drawAllContours(cv::Mat input, std::vector<std::vector<cv::Point>>& contours);
  void draw(cv::Mat out, const std::vector<Marker>& markers);



  enum ThreadTasks
  {
    THRESHOLD_TASK,
    ENCLOSE_TASK,
    EXIT_TASK
  };
  struct ThresAndDetectRectTASK
  {
    int inIdx, outIdx;
    int param1, param2;
    ThreadTasks task;
  };
  void thresholdAndDetectRectangles_thread();

  // thread safe queue to implement producer-consumer
  template <typename T>
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

    size_t size()
    {
      std::unique_lock<std::mutex> mlock(mutex_);
      size_t s = queue_.size();
      return s;
    }

  private:
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
  };
  Queue<ThresAndDetectRectTASK> _tasks;
  void refineCornerWithContourLines(aruco::Marker& marker, cv::Mat cameraMatrix = cv::Mat(),
                                    cv::Mat distCoef = cv::Mat());

  inline float pointSqdist(cv::Point& p, cv::Point2f& p2)
  {
    float dx = p.x - p2.x;
    float dy = p.y - p2.y;
    return dx * dx + dy * dy;
  }

  float _tooNearDistance = -1;  // pixel distance between nearr rectangle. Computed
                                // automatically based on the params



  struct marker_analyzer
  {
    void setParams(const std::vector<cv::Point2f>& m)
    {
      corners = m;
      bax = m[1].x - m[0].x;
      bay = m[1].y - m[0].y;
      dax = m[2].x - m[0].x;
      day = m[2].y - m[0].y;
      a = m[0];
      b = m[1];
      d = m[2];
      area = _getArea();

      center = cv::Point2f(0, 0);
      for (auto& p : corners)
        center += p;
      center *= 1. / 4.;
    }

    bool isInto(const cv::Point2f& p) const
    {
      if (signD(corners[0], corners[1], p) < 0)
        return false;
      if (signD(corners[1], corners[2], p) < 0)
        return false;
      if (signD(corners[2], corners[3], p) < 0)
        return false;
      if (signD(corners[3], corners[0], p) < 0)
        return false;
      return true;
    }
    cv::Point2f getCenter() const
    {
      return center;
    }
    float getArea() const
    {
      return area;
    }
    float _getArea() const
    {
      // use the cross products
      cv::Point2f v01 = corners[1] - corners[0];
      cv::Point2f v03 = corners[3] - corners[0];
      float area1 = fabs(v01.x * v03.y - v01.y * v03.x);
      cv::Point2f v21 = corners[1] - corners[2];
      cv::Point2f v23 = corners[3] - corners[2];
      float area2 = fabs(v21.x * v23.y - v21.y * v23.x);
      return (area2 + area1) / 2.f;
    }
    float bax, bay, dax, day;
    cv::Point2f a, b, d;
    float area;
    cv::Point2f center;

    std::vector<cv::Point2f> corners;

    inline float signD(cv::Point2f p0, cv::Point2f p1, cv::Point2f p) const
    {
      return ((p0.y - p1.y) * p.x + (p1.x - p0.x) * p.y + (p0.x * p1.y - p1.x * p0.y)) /
             sqrt((p1.x - p0.x) * (p1.x - p0.x) + (p1.y - p0.y) * (p1.y - p0.y));
    }
  };
  void addToImageHist(cv::Mat& im, std::vector<float>& hist);
  int Otsu(std::vector<float>& hist);

  void filter_ambiguous_query(std::vector<cv::DMatch>& matches);
  // obfuscate end
};
};  // namespace aruco
#endif
