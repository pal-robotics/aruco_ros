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

#ifndef _ARUCO_MarkerDetector_H
#define _ARUCO_MarkerDetector_H

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

#include <opencv2/imgproc/imgproc.hpp>

namespace aruco
{

/**
 * @brief The DetectionMode enum defines the different possibilities for detection.
 * Specifies the detection mode. We have preset three types of detection modes. These are
 * ways to configure the internal parameters for the most typical situations. The modes
 * are:
 * - DM_NORMAL: In this mode, the full resolution image is employed for detection and slow
 * threshold method. Use this method when you process individual images that are not part
 * of a video sequence and you are not interested in speed.
 *
 * - DM_FAST: In this mode, there are two main improvements. First, image is threshold
 * using a faster method using a global threshold. Also, the full resolution image is
 * employed for detection, but, you could speed up detection even more by indicating a
 * minimum size of the markers you will accept. This is set by the variable minMarkerSize
 * which shoud be in range [0,1]. When it is 0, means that you do not set a limit in the
 * size of the accepted markers. However, if you set 0.1, it means that markers smaller
 * than 10% of the total image area, will not be detected. Then, the detection can be
 * accelated up to orders of magnitude compared to the normal mode.
 *
 * - DM_VIDEO_FAST: This is similar to DM_FAST, but specially adapted to video processing.
 * In that case, we assume that the observed markers when you call to detect() have a size
 * similar to the ones observed in the previous frame. Then, the processing can be speeded
 * up by employing smaller versions of the image automatically calculated.
 *
 */
enum DetectionMode : int
{
  DM_NORMAL = 0,
  DM_FAST = 1,
  DM_VIDEO_FAST = 2
};
/** Method employed to refine the estimation of the corners
 * - CORNER_SUBPIX: uses subpixel refinement implemented in opencv
 * - CORNER_LINES: uses all the pixels in the corner border to estimate the 4 lines of the square. Then
 *  estimate the point in which they intersect. In seems that it more robust to noise. However,
 * it only works if input image is not resized. So, the value minMarkerSize will be set to 0.
 *
 * - CORNER_NONE: Does no refinement of the corner. Again, it requires minMakerSize to be 0
 */
enum CornerRefinementMethod : int
{
  CORNER_SUBPIX = 0,
  CORNER_LINES = 1,
  CORNER_NONE = 2
};


class CameraParameters;
class MarkerLabeler;
class MarkerDetector_Impl;
typedef Marker MarkerCandidate;

/**\brief Main class for marker detection
 *
 */

class ARUCO_EXPORT MarkerDetector
{
  friend class MarkerDetector_Impl;

public:
  enum ThresMethod : int
  {
    THRES_ADAPTIVE = 0,
    THRES_AUTO_FIXED = 1
  };

  /**Operating params
   */
  struct ARUCO_EXPORT Params
  {
    /**Specifies the detection mode. We have preset three types of detection modes. These are
     * ways to configure the internal parameters for the most typical situations. The modes are:
     * - DM_NORMAL: In this mode, the full resolution image is employed for detection and
     * slow threshold method. Use this method when you process individual images that are
     * not part of a video sequence and you are not interested in speed.
     *
     * - DM_FAST: In this mode, there are two main improvements. First, image is threshold
     * using a faster method using a global threshold. Also, the full resolution image is
     * employed for detection, but, you could speed up detection even more by indicating a minimum size of the
     * markers you will accept. This is set by the variable minMarkerSize which shoud be
     * in range [0,1]. When it is 0, means that you do not set a limit in the size of the
     * accepted markers. However, if you set 0.1, it means that markers smaller than 10%
     * of the total image area, will not be detected. Then, the detection can be accelated
     * up to orders of magnitude compared to the normal mode.
     *
     * - DM_VIDEO_FAST: This is similar to DM_FAST, but specially adapted to video processing.
     * In that case, we assume that the observed markers when you call to detect() have a
     * size similar to the ones observed in the previous frame. Then, the processing can
     * be speeded up by employing smaller versions of the image automatically calculated.
     *
     */
    void setDetectionMode(DetectionMode dm, float minMarkerSize);

    /**Enables/Disbles the detection of enclosed markers. Enclosed markers are markers
     * where corners are like opencv chessboard pattern
     */
    void detectEnclosedMarkers(bool do_)
    {
      enclosedMarker = do_;
    }

    /**Sets the corner refinement method
     * - CORNER_SUBPIX: uses subpixel refinement implemented in opencv
     * - CORNER_LINES: uses all the pixels in the corner border to estimate the 4 lines of
     * the square. Then estimate the point in which they intersect. In seems that it more
     * robust to noise. However, it only works if input image is not resized. So, the
     * value minMarkerSize will be set to 0.
     *
     * - CORNER_NONE: Does no refinement of the corner. Again, it requires minMakerSize to
     * be 0
     */
    void setCornerRefinementMethod(CornerRefinementMethod method);


    //-----------------------------------------------------------------------------
    // Below this point you probably should not use the functions
    /**Sets the thresholding method manually. Do no
     */
    void setThresholdMethod(ThresMethod method, int thresHold = -1, int wsize = -1,
                            int wsize_range = 0);



    void setAutoSizeSpeedUp(bool v, float Ts = 0.25)
    {
      autoSize = v;
      ts = Ts;
    }
    bool getAutoSizeSpeedUp() const
    {
      return autoSize;
    }



    void save(cv::FileStorage &fs) const;
    void load(cv::FileStorage &fs);

    void toStream(std::ostream &str) const;
    void fromStream(std::istream &str);

    static std::string toString(DetectionMode dm);
    static DetectionMode getDetectionModeFromString(const std::string &str);
    static std::string toString(CornerRefinementMethod dm);
    static CornerRefinementMethod getCornerRefinementMethodFromString(const std::string &str);
    static std::string toString(ThresMethod dm);
    static ThresMethod getCornerThresMethodFromString(const std::string &str);

    // Detection mode

    DetectionMode detectMode = DM_NORMAL;

    // maximum number of parallel threads
    int maxThreads = 1;  //-1 means all

    // border around image limits in which corners are not allowed to be detected. (0,1)
    float borderDistThres = 0.015f;
    int lowResMarkerSize = 20;  // minimum size of a marker in the low resolution image

    // minimum  size of a contour lenght. We use the following formula
    // minLenght=  min ( _minSize_pix , _minSize* Is)*4
    // being Is=max(imageWidth,imageHeight)
    // the value  _minSize are normalized, thus, depends on camera image size
    // However, _minSize_pix is expressed in pixels (you can use the one you prefer)
    float minSize = -1;  // tau_i in paper
    int minSize_pix = -1;
    bool enclosedMarker = false;  // special treatment for enclosed markers
    float error_correction_rate = 0;
    std::string dictionary = "ALL_DICTS";
    // threshold methods
    ThresMethod thresMethod = THRES_ADAPTIVE;
    int NAttemptsAutoThresFix =
        3;  // number of times that tries a random threshold in case of THRES_AUTO_FIXED
    int trackingMinDetections = 0;  // no tracking


    // Threshold parameters
    int AdaptiveThresWindowSize = -1, ThresHold = 7, AdaptiveThresWindowSize_range = 0;
    // size of the image passedta to the MarkerLabeler
    int markerWarpPixSize = 5;  // tau_c in paper

    CornerRefinementMethod cornerRefinementM = CORNER_SUBPIX;
    // enable/disables the method for automatic size estimation for speed up
    bool autoSize = false;
    float ts =
        0.25f;  //$\tau_s$ is a factor in the range $(0,1]$ that accounts for the camera
                //motion speed. For instance, when $\tau_s=0.1$, it means that in the next
                //frame, $\tau_i$ is such that markers $10\%$ smaller than the smallest
                //marker in the current image  will be seek. To avoid loosing track of the
                //markers. If no markers are detected in a frame, $\tau_i$ is set to zero
                //for the next frame so that markers of any size can be detected.
    /**Enables automatic image resize according to elements detected in previous frame
     * @param v
     * @param ts  is a factor in the range $(0,1]$ that accounts for the camera motion
     * speed. For instance, when ts=0.1 , it means that in the next frame, $\tau_i$ is
     * such that markers $10\%$ smaller than the smallest marker in the current image will
     * be seek. To avoid loosing track of the markers.
     */
    float pyrfactor = 2;
    int closingSize =
        0;  // enables/disables morph closing operation. The actual param used is closingSize*2+1

  private:
    static void _toStream(const std::string &strg, std::ostream &str);
    static void _fromStream(std::string &strg, std::istream &str);
    template <typename Type>
    static bool attemtpRead(const std::string &name, Type &var, cv::FileStorage &fs)
    {
      if (fs[name].type() != cv::FileNode::NONE)
      {
        fs[name] >> var;
        return true;
      }
      return false;
    }
  };

  /**
   * See
   */
  MarkerDetector();
  /**Creates indicating the dictionary. See @see  setDictionary for further details
   * @param dict_type Dictionary employed. See @see  setDictionary for further details
   * @param error_correction_rate value indicating the correction error allowed. Is in
   * range [0,1]. 0 means no correction at all. So an erroneous bit will result in
   * discarding the marker. 1, mean full correction. The maximum number of bits that can
   * be corrected depends on each ditionary. We recommend using values from 0 to 0.5. (in
   * general, this will allow up to 3 bits or correction).       */
  MarkerDetector(int dict_type, float error_correction_rate = 0);
  MarkerDetector(std::string dict_type, float error_correction_rate = 0);

  /**Saves the configuration of the detector to a file.
   */
  void saveParamsToFile(const std::string &path) const;

  /**Loads the configuration from a file.
   */
  void loadParamsFromFile(const std::string &path);


  /**
   */
  ~MarkerDetector();
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
  std::vector<aruco::Marker> detect(const cv::Mat &input);
  std::vector<aruco::Marker> detect(const cv::Mat &input, const CameraParameters &camParams,
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
  void detect(const cv::Mat &input, std::vector<Marker> &detectedMarkers,
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
   * @param distCoeff camera distortion coefficient. If set Mat() if is assumed no camera
   * distortion
   * @param extrinsics translation (tx,ty,tz) from right stereo camera to left. Empty if
   * no stereo or left camera
   * @param markerSizeMeters size of the marker sides expressed in meters
   * @param setYPerperdicular If set the Y axis will be perpendicular to the surface.
   * Otherwise, it will be the Z axis
   * @param correctFisheye Correct fisheye distortion
   */
  void detect(const cv::Mat &input, std::vector<Marker> &detectedMarkers,
              cv::Mat camMatrix = cv::Mat(), cv::Mat distCoeff = cv::Mat(),
              cv::Mat extrinsics = cv::Mat(), float markerSizeMeters = -1,
              bool setYPerperdicular = false, bool correctFisheye = false);

  /**Returns operating params
   */
  Params getParameters() const;
  /**Returns operating params
   */
  Params &getParameters();
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
  cv::Ptr<MarkerLabeler> getMarkerLabeler();


  /**Returns a list candidates to be markers (rectangles), for which no valid id was found
   * after calling detectRectangles
   */
  std::vector<MarkerCandidate> getCandidates() const;

  std::vector<cv::Mat> getImagePyramid();
  /*
   * @param corners vectors of vectors
   */
  void cornerUpsample(std::vector<std::vector<cv::Point2f> > &corners, cv::Size lowResImageSize);
  void cornerUpsample(std::vector<Marker> &corners, cv::Size lowResImageSize);

  /**
   * Given the iput image with markers, creates an output image with it in the canonical position
   * @param in input image
   * @param out image with the marker
   * @param size of out
   * @param points 4 corners of the marker in the image in
   * @return true if the operation succeed
   */
  bool warp(cv::Mat &in, cv::Mat &out, cv::Size size, std::vector<cv::Point2f> points);


  // serialization in binary mode
  void toStream(std::ostream &str) const;
  void fromStream(std::istream &str);
  // configure the detector from a set of parameters
  void setParameters(const Params &params);



private:
  MarkerDetector_Impl *_impl;
};
};  // namespace aruco
#endif
