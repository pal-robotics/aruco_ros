#include <opencv2/imgproc/imgproc.hpp>
#include "../picoflann.h"
#include "../cameraparameters.h"
#include "../markerdetector.h"
#include "fractalmarkerset.h"
#include "../aruco_export.h"
namespace aruco
{


void kfilter(std::vector<cv::KeyPoint> &kpoints);

/** classification of the corners
 * @brief assignClass
 * @param im image used in the classification
 * @param kpoints to classify
 * @param sizeNorm. Is it necessary to transform keypoints? When keypoints have as
 * reference the center of image(0,0), it is necessary to provide the fractal marker size
 * @param wsize. Window size
 */
void assignClass(const cv::Mat &im, std::vector<cv::KeyPoint> &kpoints,
                 float sizeNorm = 0.f, int wsize = 5);

struct PicoFlann_KeyPointAdapter
{
  inline float operator()(const cv::KeyPoint &elem, int dim) const
  {
    return dim == 0 ? elem.pt.x : elem.pt.y;
  }
  inline float operator()(const cv::Point2f &elem, int dim) const
  {
    return dim == 0 ? elem.x : elem.y;
  }
};

class ARUCO_EXPORT FractalPoseTracker
{
public:
  FractalPoseTracker();
  /**     init fractlPoseTracker parameters
   * @brief setParams
   * @param cam_params camera paremeters
   * @param msconf FractalMarkerSet configuration
   * @param markerSize FractalMarker size
   */
  void setParams(const CameraParameters &cam_params, const FractalMarkerSet &msconf,
                 const float markerSize = -1);
  /**     estimate the pose of the fractal marker.
   * @brief fractalInnerPose
   * @param markerDetector
   * @param detected markers
   * @param refinement, use or not pose refinement. True by default.
   * @return true if the pose is estimated and false otherwise. If not estimated, the
   * parameters m.Rvec and m.Tvec and not set.
   */
  bool fractalInnerPose(const cv::Ptr<MarkerDetector> markerDetector,
                        const std::vector<aruco::Marker> &markers, bool refinement = true);
  /**     extraction of the region of the image where the marker is estimated to be based on the previous pose
   * @brief ROI
   * @param imagePyramid set images
   * @param img original image. The image is scaled according to the selected pyramid image.
   * @param innerPoints2d collection fractal inner points. The points are scaled according
   * to the selected pyramid image.
   * @param offset. Position of the upper inner corner of the marker. The offset is scaled
   * according to the selected pyramid image.
   * @param ratio selected scaling factor
   */
  bool ROI(const std::vector<cv::Mat> imagePyramid, cv::Mat &img,
           std::vector<cv::Point2f> &innerPoints2d, cv::Point2f &offset, float &ratio);

  /**     estimate the pose of the fractal marker. Method case 2, paper.
   * @brief fractal_solve_ransac
   * @param ninners (number of total inners points used)
   * @param inner_kpnt matches (inner points - detected keypoints)
   * @param kpnts keypoints
   * @param maxIter maximum number of iterations
   * @param _minInliers beta in paper
   * @param _thresInliers alpha in paper
   * @return Mat best model homography.
   */
  cv::Mat fractal_solve_ransac(int ninners,
                               std::vector<std::pair<uint, std::vector<uint>>> inner_kpnt,
                               std::vector<cv::KeyPoint> kpnts, uint32_t maxIter = 500,
                               float _minInliers = 0.2f, float _thresInliers = 0.7f);

  /**     Draw keypoints
   * @brief image
   * @param kpoints
   * @param text
   * @param transf. Is it necessary to transform keypoints? (-MarkerSize/2 ..
   * MarkerSize/2) to (0 .. ImageSize)
   */
  void drawKeyPoints(const cv::Mat image, std::vector<cv::KeyPoint> kpoints,
                     bool text = false, bool transf = false);

  /**     Refinement of the internal points of the marker and pose estimation with these.
   * @brief fractalRefinement
   * @param markerDetector
   * @param markerWarpPix. Optimal markerwarpPix used to select the image of the pyramid.
   * Default value 10.
   * @return true if the pose is estimated and false otherwise. If not estimated, the
   * parameters m.Rvec and m.Tvec and not set.
   */
  bool fractalRefinement(const cv::Ptr<MarkerDetector> markerDetector, int markerWarpPix = 10);

  // return the rotation vector. Returns an empty matrix if last call to estimatePose returned false
  const cv::Mat getRvec() const
  {
    return _rvec;
  }

  // return the translation vector. Returns an empty matrix if last call to estimatePose returned false
  const cv::Mat getTvec() const
  {
    return _tvec;
  }

  // return all corners from fractal marker
  const std::vector<cv::Point3f> getInner3d()
  {
    return _innerp3d;
  }

  // is the pose valid?
  bool isPoseValid() const
  {
    return !_rvec.empty() && !_tvec.empty();
  }

  FractalMarkerSet getFractal()
  {
    return _fractalMarker;
  }

private:
  FractalMarkerSet _fractalMarker;                       // FractalMarkerSet configuration
  aruco::CameraParameters _cam_params;                   // Camera parameters.
  cv::Mat _rvec, _tvec;                                  // current poses
  std::map<int, std::vector<cv::Point3f>> _id_innerp3d;  // Id_marker-Inners_corners
  std::vector<cv::Point3f> _innerp3d;                    // All inners corners
  std::vector<cv::KeyPoint> _innerkpoints;               // All inners keypoints
  picoflann::KdTreeIndex<2, PicoFlann_KeyPointAdapter> _kdtree;
  std::map<int, double> _id_radius;  // Idmarker_Radius(Optimus)
  std::map<int, float> _id_area;     // Idmarker_projectedArea(Optimus)
  float _preRadius = 0;              // radius used previous iteration
};
}  // namespace aruco
