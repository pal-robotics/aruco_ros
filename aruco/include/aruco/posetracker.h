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

#ifndef ARUCO_POSETRACKER
#define ARUCO_POSETRACKER

#include "aruco_export.h"
#include "cameraparameters.h"
#include "marker.h"
#include "markermap.h"

#include <map>
#include <opencv2/core/core.hpp>

namespace aruco
{
/**Tracks the position of a marker. Instead of trying to calculate the position from
 * scratch everytime, it uses past
 * observations to
 * estimate the pose. It should solve the problem with ambiguities that arises in some
 * circumstances
 *
 * To solve ambiguity we follow the following idea. We are using the IPPE method, which
 * returns the two possible
 * solutions s0,s1.
 *  Error solution has a reprojection error e(s_i) and it is assumed that e(s0)<e(s1). If
 * the reprojection errors
 * are similar, then, it is difficult from only one image to know the correct location.
 *  So, if  the error ratio er=e(s_1)/e(s_0) ~ 1, there is a possible ambiguity problem.
 * Please notice that er is in
 * range [1,inf)
 *
 * To solve the problem, we should do a tracking process, so that when the ambiguity
 * occur, you can select the
 * solution nearest to the previos position.
 *
 * So, the only problem now becomes the initialization. You have two options. First, you
 * wait for a position in
 * which ambiguity does not happen. When a robuts position is seen,
 * you start tracking. Scond, you risk and start tracking and if later you discover the
 * error, you correct.
 *
 * This idea is implemented in the tracker.
 *
 * Call estimatePose indicated as last parameter the error ratio you want for the
 * initialization. If the parameter
 * is set er=1, then you start right away.
 * Be warned then that you might suffer a big shift later if there was an erroneous
 * starting location
 *
 * If you do not want to risk, I recommen using more conservative approach, use a value of
 * er=4.
 */
class ARUCO_EXPORT MarkerPoseTracker
{
public:
  /**     estimate the pose of the marker.
   * @brief estimatePose
   * @param m marker info
   * @param cam_params camera parameters
   * @param markerSize
   * @param minErrorRatio see explanation above. If you want to be conservative, use
   * minErrorRatio=4. tau_e in paper
   * @return true if the pose is estimated and false otherwise. If not estimated, the
   * parameters m.Rvec and m.Tvec
   * and not set.
   */
  bool estimatePose(Marker& m, const CameraParameters& cam_params, float markerSize,
                    float minErrorRatio = 10 /*tau_e in paper*/);

  // returns the 4x4 transform matrix. Returns an empty matrix if last call to
  // estimatePose returned false
  cv::Mat getRTMatrix() const;
  // return the rotation vector. Returns an empty matrix if last call to estimatePose
  // returned false
  const cv::Mat getRvec() const
  {
    return _rvec;
  }
  // return the translation vector. Returns an empty matrix if last call to estimatePose
  // returned false
  const cv::Mat getTvec() const
  {
    return _tvec;
  }

private:
  cv::Mat _rvec, _tvec;  // current poses
  double solve_pnp(const std::vector<cv::Point3f>& p3d,
                   const std::vector<cv::Point2f>& p2d, const cv::Mat& cam_matrix,
                   const cv::Mat& dist, cv::Mat& r_io, cv::Mat& t_io);
};
/**Tracks the position of a markermap
 */

class ARUCO_EXPORT MarkerMapPoseTracker
{
public:
  MarkerMapPoseTracker();
  // Sets the parameters required for operation
  // If the msconf has data expressed in meters, then the markerSize parameter is not
  // required. If it is in
  // pixels, the markersize will be used to
  // transform to meters
  // Throws exception if wrong configuraiton
  void setParams(const CameraParameters& cam_params, const MarkerMap& msconf,
                 float markerSize = -1);
  // indicates if the call to setParams has been successfull and this object is ready to
  // call estimatePose
  bool isValid() const
  {
    return _isValid;
  }

  // resets current state
  void reset()
  {
    _isValid = false;
    _rvec = cv::Mat();
    _tvec = cv::Mat();
  }

  // estimates camera pose wrt the markermap
  // returns true if pose has been obtained and false otherwise
  bool estimatePose(const std::vector<Marker>& v_m);

  // returns the 4x4 transform matrix. Returns an empty matrix if last call to
  // estimatePose returned false
  cv::Mat getRTMatrix() const;
  // return the rotation vector. Returns an empty matrix if last call to estimatePose
  // returned false
  const cv::Mat getRvec() const
  {
    return _rvec;
  }
  // return the translation vector. Returns an empty matrix if last call to estimatePose
  // returned false
  const cv::Mat getTvec() const
  {
    return _tvec;
  }
  // prevents from big jumps. If the difference between current and previous positions are
  // greater than the value indicated
  // assumes no good tracking and the pose will be set as null
  void setMaxTrackingDifference(float maxTranslation, float maxAngle)
  {
    _maxTranslation = maxTranslation;
    _maxAngle = maxAngle;
  }

  void setMinErrorRatio(float minErrorRatio)
  {
    aruco_minerrratio_valid = minErrorRatio;
  }

  double getInitialErr()
  {
    return _initial_err;
  }

protected:
  cv::Mat _rvec, _tvec;  // current poses
  aruco::CameraParameters _cam_params;
  MarkerMap _msconf;
  std::map<int, Marker3DInfo> _map_mm;
  bool _isValid;
  cv::Mat relocalization(const std::vector<Marker>& v_m);
  float aruco_minerrratio_valid;           /*tau_e in paper*/
  std::map<uint32_t, cv::Mat> marker_m2g;  // for each marker, the transform from the
                                           // global ref system to the marker ref system
  float _maxTranslation = -1, _maxAngle = -1;
  double _initial_err;
};
};  // namespace aruco

#endif
