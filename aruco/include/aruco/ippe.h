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


// This is the core header file for IPPE. Infinitesimal Plane-based Pose Estimation (IPPE)
// is a very fast and accurate way to compute a camera's pose from a single image of a
// planar object using point correspondences. This has uses in several applications,
// including augmented reality, 3D tracking and pose estimation with planar markers, and
// 3D scene understanding. This package is free and covered by the BSD licence without any
// warranty. We hope you find this code useful and if so please cite our paper in your
// work:

//@article{ year={2014}, issn={0920-5691}, journal={International Journal of Computer
//Vision}, volume={109}, number={3}, doi={10.1007/s11263-014-0725-5}, title={Infinitesimal
// Plane-Based Pose Estimation}, url={http://dx.doi.org/10.1007/s11263-014-0725-5},
// publisher={Springer US}, keywords={Plane; Pose; SfM; PnP; Homography}, author={Collins,
// Toby and Bartoli, Adrien}, pages={252-286}, language={English} }

// Please contact Toby (toby.collins@gmail.com) if you have any questions about the code, paper and IPPE.


#ifndef _IPPE_H_
#define _IPPE_H_

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "aruco_export.h"
#include <limits>

#define IPPE_SMALL 1e-7  // a small constant used to test 'small' values close to zero.

namespace aruco
{
// returns the two solutions
std::vector<cv::Mat> solvePnP(const std::vector<cv::Point3f>& objPoints,
                              const std::vector<cv::Point2f>& imgPoints,
                              cv::InputArray cameraMatrix, cv::InputArray distCoeffs);
void solvePnP(const std::vector<cv::Point3f>& objPoints,
              const std::vector<cv::Point2f>& imgPoints, cv::InputArray cameraMatrix,
              cv::InputArray distCoeffs, cv::Mat& rvec, cv::Mat& tvec);
ARUCO_EXPORT std::vector<std::pair<cv::Mat, double>> solvePnP_(
    float size, const std::vector<cv::Point2f>& imgPoints, cv::InputArray cameraMatrix,
    cv::InputArray distCoeffs);

std::vector<std::pair<cv::Mat, double>> solvePnP_(const std::vector<cv::Point3f>& objPoints,
                                                  const std::vector<cv::Point2f>& imgPoints,
                                                  cv::InputArray cameraMatrix,
                                                  cv::InputArray distCoeffs);

}  // namespace aruco
namespace IPPE
{

class PoseSolver
{
public:
  /**
   * @brief PoseSolver constructor
   */
  PoseSolver();

  /**
   * @brief PoseSolver destructor
   */
  ~PoseSolver();

  /**
   * @brief                Finds the two possible poses of a planar object given a set of
   * correspondences and their respective reprojection errors. The poses are sorted with
   * the first having the lowest reprojection error.
   * @param _objectPoints  Array of 4 or more coplanar object points defined in object coordinates.
   * 1xN/Nx1 3-channel (float or double) where N is the number of points
   * @param _imagePoints   Array of corresponding image points, 1xN/Nx1 2-channel. This can
   * either be in pixel coordinates or normalized pixel coordinates.
   * @param _cameraMatrix  Intrinsic camera matrix (same definition as OpenCV). If _imagePoints
   * is in normalized pixel coordinates you must set  _cameraMatrix = cv::noArray()
   * @param _distCoeffs    Intrinsic camera distortion vector (same definition as OpenCV).
   * If _imagePoints is in normalized pixel coordinates you must set  _cameraMatrix = cv::noArray()
   * @param _rvec1         First rotation solution (3x1 rotation vector)
   * @param _tvec1         First translation solution (3x1 vector)
   * @param reprojErr1     Reprojection error of first solution
   * @param _rvec2         Second rotation solution (3x1 rotation vector)
   * @param _tvec2         Second translation solution (3x1 vector)
   * @param reprojErr2     Reprojection error of second solution
   */
  void solveGeneric(cv::InputArray _objectPoints, cv::InputArray _imagePoints,
                    cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
                    cv::OutputArray _rvec1, cv::OutputArray _tvec1, float& reprojErr1,
                    cv::OutputArray _rvec2, cv::OutputArray _tvec2, float& reprojErr2);

  /** @brief                Finds the two possible poses of a square planar object and their
   * respective reprojection errors using IPPE. These poses are sorted so that the first
   * one is the one with the lowest reprojection error.
   *
   * @param _squareLength      The square's length (which is also it's width) in object coordinate
   * units (e.g. millimeters, meters, etc.)
   * @param _imagePoints       Array of corresponding image points, 1xN/Nx1 2-channel. This
   * can either be in pixel coordinates or normalized pixel coordinates.
   * @param _cameraMatrix      Intrinsic camera matrix (same definition as OpenCV). If _imagePoints
   * is in normalized pixel coordinates you must set  _cameraMatrix = cv::noArray()
   * @param _distCoeffs        Intrinsic camera distortion vector (same definition as OpenCV).
   * If _imagePoints is in normalized pixel coordinates you must set  _cameraMatrix = cv::noArray()
   * @param _rvec1             First rotation solution (3x1 rotation vector)
   * @param _tvec1             First translation solution (3x1 vector)
   * @param reprojErr1         Reprojection error of first solution
   * @param _rvec2             Second rotation solution (3x1 rotation vector)
   * @param _tvec2             Second translation solution (3x1 vector)
   * @param reprojErr2         Reprojection error of second solution
   */
  void solveSquare(double squareLength, cv::InputArray _imagePoints,
                   cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
                   cv::OutputArray _rvec1, cv::OutputArray _tvec1, float& reprojErr1,
                   cv::OutputArray _rvec2, cv::OutputArray _tvec2, float& reprojErr2);

  /**
   * @brief                   Generates the 4 object points of a square planar object
   * @param squareLength      The square's length (which is also it's width) in object
   * coordinate units (e.g. millimeters, meters, etc.)
   * @param _objectPoints        Set of 4 object points (1x4 3-channel double)
   */
  void generateSquareObjectCorners3D(double squareLength, cv::OutputArray _objectPoints);

  /**
   * @brief                   Generates the 4 object points of a square planar object,
   * without including the z-component (which is z=0 for all points).
   * @param squareLength      The square's length (which is also it's width) in object
   * coordinate units (e.g. millimeters, meters, etc.)
   * @param _objectPoints        Set of 4 object points (1x4 2-channel double)
   */
  void generateSquareObjectCorners2D(double squareLength, cv::OutputArray _objectPoints);

  /**
   * @brief                   Computes the average depth of an object given its pose in
   * camera coordinates
   * @param objectPoints:        Object points defined in 3D object space
   * @param rvec:             Rotation component of pose
   * @param tvec:             Translation component of pose
   * @return:                 average depth of the object
   */
  double meanSceneDepth(cv::InputArray objectPoints, cv::InputArray rvec, cv::InputArray tvec);

private:
  /**
   * @brief                       Finds the two possible poses of a planar object given a
   * set of correspondences in normalized pixel coordinates. These poses are **NOT**
   * sorted on reprojection error. Note that the returned poses are object-to-camera
   * transforms, and not camera-to-object transforms.
   * @param _objectPoints         Array of 4 or more coplanar object points defined in
   * object coordinates. 1xN/Nx1 3-channel (float or double).
   * @param _normalizedImagePoints   Array of corresponding image points in normalized
   * pixel coordinates, 1xN/Nx1 2-channel (float or double).
   * @param _Ma                   First pose solution (unsorted)
   * @param _Mb                   Second pose solution (unsorted)
   */
  void solveGeneric(cv::InputArray _objectPoints, cv::InputArray _normalizedImagePoints,
                    cv::OutputArray _Ma, cv::OutputArray _Mb);

  /**
   * @brief                      Finds the two possible poses of a planar object in its canonical
   * position, given a set of correspondences in normalized pixel coordinates. These poses
   * are **NOT** sorted on reprojection error. Note that the returned poses are object-to-camera
   * transforms, and not camera-to-object transforms.
   * @param _canonicalObjPoints      Array of 4 or more coplanar object points defined in
   * object coordinates. 1xN/Nx1 3-channel (double) where N is the number of points
   * @param _normalizedInputPoints   Array of corresponding image points in normalized pixel
   * coordinates, 1xN/Nx1 2-channel (double) where N is the number of points
   * @param _H                   Homography mapping canonicalObjPoints to normalizedInputPoints.
   * @param _Ma
   * @param _Mb
   */
  void solveCanonicalForm(cv::InputArray _canonicalObjPoints, cv::InputArray _normalizedInputPoints,
                          cv::InputArray _H, cv::OutputArray _Ma, cv::OutputArray _Mb);

  /** @brief                        Computes the translation solution for a given rotation solution
   * @param _objectPoints              Array of corresponding object points, 1xN/Nx1 3-channel
   * where N is the number of points
   * @param _normalizedImagePoints     Array of corresponding image points (undistorted),
   * 1xN/Nx1 2-channel where N is the number of points
   * @param _R                         Rotation solution (3x1 rotation vector)
   * @param _t  Translation solution   Translation solution (3x1 rotation vector)
   */
  void computeTranslation(cv::InputArray _objectPoints, cv::InputArray _normalizedImgPoints,
                          cv::InputArray _R, cv::OutputArray _t);

  /** @brief                        Computes the two rotation solutions from the Jacobian
   * of a homography matrix H at a point (ux,uy) on the object plane. For highest accuracy
   * the Jacobian should be computed at the centroid of the point correspondences (see the
   * IPPE paper for the explanation of this). For a point (ux,uy) on the object plane, suppose
   * the homography H maps (ux,uy) to a point (p,q) in the image (in normalized pixel coordinates).
   * The Jacobian matrix [J00, J01; J10,J11] is the Jacobian of the mapping evaluated at (ux,uy).
   * @param j00                        Homography jacobian coefficent at (ux,uy)
   * @param j01                        Homography jacobian coefficent at (ux,uy)
   * @param j10                        Homography jacobian coefficent at (ux,uy)
   * @param j11                        Homography jacobian coefficent at (ux,uy)
   * @param p                          the x coordinate of point (ux,uy) mapped into the
   * image (undistorted and normalized position)
   * @param q                          the y coordinate of point (ux,uy) mapped into the
   * image (undistorted and normalized position)
   */
  void computeRotations(double j00, double j01, double j10, double j11, double p,
                        double q, cv::OutputArray _R1, cv::OutputArray _R2);

  /** @brief                        Closed-form solution for the homography mapping with
   * four corner correspondences of a square (it maps source points to target points). The
   * source points are the four corners of a zero-centred squared defined by: point 0: [-squareLength
   * / 2.0, squareLength / 2.0] point 1: [squareLength / 2.0, squareLength / 2.0] point 2:
   * [squareLength / 2.0, -squareLength / 2.0] point 3: [-squareLength / 2.0, -squareLength / 2.0]
   *
   * @param _targetPoints              Array of four corresponding target points, 1x4/4x1
   * 2-channel. Note that the points should be ordered to correspond with points 0, 1, 2 and 3.
   * @param halfLength                 the square's half length (i.e. squareLength/2.0)
   * @param _H                         Homograhy mapping the source points to the target points, 3x3 single channel
   */
  void homographyFromSquarePoints(cv::InputArray _targetPoints, double halfLength,
                                  cv::OutputArray _H);

  /** @brief              Fast conversion from a rotation matrix to a rotation vector
   * using Rodrigues' formula
   * @param _R               Input rotation matrix, 3x3 1-channel (double)
   * @param _r               Output rotation vector, 3x1/1x3 1-channel (double)
   */
  void rot2vec(cv::InputArray _R, cv::OutputArray _r);

  /**
   * @brief                          Takes a set of planar object points and transforms them
   * to 'canonical' object coordinates This is when they have zero mean and are on the plane z=0
   * @param _objectPoints            Array of 4 or more coplanar object points defined in
   * object coordinates. 1xN/Nx1 3-channel (float or double) where N is the number of points
   * @param _canonicalObjectPoints   Object points in canonical coordinates 1xN/Nx1 2-channel (double)
   * @param _MobjectPoints2Canonical Transform matrix mapping _objectPoints to _canonicalObjectPoints:
   * 4x4 1-channel (double)
   */
  void makeCanonicalObjectPoints(cv::InputArray _objectPoints,
                                 cv::OutputArray _canonicalObjectPoints,
                                 cv::OutputArray _MobjectPoints2Canonical);

  /**
   * @brief                           Evaluates the Root Mean Squared (RMS) reprojection
   * error of a pose solution.
   * @param _objectPoints             Array of 4 or more coplanar object points defined in
   * object coordinates. 1xN/Nx1 3-channel (float or double) where N is the number of
   * points
   * @param _imagePoints              Array of corresponding image points, 1xN/Nx1
   * 2-channel. This can either be in pixel coordinates or normalized pixel coordinates.
   * @param _cameraMatrix             Intrinsic camera matrix (same definition as OpenCV).
   * If _imagePoints is in normalized pixel coordinates you must set  _cameraMatrix =
   * cv::noArray().
   * @param _distCoeffs               Intrinsic camera distortion vector (same definition
   * as OpenCV). If _imagePoints is in normalized pixel coordinates you must set
   * _cameraMatrix = cv::noArray().
   * @param _M                        Pose matrix from 3D object to camera coordinates:
   * 4x4 1-channel (double)
   * @param err                       RMS reprojection error
   */
  void evalReprojError(cv::InputArray _objectPoints, cv::InputArray _imagePoints,
                       cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
                       cv::InputArray _M, float& err);

  /**
   * @brief                           Sorts two pose solutions according to their RMS
   * reprojection error (lowest first).
   * @param _objectPoints             Array of 4 or more coplanar object points defined in
   * object coordinates. 1xN/Nx1 3-channel (float or double) where N is the number of
   * points
   * @param _imagePoints              Array of corresponding image points, 1xN/Nx1
   * 2-channel.  This can either be in pixel coordinates or normalized pixel coordinates.
   * @param _cameraMatrix             Intrinsic camera matrix (same definition as OpenCV).
   * If _imagePoints is in normalized pixel coordinates you must set  _cameraMatrix =
   * cv::noArray().
   * @param _distCoeffs               Intrinsic camera distortion vector (same definition
   * as OpenCV). If _imagePoints is in normalized pixel coordinates you must set
   * _cameraMatrix = cv::noArray().
   * @param _Ma                       Pose matrix 1: 4x4 1-channel
   * @param _Mb                       Pose matrix 2: 4x4 1-channel
   * @param _M1                       Member of (Ma,Mb} with lowest RMS reprojection
   * error. Performs deep copy.
   * @param _M2                       Member of (Ma,Mb} with highest RMS reprojection
   * error. Performs deep copy.
   * @param err1                      RMS reprojection error of _M1
   * @param err2                      RMS reprojection error of _M2
   */
  void sortPosesByReprojError(cv::InputArray _objectPoints, cv::InputArray _imagePoints,
                              cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs,
                              cv::InputArray _Ma, cv::InputArray _Mb, cv::OutputArray _M1,
                              cv::OutputArray _M2, float& err1, float& err2);

  /**
   * @brief                           Finds the rotation _Ra that rotates a vector _a to
   * the z axis (0,0,1)
   * @param _a                        vector: 3x1 mat (double)
   * @param _Ra                       Rotation: 3x3 mat (double)
   */
  void rotateVec2ZAxis(cv::InputArray _a, cv::OutputArray _Ra);


  /**
   * @brief                           Computes the rotation _R that rotates the object points
   * to the plane z=0. This uses the cross-product method with the first three object points.
   * @param _objectPoints             Array of N>=3 coplanar object points defined in object
   * coordinates. 1xN/Nx1 3-channel (float or double) where N is the number of points
   * @param _R                        Rotation Mat: 3x3 (double)
   * @return                          success (true) or failure (false)
   */
  bool computeObjextSpaceR3Pts(cv::InputArray _objectPoints, cv::OutputArray _R);

  /**
   * @brief computeObjextSpaceRSvD    Computes the rotation _R that rotates the object points
   * to the plane z=0. This uses the cross-product method with the first three object points.
   * @param _objectPointsZeroMean     zero-meaned co=planar object points: 3xN matrix (double) where N>=3
   * @param _R                        Rotation Mat: 3x3 (double)
   * @return                          success (true) or failure (false)
   */
  bool computeObjextSpaceRSvD(cv::InputArray _objectPointsZeroMean, cv::OutputArray _R);
};
}  // namespace IPPE

namespace HomographyHO
{

/**
 * @brief                   Computes the best-fitting homography matrix from source to
 * target points using Harker and O'Leary's method: Harker, M., O'Leary, P., Computation
 * of Homographies, Proceedings of the British Machine Vision Conference 2005, Oxford,
 * England. This is not the author's implementation.
 * @param srcPoints         Array of source points: 1xN/Nx1 2-channel (float or double)
 * where N is the number of points
 * @param targPoints        Array of target points: 1xN/Nx1 2-channel (float or double)
 * @param H                 Homography from source to target: 3x3 1-channel (double)
 */
void homographyHO(cv::InputArray srcPoints, cv::InputArray targPoints, cv::OutputArray H);

/**
 * @brief                      Performs data normalization before homography estimation.
 * For details see Hartley, R., Zisserman, A., Multiple View Geometry in Computer Vision,
 *                             Cambridge University Press, Cambridge, 2001
 * @param Data                 Array of source data points: 1xN/Nx1 2-channel (float or double)
 * where N is the number of points
 * @param DataN                Normalized data points: 1xN/Nx1 2-channel (float or double)
 * where N is the number of points
 * @param T                    Homogeneous transform from source to normalized: 3x3 1-channel (double)
 * @param Ti                   Homogeneous transform from normalized to source: 3x3 1-channel (double)
 */
void normalizeDataIsotropic(cv::InputArray Data, cv::OutputArray DataN, cv::OutputArray T,
                            cv::OutputArray Ti);
}  // namespace HomographyHO

#endif
