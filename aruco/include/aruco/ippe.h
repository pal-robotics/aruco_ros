// This is the core header file for IPPE. Infinitesimal Plane-based Pose Estimation (IPPE) is a very fast and accurate
// way to compute a camera's pose from a single image of a planar object using point correspondences. This has uses in
// several applications, including augmented reality, 3D tracking and pose estimation with planar markers, and 3D scene
// understanding.
// This package is free and covered by the BSD licence without any warranty. We hope you find this code useful and if so
// please cite our paper in your work:

//@article{ year={2014}, issn={0920-5691}, journal={International Journal of Computer Vision}, volume={109}, number={3},
//doi={10.1007/s11263-014-0725-5}, title={Infinitesimal Plane-Based Pose Estimation},
//url={http://dx.doi.org/10.1007/s11263-014-0725-5}, publisher={Springer US}, keywords={Plane; Pose; SfM; PnP;
//Homography}, author={Collins, Toby and Bartoli, Adrien}, pages={252-286}, language={English} }

// Please contact Toby (toby.collins@gmail.com) if you have any questions about the code, paper and IPPE.

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

#ifndef _IPPE_H_
#define _IPPE_H_

#include <opencv2/core.hpp>
#include "aruco_export.h"

namespace aruco
{

// returns the two solutions
std::vector<cv::Mat> solvePnP(const std::vector<cv::Point3f>& objPoints, const std::vector<cv::Point2f>& imgPoints,
                              cv::InputArray cameraMatrix, cv::InputArray distCoeffs);
ARUCO_EXPORT std::vector<std::pair<cv::Mat, double> > solvePnP_(float size, const std::vector<cv::Point2f> &imgPoints,
                                                                cv::InputArray cameraMatrix, cv::InputArray distCoeffs);

std::vector<std::pair<cv::Mat, double>> solvePnP_(const std::vector<cv::Point3f>& objPoints,
                                                  const std::vector<cv::Point2f>& imgPoints,
                                                  cv::InputArray cameraMatrix, cv::InputArray distCoeffs);

/**
 * @brief Finds the two possible poses of a square planar object given its four corner correspondences in an image
 * using IPPE. These poses are sorted so that the first one is the one with the lowest reprojection error. The second
 * pose is needed if the problem is ambiguous. The problem is ambiguous when the projection of the model is close to
 * affine, which in practice happens if it is small or viewed from a large distance. In these cases there are two pose
 * solutions that can correctly align the correspondences (up to noise), so it is impossible to select the right one
 * from just the reprojection error. IPPE gives you both the solutions, rather than just a single solution (which in
 * ambiguous cases would be wrong 50% of the time). Geometrically, the two poses roughly correspond to a reflection of
 * the model about a plane whose normal passes through the line-of-sight from the camera center to the model's center.
 * For more details about these ambiguities, please refer to the IPPE paper.
 *
 * It is possible to reject the second pose if its reprojection error is significantly worse than the first pose.
 * The correct way to do this is with a likelihood ratio test (https://en.wikipedia.org/wiki/Likelihood-ratio_test).
 *
 * @param squareLength the squares's length (which is also it's width) in object coordinate units (e.g. millimeters,
 * meters, etc.)
 * The square is defined in object coordinates on the plane z=0 and centered at the origin. Therefore its four points in
 * object coordinates are given by:
 * point 0: [-squareLength / 2.0, squareLength / 2.0, 0]
 * point 1: [squareLength / 2.0, squareLength / 2.0, 0]
 * point 2: [squareLength / 2.0, -squareLength / 2.0, 0]
 * point 3: [-squareLength / 2.0, -squareLength / 2.0, 0]
 * @param imagePoints Array of four corresponding image points, 1x4/4x1 2-channel. Note that the points should be
 * ordered to correspond with points 0, 1, 2 and 3.
 * @param cameraMatrix Input camera matrix \f$A = \vecthreethree{fx}{0}{cx}{0}{fy}{cy}{0}{0}{1}\f$ .
 * @param distCoeffs Input vector of distortion coefficients
 * \f$(k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6 [, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\f$ of
 * 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
 * @param _rvec1 Output rotation vector (see Rodrigues ) for first pose. That, together with tvec , brings points from
 * the model coordinate system to the camera coordinate system.
 * @param _tvec1 Output translation vector for first pose.
 * @param reprojErr1 Output reprojection error of first pose
 * @param _rvec2 Output rotation vector (see Rodrigues ) for second pose. That, together with tvec , brings points from
 * the model coordinate system to the camera coordinate system.
 * @param _tvec2 Output translation vector for second pose.
 * @param reprojErr1 Output reprojection error of second pose
 */
void solvePoseOfCentredSquare(float squareLength, cv::InputArray imagePoints, cv::InputArray cameraMatrix,
                              cv::InputArray distCoeffs, cv::OutputArray _rvec1, cv::OutputArray _tvec1,
                              float& reprojErr1, cv::OutputArray _rvec2, cv::OutputArray _tvec2, float& reprojErr2);

/**
 * @brief Determines which of the two pose solutions from IPPE has the lowest reprojection error.
 * @param _R1 First rotation solution from IPPE, 3x3 1-channel float
 * @param _R2 Second rotation solution from IPPE  3x3 1-channel float
 * @param _t1 First translation solution from IPPE  3x1 1-channel float
 * @param _t2 Second translation solution from IPPE  3x3 1-channel float
 * @param _t2 Second translation solution from IPPE  3x1 1-channel float
 * @param _objectPoints Array of corresponding model points, 1xN/Nx1 3-channel where N is the number of points
 * @param _undistortedPoints Array of corresponding image points (undistorted and normalize), 1xN/Nx1 2-channel where N
 * is the number of points
 */
int IPPEvalBestPose(cv::InputArray _R1, cv::InputArray _R2, cv::InputArray _t1, cv::InputArray _t2,
                    cv::InputArray _objectPoints, cv::InputArray _undistortedPoints);

/**
 * @brief Determines the reprojection error of a pose solution
 * @param _R1 Rotation solution from IPPE, 3x3 1-channel float
 * @param _t  Translation solution from IPPE  3x1 1-channel float
 * @param _objectPoints Array of corresponding model points, 1xN/Nx1 3-channel where N is the number of points
 * @param _undistortedPoints Array of corresponding image points (undistorted and normalized), 1xN/Nx1 2-channel where
 * N is the number of points
 * @return The pose solution with the lowest reprojection error. This is either 1 (first pose) or 2 (second pose)
 */
float IPPEvalReprojectionError(cv::InputArray _R, cv::InputArray _t, cv::InputArray _objectPoints,
                               cv::InputArray _undistortedPoints);

/**
 * @brief Fast conversion from a rotation matrix to a rotation vector using Rodrigues' formula
 * @param _R Rotation matrix, 3x3 1-channel float
 * @param _r Rotation vector, 3x1/1x3 1-channel float
 */
void IPPERot2vec(cv::InputArray _R, cv::OutputArray _r);

/**
 * @brief Computes the translation solution for a given rotation solution
 * @param _objectPoints Array of corresponding model points, 1xN/Nx1 3-channel where N is the number of points
 * @param _undistortedPoints Array of corresponding image points (undistorted), 1xN/Nx1 2-channel where N is the number
 * of points
 * @param _R1 Rotation solution from IPPE, 3x3 1-channel float
 * @param _t  Translation solution, 3x1 1-channel float
 */
void IPPComputeTranslation(cv::InputArray _objectPoints, cv::InputArray _imgPoints, cv::InputArray _R,
                           cv::OutputArray _t);

/**
 * @brief Computes the two rotation solutions from the Jacobian of a homography matrix H. For highest accuracy the
 * Jacobian should be computed at the centroid of the point correspondences (see the IPPE paper for the explanation of
 * this). For a point (ux, uy) on the model plane, suppose the homography H maps (ux, uy) to a point (p,q) in the image
 * (in normalized pixel coordinates). The Jacobian matrix [J00, J01; J10,J11] is the Jacobian of the mapping evaluated
 * at (ux, uy).
 * @param j00 Jacobian coefficient
 * @param j01 Jacobian coefficient
 * @param j10 Jacobian coefficient
 * @param j11 Jacobian coefficient
 * @param p the x coordinate of point (ux, uy) mapped into the image (undistorted and normalized position)
 * @param q the y coordinate of point (ux, uy) mapped into the image (undistorted and normalized position)
 */
void IPPComputeRotations(double j00, double j01, double j10, double j11, double p, double q, cv::OutputArray _R1,
                         cv::OutputArray _R2);

/**
 * @brief Closed-form solution for the homography mapping with four corner correspondences of a square (it maps
 * source points to target points). The source points are the four corners of a zero-centered squared defined by:
 * point 0: [-squareLength / 2.0, squareLength / 2.0]
 * point 1: [squareLength / 2.0, squareLength / 2.0]
 * point 2: [squareLength / 2.0, -squareLength / 2.0]
 * point 3: [-squareLength / 2.0, -squareLength / 2.0]
 *
 * @param _targetPts Array of four corresponding target points, 1x4/4x1 2-channel. Note that the points should be
 * ordered to correspond with points 0, 1, 2 and 3.
 * @param halfLength the square's half length (i.e. squareLength/2.0)
 * @param _R1 Rotation solution from IPPE, 3x3 1-channel float
 * @param _H  Homography mapping the source points to the target points, 3x3 single channel
 */
void homographyFromSquarePoints(cv::InputArray _targetPts, double halfLength, cv::OutputArray _H);

} // namespace aruco

#endif /* _IPPE_H_ */
