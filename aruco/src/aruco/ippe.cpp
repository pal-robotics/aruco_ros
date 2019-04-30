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

#include "ippe.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace aruco
{

/**
 *
 */
cv::Mat getRTMatrix(const cv::Mat& R_, const cv::Mat& T_, int forceType)
{
  cv::Mat M;
  cv::Mat R, T;
  R_.copyTo(R);
  T_.copyTo(T);
  if (R.type() == CV_64F)
  {
    assert(T.type() == CV_64F);
    cv::Mat Matrix = cv::Mat::eye(4, 4, CV_64FC1);

    cv::Mat R33 = cv::Mat(Matrix, cv::Rect(0, 0, 3, 3));
    if (R.total() == 3)
    {
      cv::Rodrigues(R, R33);
    }
    else if (R.total() == 9)
    {
      cv::Mat R64;
      R.convertTo(R64, CV_64F);
      R.copyTo(R33);
    }
    for (int i = 0; i < 3; i++)
      Matrix.at<double>(i, 3) = T.ptr<double>(0)[i];
    M = Matrix;
  }
  else if (R.depth() == CV_32F)
  {
    cv::Mat Matrix = cv::Mat::eye(4, 4, CV_32FC1);
    cv::Mat R33 = cv::Mat(Matrix, cv::Rect(0, 0, 3, 3));
    if (R.total() == 3)
    {
      cv::Rodrigues(R, R33);
    }
    else if (R.total() == 9)
    {
      cv::Mat R32;
      R.convertTo(R32, CV_32F);
      R.copyTo(R33);
    }

    for (int i = 0; i < 3; i++)
      Matrix.at<float>(i, 3) = T.ptr<float>(0)[i];
    M = Matrix;
  }

  if (forceType == -1)
    return M;
  else
  {
    cv::Mat MTyped;
    M.convertTo(MTyped, forceType);
    return MTyped;
  }
}

std::vector<cv::Mat> solvePnP(const std::vector<cv::Point3f>& objPoints, const std::vector<cv::Point2f>& imgPoints,
                              cv::InputArray cameraMatrix, cv::InputArray distCoeffs)
{
  cv::Mat Rvec, Tvec;
  float markerLength = static_cast<float>(cv::norm(objPoints[1] - objPoints[0]));
  float reprojErr1, reprojErr2;
  cv::Mat Rvec2, Tvec2;

  solvePoseOfCentredSquare(markerLength, imgPoints, cameraMatrix, distCoeffs, Rvec, Tvec, reprojErr1, Rvec2, Tvec2,
                           reprojErr2);
  return
  { getRTMatrix(Rvec, Tvec, CV_32F), getRTMatrix(Rvec2, Tvec2, CV_32F)};
}

std::vector<std::pair<cv::Mat, double> > solvePnP_(float size, const std::vector<cv::Point2f> &imgPoints,
                                                   cv::InputArray cameraMatrix, cv::InputArray distCoeffs)
{
  cv::Mat Rvec, Tvec, Rvec2, Tvec2;
  float reprojErr1, reprojErr2;
  solvePoseOfCentredSquare(size, imgPoints, cameraMatrix, distCoeffs, Rvec, Tvec, reprojErr1, Rvec2, Tvec2, reprojErr2);
  return std::vector<std::pair<cv::Mat, double>> {std::make_pair(getRTMatrix(Rvec, Tvec, CV_32F), reprojErr1),
                                                  std::make_pair(getRTMatrix(Rvec2, Tvec2, CV_32F), reprojErr2)};
}

std::vector<std::pair<cv::Mat, double>> solvePnP_(const std::vector<cv::Point3f>& objPoints,
                                                  const std::vector<cv::Point2f>& imgPoints,
                                                  cv::InputArray cameraMatrix, cv::InputArray distCoeffs)
{
  cv::Mat Rvec, Tvec;
  float markerLength = static_cast<float>(cv::norm(objPoints[1] - objPoints[0]));
  float reprojErr1, reprojErr2;
  cv::Mat Rvec2, Tvec2;

  solvePoseOfCentredSquare(markerLength, imgPoints, cameraMatrix, distCoeffs, Rvec, Tvec, reprojErr1, Rvec2, Tvec2,
                           reprojErr2);
  return std::vector<std::pair<cv::Mat, double>> {std::make_pair(getRTMatrix(Rvec, Tvec, CV_32F), reprojErr1),
                                                  std::make_pair(getRTMatrix(Rvec2, Tvec2, CV_32F), reprojErr2)};
}

void solvePoseOfCentredSquare(float squareLength, cv::InputArray imagePoints, cv::InputArray cameraMatrix,
                              cv::InputArray distCoeffs, cv::OutputArray _rvec1, cv::OutputArray _tvec1,
                              float& reprojErr1, cv::OutputArray _rvec2, cv::OutputArray _tvec2, float& reprojErr2)
{
  cv::Mat undistortedPoints; // undistorted version of imagePoints
  cv::Mat modelPoints(4, 1, CV_32FC3);
  // set coordinate system in the middle of the marker, with Z pointing out
  modelPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-squareLength / 2.0f, squareLength / 2.0f, 0);
  modelPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(squareLength / 2.0f, squareLength / 2.0f, 0);
  modelPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(squareLength / 2.0f, -squareLength / 2.0f, 0);
  modelPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-squareLength / 2.0f, -squareLength / 2.0f, 0);

  //(Ra, ta), (Rb, tb) are the two pose solutions from IPPE.
  _rvec1.create(3, 1, CV_64FC1);
  _tvec1.create(3, 1, CV_64FC1);
  _rvec2.create(3, 1, CV_64FC1);
  _tvec2.create(3, 1, CV_64FC1);

  cv::Mat H, Ra, Rb, ta, tb;
  cv::Mat tvec1 = _tvec1.getMat();
  cv::Mat rvec1 = _rvec1.getMat();
  cv::Mat tvec2 = _tvec2.getMat();
  cv::Mat rvec2 = _rvec2.getMat();

  // undistort the image points (i.e. put them in normalized pixel coordinates).
  undistortPoints(imagePoints, undistortedPoints, cameraMatrix, distCoeffs);

  // compute the homography mapping the model's four corners to undistortedPoints
  homographyFromSquarePoints(undistortedPoints, squareLength / 2.0f, H);

  // compute the Jacobian J of the homography at (0, 0):
  double j00, j01, j10, j11, v0, v1;

  j00 = H.at<double>(0, 0) - H.at<double>(2, 0) * H.at<double>(0, 2);
  j01 = H.at<double>(0, 1) - H.at<double>(2, 1) * H.at<double>(0, 2);
  j10 = H.at<double>(1, 0) - H.at<double>(2, 0) * H.at<double>(1, 2);
  j11 = H.at<double>(1, 1) - H.at<double>(2, 1) * H.at<double>(1, 2);

  // compute the transformation of (0, 0) into the image:
  v0 = H.at<double>(0, 2);
  v1 = H.at<double>(1, 2);

  // compute the two rotation solutions:
  IPPComputeRotations(j00, j01, j10, j11, v0, v1, Ra, Rb);

  // for each rotation solution, compute the corresponding translation solution:
  IPPComputeTranslation(modelPoints, undistortedPoints, Ra, ta);
  IPPComputeTranslation(modelPoints, undistortedPoints, Rb, tb);

  float reprojErra = IPPEvalReprojectionError(Ra, ta, modelPoints, undistortedPoints);
  float reprojErrb = IPPEvalReprojectionError(Rb, tb, modelPoints, undistortedPoints);

  if (reprojErra < reprojErrb)
  {
    tvec1.at<double>(0) = ta.at<double>(0);
    tvec1.at<double>(1) = ta.at<double>(1);
    tvec1.at<double>(2) = ta.at<double>(2);
    IPPERot2vec(Ra, rvec1);

    tvec2.at<double>(0) = tb.at<double>(0);
    tvec2.at<double>(1) = tb.at<double>(1);
    tvec2.at<double>(2) = tb.at<double>(2);
    IPPERot2vec(Rb, rvec2);

    reprojErr1 = reprojErra;
    reprojErr2 = reprojErrb;
  }
  else
  {
    tvec1.at<double>(0) = tb.at<double>(0);
    tvec1.at<double>(1) = tb.at<double>(1);
    tvec1.at<double>(2) = tb.at<double>(2);
    IPPERot2vec(Rb, rvec1);

    tvec2.at<double>(0) = ta.at<double>(0);
    tvec2.at<double>(1) = ta.at<double>(1);
    tvec2.at<double>(2) = ta.at<double>(2);
    IPPERot2vec(Ra, rvec2);

    reprojErr1 = reprojErrb;
    reprojErr2 = reprojErra;
  }
}

int IPPEvalBestPose(cv::InputArray _R1, cv::InputArray _R2, cv::InputArray _t1, cv::InputArray _t2,
                    cv::InputArray _objectPoints, cv::InputArray _undistortedPoints)
{
  cv::Mat modelPoints = _objectPoints.getMat();
  cv::Mat imgPoints = _undistortedPoints.getMat();

  cv::Mat R1 = _R1.getMat();
  cv::Mat t1 = _t1.getMat();

  cv::Mat R2 = _R2.getMat();
  cv::Mat t2 = _t2.getMat();

  int numPts = modelPoints.rows;

  // now loop over each correspondence and compute the reprojection error of both pose solution
  float px, py, pz;
  float reprojError1 = 0; // reprojection error of pose 1
  float reprojError2 = 0; // reprojection error of pose 2

  float dx, dy; // residual reprojection error with respect to x and y coordinates
  for (int i = 0; i < numPts; i++)
  {
    // projection with first pose solution:
    px = static_cast<float>(R1.at<double>(0, 0) * modelPoints.at<cv::Vec3f>(i)(0))
        + static_cast<float>(R1.at<double>(0, 1) * modelPoints.at<cv::Vec3f>(i)(1))
        + static_cast<float>(R1.at<double>(0, 2) * modelPoints.at<cv::Vec3f>(i)(2) + t1.at<double>(0));
    py = static_cast<float>(R1.at<double>(1, 0) * modelPoints.at<cv::Vec3f>(i)(0))
        + static_cast<float>(R1.at<double>(1, 1) * modelPoints.at<cv::Vec3f>(i)(1))
        + static_cast<float>(R1.at<double>(1, 2) * modelPoints.at<cv::Vec3f>(i)(2) + t1.at<double>(1));
    pz = static_cast<float>(R1.at<double>(2, 0) * modelPoints.at<cv::Vec3f>(i)(0))
        + static_cast<float>(R1.at<double>(2, 1) * modelPoints.at<cv::Vec3f>(i)(1))
        + static_cast<float>(R1.at<double>(2, 2) * modelPoints.at<cv::Vec3f>(i)(2) + t1.at<double>(2));

    dx = px / pz - imgPoints.at<cv::Vec2f>(i)(0);
    dy = py / pz - imgPoints.at<cv::Vec2f>(i)(1);

    reprojError1 = reprojError1 + std::sqrt(dx * dx + dy * dy);

    // projection with second pose solution:
    px = static_cast<float>(R2.at<double>(0, 0) * modelPoints.at<cv::Vec3f>(i)(0))
        + static_cast<float>(R2.at<double>(0, 1) * modelPoints.at<cv::Vec3f>(i)(1))
        + static_cast<float>(R2.at<double>(0, 2) * modelPoints.at<cv::Vec3f>(i)(2) + t2.at<double>(0));
    py = static_cast<float>(R2.at<double>(1, 0) * modelPoints.at<cv::Vec3f>(i)(0))
        + static_cast<float>(R2.at<double>(1, 1) * modelPoints.at<cv::Vec3f>(i)(1))
        + static_cast<float>(R2.at<double>(1, 2) * modelPoints.at<cv::Vec3f>(i)(2) + t2.at<double>(1));
    pz = static_cast<float>(R2.at<double>(2, 0) * modelPoints.at<cv::Vec3f>(i)(0))
        + static_cast<float>(R2.at<double>(2, 1) * modelPoints.at<cv::Vec3f>(i)(1))
        + static_cast<float>(R2.at<double>(2, 2) * modelPoints.at<cv::Vec3f>(i)(2) + t2.at<double>(2));

    dx = px / pz - imgPoints.at<cv::Vec2f>(i)(0);
    dy = py / pz - imgPoints.at<cv::Vec2f>(i)(1);

    reprojError2 = reprojError2 + std::sqrt(dx * dx + dy * dy);
  }
  if (reprojError1 < reprojError2)
  {
    return 1;
  }
  else
  {
    return 2;
  }
}

float IPPEvalReprojectionError(cv::InputArray _R, cv::InputArray _t, cv::InputArray _objectPoints,
                               cv::InputArray _undistortedPoints)
{
  cv::Mat modelPoints = _objectPoints.getMat();
  cv::Mat imgPoints = _undistortedPoints.getMat();

  cv::Mat R = _R.getMat();
  cv::Mat t = _t.getMat();

  int numPts = modelPoints.rows;
  float px, py, pz;
  float reprojError = 0;
  float dx, dy; // residual reprojection error with respect to x and y coordinates

  // now loop over each correspondence and compute the reprojection error
  for (int i = 0; i < numPts; i++)
  {
    px = static_cast<float>(R.at<double>(0, 0) * modelPoints.at<cv::Vec3f>(i)(0))
        + static_cast<float>(R.at<double>(0, 1) * modelPoints.at<cv::Vec3f>(i)(1))
        + static_cast<float>(R.at<double>(0, 2) * modelPoints.at<cv::Vec3f>(i)(2) + t.at<double>(0));
    py = static_cast<float>(R.at<double>(1, 0) * modelPoints.at<cv::Vec3f>(i)(0))
        + static_cast<float>(R.at<double>(1, 1) * modelPoints.at<cv::Vec3f>(i)(1))
        + static_cast<float>(R.at<double>(1, 2) * modelPoints.at<cv::Vec3f>(i)(2) + t.at<double>(1));
    pz = static_cast<float>(R.at<double>(2, 0) * modelPoints.at<cv::Vec3f>(i)(0))
        + static_cast<float>(R.at<double>(2, 1) * modelPoints.at<cv::Vec3f>(i)(1))
        + static_cast<float>(R.at<double>(2, 2) * modelPoints.at<cv::Vec3f>(i)(2) + t.at<double>(2));

    dx = px / pz - imgPoints.at<cv::Vec2f>(i)(0);
    dy = py / pz - imgPoints.at<cv::Vec2f>(i)(1);

    reprojError = reprojError + std::sqrt(dx * dx + dy * dy);
  }
  return reprojError;
}

void IPPERot2vec(cv::InputArray _R, cv::OutputArray _r)
{
  cv::Mat R = _R.getMat();
  cv::Mat rvec = _r.getMat();
  double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
  double w_norm = acos((trace - 1.0) / 2.0);
  double c0, c1, c2;
  double eps = std::numeric_limits<double>::epsilon();
  double d = 1 / (2 * sin(w_norm)) * w_norm;
  if (w_norm < eps) // rotation is the identity
  {
    rvec.setTo(0);
  }
  else
  {
    c0 = R.at<double>(2, 1) - R.at<double>(1, 2);
    c1 = R.at<double>(0, 2) - R.at<double>(2, 0);
    c2 = R.at<double>(1, 0) - R.at<double>(0, 1);
    rvec.at<double>(0) = d * c0;
    rvec.at<double>(1) = d * c1;
    rvec.at<double>(2) = d * c2;
  }
}

void IPPComputeTranslation(cv::InputArray _objectPoints, cv::InputArray _imgPoints, cv::InputArray _R,
                           cv::OutputArray _t)
{
  // This is solved by building the linear system At = b, where t corresponds to the (ALL_DICTS) translation.
  // This is then inverted with the associated normal equations to give t = inv(transpose(A)*A)*transpose(A)*b
  // For efficiency we only store the coefficients of (transpose(A)*A) and (transpose(A)*b)
  cv::Mat modelPoints = _objectPoints.getMat();
  cv::Mat imgPoints = _imgPoints.getMat();
  int numPts = modelPoints.rows;

  _t.create(3, 1, CV_64FC1);

  cv::Mat R = _R.getMat();

  // coefficients of (transpose(A) * A)
  double ATA00 = numPts;
  double ATA02 = 0;
  double ATA11 = numPts;
  double ATA12 = 0;
  double ATA20 = 0;
  double ATA21 = 0;
  double ATA22 = 0;

  // coefficients of (transpose(A) * b)
  double ATb0 = 0;
  double ATb1 = 0;
  double ATb2 = 0;

  // S gives inv(transpose(A) * A) / det(A)^2
  double S00, S01, S02;
  double S10, S11, S12;
  double S20, S21, S22;

  double rx, ry, rz;
  double a2;
  double b2;
  double bx, by;

  // now loop through each point and increment the coefficients:
  for (int i = 0; i < numPts; i++)
  {
    rx = R.at<double>(0, 0) * modelPoints.at<cv::Vec3f>(i)(0) + R.at<double>(0, 1) * modelPoints.at<cv::Vec3f>(i)(1)
        + R.at<double>(0, 2) * modelPoints.at<cv::Vec3f>(i)(2);
    ry = R.at<double>(1, 0) * modelPoints.at<cv::Vec3f>(i)(0) + R.at<double>(1, 1) * modelPoints.at<cv::Vec3f>(i)(1)
        + R.at<double>(1, 2) * modelPoints.at<cv::Vec3f>(i)(2);
    rz = R.at<double>(2, 0) * modelPoints.at<cv::Vec3f>(i)(0) + R.at<double>(2, 1) * modelPoints.at<cv::Vec3f>(i)(1)
        + R.at<double>(2, 2) * modelPoints.at<cv::Vec3f>(i)(2);
    a2 = -imgPoints.at<cv::Vec2f>(i)(0);
    b2 = -imgPoints.at<cv::Vec2f>(i)(1);
    ATA02 = ATA02 + a2;
    ATA12 = ATA12 + b2;
    ATA20 = ATA20 + a2;
    ATA21 = ATA21 + b2;
    ATA22 = ATA22 + a2 * a2 + b2 * b2;
    bx = (imgPoints.at<cv::Vec2f>(i)(0)) * rz - rx;
    by = (imgPoints.at<cv::Vec2f>(i)(1)) * rz - ry;
    ATb0 = ATb0 + bx;
    ATb1 = ATb1 + by;
    ATb2 = ATb2 + a2 * bx + b2 * by;
  }

  double detAInv = 1.0 / (ATA00 * ATA11 * ATA22 - ATA00 * ATA12 * ATA21 - ATA02 * ATA11 * ATA20);

  // construct S:
  S00 = ATA11 * ATA22 - ATA12 * ATA21;
  S01 = ATA02 * ATA21;
  S02 = -ATA02 * ATA11;
  S10 = ATA12 * ATA20;
  S11 = ATA00 * ATA22 - ATA02 * ATA20;
  S12 = -ATA00 * ATA12;
  S20 = -ATA11 * ATA20;
  S21 = -ATA00 * ATA21;
  S22 = ATA00 * ATA11;

  // solve t:
  cv::Mat t = _t.getMat();
  t.at<double>(0) = detAInv * (S00 * ATb0 + S01 * ATb1 + S02 * ATb2);
  t.at<double>(1) = detAInv * (S10 * ATb0 + S11 * ATb1 + S12 * ATb2);
  t.at<double>(2) = detAInv * (S20 * ATb0 + S21 * ATb1 + S22 * ATb2);
}

void IPPComputeRotations(double j00, double j01, double j10, double j11, double p, double q, cv::OutputArray _R1,
                         cv::OutputArray _R2)
{
  // Note that it is very hard to understand what is going on here from the code, so if you want to have a clear
  // explanation then please refer to the IPPE paper (Algorithm 1 and its description).
  _R1.create(3, 3, CV_64FC1);
  _R2.create(3, 3, CV_64FC1);

  double a00, a01, a10, a11, ata00, ata01, ata11, b00, b01, b10, b11, binv00, binv01, binv10, binv11;
  double rv00, rv01, rv02, rv10, rv11, rv12, rv20, rv21, rv22;
  double rtilde00, rtilde01, rtilde10, rtilde11;
  double rtilde00_2, rtilde01_2, rtilde10_2, rtilde11_2;
  double b0, b1, gamma, dtinv;
  double s, t, sp, krs0, krs1, krs0_2, krs1_2, costh, sinth;

  s = std::sqrt(p * p + q * q + 1);
  t = std::sqrt(p * p + q * q);
  costh = 1 / s;
  sinth = std::sqrt(1 - 1 / (s * s));

  krs0 = p / t;
  krs1 = q / t;
  krs0_2 = krs0 * krs0;
  krs1_2 = krs1 * krs1;

  rv00 = (costh - 1) * krs0_2 + 1;
  rv01 = krs0 * krs1 * (costh - 1);
  rv02 = krs0 * sinth;
  rv10 = krs0 * krs1 * (costh - 1);
  rv11 = (costh - 1) * krs1_2 + 1;
  rv12 = krs1 * sinth;
  rv20 = -krs0 * sinth;
  rv21 = -krs1 * sinth;
  rv22 = (costh - 1) * (krs0_2 + krs1_2) + 1;

  // setup the 2x2 SVD decomposition:
  b00 = rv00 - p * rv20;
  b01 = rv01 - p * rv21;
  b10 = rv10 - q * rv20;
  b11 = rv11 - q * rv21;

  dtinv = 1.0 / ((b00 * b11 - b01 * b10));

  binv00 = dtinv * b11;
  binv01 = -dtinv * b01;
  binv10 = -dtinv * b10;
  binv11 = dtinv * b00;

  a00 = binv00 * j00 + binv01 * j10;
  a01 = binv00 * j01 + binv01 * j11;
  a10 = binv10 * j00 + binv11 * j10;
  a11 = binv10 * j01 + binv11 * j11;

  // compute the largest singular value of A:
  ata00 = a00 * a00 + a01 * a01;
  ata01 = a00 * a10 + a01 * a11;
  ata11 = a10 * a10 + a11 * a11;

  gamma = std::sqrt(0.5 * (ata00 + ata11 + std::sqrt((ata00 - ata11) * (ata00 - ata11) + 4.0 * ata01 * ata01)));

  // reconstruct the full rotation matrices:
  rtilde00 = a00 / gamma;
  rtilde01 = a01 / gamma;
  rtilde10 = a10 / gamma;
  rtilde11 = a11 / gamma;

  rtilde00_2 = rtilde00 * rtilde00;
  rtilde01_2 = rtilde01 * rtilde01;
  rtilde10_2 = rtilde10 * rtilde10;
  rtilde11_2 = rtilde11 * rtilde11;

  b0 = std::sqrt(-rtilde00_2 - rtilde10_2 + 1);
  b1 = std::sqrt(-rtilde01_2 - rtilde11_2 + 1);
  sp = (-rtilde00 * rtilde01 - rtilde10 * rtilde11);

  if (sp < 0)
  {
    b1 = -b1;
  }

  // save results:
  cv::Mat R1 = _R1.getMat();
  cv::Mat R2 = _R2.getMat();

  R1.at<double>(0, 0) = (rtilde00) * rv00 + (rtilde10) * rv01 + (b0) * rv02;
  R1.at<double>(0, 1) = (rtilde01) * rv00 + (rtilde11) * rv01 + (b1) * rv02;
  R1.at<double>(0, 2) = (b1 * rtilde10 - b0 * rtilde11) * rv00 + (b0 * rtilde01 - b1 * rtilde00) * rv01
      + (rtilde00 * rtilde11 - rtilde01 * rtilde10) * rv02;
  R1.at<double>(1, 0) = (rtilde00) * rv10 + (rtilde10) * rv11 + (b0) * rv12;
  R1.at<double>(1, 1) = (rtilde01) * rv10 + (rtilde11) * rv11 + (b1) * rv12;
  R1.at<double>(1, 2) = (b1 * rtilde10 - b0 * rtilde11) * rv10 + (b0 * rtilde01 - b1 * rtilde00) * rv11
      + (rtilde00 * rtilde11 - rtilde01 * rtilde10) * rv12;
  R1.at<double>(2, 0) = (rtilde00) * rv20 + (rtilde10) * rv21 + (b0) * rv22;
  R1.at<double>(2, 1) = (rtilde01) * rv20 + (rtilde11) * rv21 + (b1) * rv22;
  R1.at<double>(2, 2) = (b1 * rtilde10 - b0 * rtilde11) * rv20 + (b0 * rtilde01 - b1 * rtilde00) * rv21
      + (rtilde00 * rtilde11 - rtilde01 * rtilde10) * rv22;

  R2.at<double>(0, 0) = (rtilde00) * rv00 + (rtilde10) * rv01 + (-b0) * rv02;
  R2.at<double>(0, 1) = (rtilde01) * rv00 + (rtilde11) * rv01 + (-b1) * rv02;
  R2.at<double>(0, 2) = (b0 * rtilde11 - b1 * rtilde10) * rv00 + (b1 * rtilde00 - b0 * rtilde01) * rv01
      + (rtilde00 * rtilde11 - rtilde01 * rtilde10) * rv02;
  R2.at<double>(1, 0) = (rtilde00) * rv10 + (rtilde10) * rv11 + (-b0) * rv12;
  R2.at<double>(1, 1) = (rtilde01) * rv10 + (rtilde11) * rv11 + (-b1) * rv12;
  R2.at<double>(1, 2) = (b0 * rtilde11 - b1 * rtilde10) * rv10 + (b1 * rtilde00 - b0 * rtilde01) * rv11
      + (rtilde00 * rtilde11 - rtilde01 * rtilde10) * rv12;
  R2.at<double>(2, 0) = (rtilde00) * rv20 + (rtilde10) * rv21 + (-b0) * rv22;
  R2.at<double>(2, 1) = (rtilde01) * rv20 + (rtilde11) * rv21 + (-b1) * rv22;
  R2.at<double>(2, 2) = (b0 * rtilde11 - b1 * rtilde10) * rv20 + (b1 * rtilde00 - b0 * rtilde01) * rv21
      + (rtilde00 * rtilde11 - rtilde01 * rtilde10) * rv22;
}

void homographyFromSquarePoints(cv::InputArray _targetPts, double halfLength, cv::OutputArray H_)
{
  cv::Mat pts = _targetPts.getMat();
  H_.create(3, 3, CV_64FC1);
  cv::Mat H = H_.getMat();

  double p1x = -pts.at<cv::Vec2f>(0)(0);
  double p1y = -pts.at<cv::Vec2f>(0)(1);

  double p2x = -pts.at<cv::Vec2f>(1)(0);
  double p2y = -pts.at<cv::Vec2f>(1)(1);

  double p3x = -pts.at<cv::Vec2f>(2)(0);
  double p3y = -pts.at<cv::Vec2f>(2)(1);

  double p4x = -pts.at<cv::Vec2f>(3)(0);
  double p4y = -pts.at<cv::Vec2f>(3)(1);

  // analytic solution:
  double detsInv = -1
      / (halfLength * (p1x * p2y - p2x * p1y - p1x * p4y + p2x * p3y - p3x * p2y + p4x * p1y + p3x * p4y - p4x * p3y));

  H.at<double>(0, 0) = detsInv
      * (p1x * p3x * p2y - p2x * p3x * p1y - p1x * p4x * p2y + p2x * p4x * p1y - p1x * p3x * p4y + p1x * p4x * p3y
          + p2x * p3x * p4y - p2x * p4x * p3y);
  H.at<double>(0, 1) = detsInv
      * (p1x * p2x * p3y - p1x * p3x * p2y - p1x * p2x * p4y + p2x * p4x * p1y + p1x * p3x * p4y - p3x * p4x * p1y
          - p2x * p4x * p3y + p3x * p4x * p2y);
  H.at<double>(0, 2) = detsInv * halfLength
      * (p1x * p2x * p3y - p2x * p3x * p1y - p1x * p2x * p4y + p1x * p4x * p2y - p1x * p4x * p3y + p3x * p4x * p1y
          + p2x * p3x * p4y - p3x * p4x * p2y);
  H.at<double>(1, 0) = detsInv
      * (p1x * p2y * p3y - p2x * p1y * p3y - p1x * p2y * p4y + p2x * p1y * p4y - p3x * p1y * p4y + p4x * p1y * p3y
          + p3x * p2y * p4y - p4x * p2y * p3y);
  H.at<double>(1, 1) = detsInv
      * (p2x * p1y * p3y - p3x * p1y * p2y - p1x * p2y * p4y + p4x * p1y * p2y + p1x * p3y * p4y - p4x * p1y * p3y
          - p2x * p3y * p4y + p3x * p2y * p4y);
  H.at<double>(1, 2) = detsInv * halfLength
      * (p1x * p2y * p3y - p3x * p1y * p2y - p2x * p1y * p4y + p4x * p1y * p2y - p1x * p3y * p4y + p3x * p1y * p4y
          + p2x * p3y * p4y - p4x * p2y * p3y);
  H.at<double>(2, 0) = -detsInv
      * (p1x * p3y - p3x * p1y - p1x * p4y - p2x * p3y + p3x * p2y + p4x * p1y + p2x * p4y - p4x * p2y);
  H.at<double>(2, 1) = detsInv
      * (p1x * p2y - p2x * p1y - p1x * p3y + p3x * p1y + p2x * p4y - p4x * p2y - p3x * p4y + p4x * p3y);
  H.at<double>(2, 2) = 1.0;
}

} // namespace aruco
