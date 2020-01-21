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

#include "marker.h"

/// \todo set this definition in the cmake code
#define _USE_MATH_DEFINES

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <cstdio>
#include <math.h>
#include "cameraparameters.h"

namespace aruco
{

/**
 *
 */
Marker::Marker()
{
  id = -1;
  ssize = -1;
  Rvec.create(3, 1, CV_32FC1);
  Tvec.create(3, 1, CV_32FC1);
  for (int i = 0; i < 3; i++)
    Tvec.at<float>(i, 0) = Rvec.at<float>(i, 0) = -999999;
}

/**
 *
 */
Marker::Marker(int _id)
{
  id = _id;
  ssize = -1;
  Rvec.create(3, 1, CV_32FC1);
  Tvec.create(3, 1, CV_32FC1);
  for (int i = 0; i < 3; i++)
    Tvec.at<float>(i, 0) = Rvec.at<float>(i, 0) = -999999;
}

/**
 *
 */
Marker::Marker(const Marker& M) :
    std::vector<cv::Point2f>(M)
{
  M.copyTo(*this);
}

/**
 *
 */
Marker::Marker(const std::vector<cv::Point2f>& corners, int _id) :
    std::vector<cv::Point2f>(corners)
{
  id = _id;
  ssize = -1;
  Rvec.create(3, 1, CV_32FC1);
  Tvec.create(3, 1, CV_32FC1);
  for (int i = 0; i < 3; i++)
    Tvec.at<float>(i, 0) = Rvec.at<float>(i, 0) = -999999;
}

void Marker::copyTo(Marker &m) const
{
  m.id = id;

  // size of the markers sides in meters
  m.ssize = ssize;

  // matrices of rotation and translation respect to the camera
  Rvec.copyTo(m.Rvec);
  Tvec.copyTo(m.Tvec);
  m.resize(size());
  for (std::size_t i = 0; i < size(); i++)
    m.at(i) = at(i);
  m.dict_info = dict_info;
}

/**
 * Compares ids
 */
Marker & Marker::operator=(const Marker& m)
{
  m.copyTo(*this);
  return *this;
}

/**
 *
 */
void Marker::glGetModelViewMatrix(double modelview_matrix[16])
{
  // check if parameters are valid
  bool invalid = false;
  for (int i = 0; i < 3 && !invalid; i++)
  {
    if (Tvec.at<float>(i, 0) != -999999)
      invalid |= false;
    if (Rvec.at<float>(i, 0) != -999999)
      invalid |= false;
  }
  if (invalid)
    throw cv::Exception(9003, "extrinsic parameters are not set", "Marker::getModelViewMatrix", __FILE__,
    __LINE__);
  cv::Mat Rot(3, 3, CV_32FC1), Jacob;
  cv::Rodrigues(Rvec, Rot, Jacob);

  double para[3][4];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      para[i][j] = Rot.at<float>(i, j);

  // now, add the translation
  para[0][3] = Tvec.at<float>(0, 0);
  para[1][3] = Tvec.at<float>(1, 0);
  para[2][3] = Tvec.at<float>(2, 0);
  double scale = 1;

  modelview_matrix[0 + 0 * 4] = para[0][0];

  // R1C2
  modelview_matrix[0 + 1 * 4] = para[0][1];
  modelview_matrix[0 + 2 * 4] = para[0][2];
  modelview_matrix[0 + 3 * 4] = para[0][3];

  // R2
  modelview_matrix[1 + 0 * 4] = para[1][0];
  modelview_matrix[1 + 1 * 4] = para[1][1];
  modelview_matrix[1 + 2 * 4] = para[1][2];
  modelview_matrix[1 + 3 * 4] = para[1][3];

  // R3
  modelview_matrix[2 + 0 * 4] = -para[2][0];
  modelview_matrix[2 + 1 * 4] = -para[2][1];
  modelview_matrix[2 + 2 * 4] = -para[2][2];
  modelview_matrix[2 + 3 * 4] = -para[2][3];
  modelview_matrix[3 + 0 * 4] = 0.0;
  modelview_matrix[3 + 1 * 4] = 0.0;
  modelview_matrix[3 + 2 * 4] = 0.0;
  modelview_matrix[3 + 3 * 4] = 1.0;
  if (scale != 0.0)
  {
    modelview_matrix[12] *= scale;
    modelview_matrix[13] *= scale;
    modelview_matrix[14] *= scale;
  }
}

/**
 *
 */
void Marker::OgreGetPoseParameters(double position[3], double orientation[4])
{
  // check if parameters are valid
  bool invalid = false;
  for (int i = 0; i < 3 && !invalid; i++)
  {
    if (Tvec.at<float>(i, 0) != -999999)
      invalid |= false;
    if (Rvec.at<float>(i, 0) != -999999)
      invalid |= false;
  }
  if (invalid)
    throw cv::Exception(9003, "extrinsic parameters are not set", "Marker::getModelViewMatrix", __FILE__,
    __LINE__);

  // calculate position vector
  position[0] = -Tvec.ptr<float>(0)[0];
  position[1] = -Tvec.ptr<float>(0)[1];
  position[2] = +Tvec.ptr<float>(0)[2];

  // now calculate orientation quaternion
  cv::Mat Rot(3, 3, CV_32FC1);
  cv::Rodrigues(Rvec, Rot);

  // calculate axes for quaternion
  double stAxes[3][3];

  // x axis
  stAxes[0][0] = -Rot.at<float>(0, 0);
  stAxes[0][1] = -Rot.at<float>(1, 0);
  stAxes[0][2] = +Rot.at<float>(2, 0);

  // y axis
  stAxes[1][0] = -Rot.at<float>(0, 1);
  stAxes[1][1] = -Rot.at<float>(1, 1);
  stAxes[1][2] = +Rot.at<float>(2, 1);

  // for z axis, we use cross product
  stAxes[2][0] = stAxes[0][1] * stAxes[1][2] - stAxes[0][2] * stAxes[1][1];
  stAxes[2][1] = -stAxes[0][0] * stAxes[1][2] + stAxes[0][2] * stAxes[1][0];
  stAxes[2][2] = stAxes[0][0] * stAxes[1][1] - stAxes[0][1] * stAxes[1][0];

  // transposed matrix
  double axes[3][3];
  axes[0][0] = stAxes[0][0];
  axes[1][0] = stAxes[0][1];
  axes[2][0] = stAxes[0][2];

  axes[0][1] = stAxes[1][0];
  axes[1][1] = stAxes[1][1];
  axes[2][1] = stAxes[1][2];

  axes[0][2] = stAxes[2][0];
  axes[1][2] = stAxes[2][1];
  axes[2][2] = stAxes[2][2];

  // Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
  // article "Quaternion Calculus and Fast Animation".
  double fTrace = axes[0][0] + axes[1][1] + axes[2][2];
  double fRoot;

  if (fTrace > 0.0)
  {
    // |w| > 1/2, may as well choose w > 1/2
    fRoot = sqrt(fTrace + 1.0); // 2w
    orientation[0] = 0.5 * fRoot;
    fRoot = 0.5 / fRoot; // 1/(4w)
    orientation[1] = (axes[2][1] - axes[1][2]) * fRoot;
    orientation[2] = (axes[0][2] - axes[2][0]) * fRoot;
    orientation[3] = (axes[1][0] - axes[0][1]) * fRoot;
  }
  else
  {
    // |w| <= 1/2
    static unsigned int s_iNext[3] = {1, 2, 0};
    unsigned int i = 0;
    if (axes[1][1] > axes[0][0])
      i = 1;
    if (axes[2][2] > axes[i][i])
      i = 2;
    unsigned int j = s_iNext[i];
    unsigned int k = s_iNext[j];

    fRoot = sqrt(axes[i][i] - axes[j][j] - axes[k][k] + 1.0);
    double* apkQuat[3] = {&orientation[1], &orientation[2], &orientation[3]};
    *apkQuat[i] = 0.5 * fRoot;
    fRoot = 0.5 / fRoot;
    orientation[0] = (axes[k][j] - axes[j][k]) * fRoot;
    *apkQuat[j] = (axes[j][i] + axes[i][j]) * fRoot;
    *apkQuat[k] = (axes[k][i] + axes[i][k]) * fRoot;
  }
}

void Marker::draw(cv::Mat& in, cv::Scalar color, int lineWidth, bool writeId, bool writeInfo) const
{
  if (size() != 4)
    return;
  if (lineWidth == -1) // auto
    lineWidth = static_cast<int>(std::max(1.f, float(in.cols) / 1000.f));
  cv::line(in, (*this)[0], (*this)[1], color, lineWidth);
  cv::line(in, (*this)[1], (*this)[2], color, lineWidth);
  cv::line(in, (*this)[2], (*this)[3], color, lineWidth);
  cv::line(in, (*this)[3], (*this)[0], color, lineWidth);

//  auto p2 = cv::Point2f(2.f * static_cast<float>(lineWidth), 2.f * static_cast<float>(lineWidth));
//  cv::rectangle(in, (*this)[0] - p2, (*this)[0] + p2, cv::Scalar(0, 0, 255, 255), -1, CV_AA);
//  cv::rectangle(in, (*this)[1] - p2, (*this)[1] + p2, cv::Scalar(0, 255, 0, 255), lineWidth, CV_AA);
//  cv::rectangle(in, (*this)[2] - p2, (*this)[2] + p2, cv::Scalar(255, 0, 0, 255), lineWidth, CV_AA);

  if (writeId)
  {
    // determine the centroid
    cv::Point cent(0, 0);
    for (int i = 0; i < 4; i++)
    {
      cent.x += static_cast<int>((*this)[i].x);
      cent.y += static_cast<int>((*this)[i].y);
    }
    cent.x /= 4;
    cent.y /= 4;
    std::string str;
    if (writeInfo)
      str += dict_info + ":";
    if (writeId)
      str += std::to_string(id);
    cv::putText(in, str, cent, cv::FONT_HERSHEY_SIMPLEX, std::max(0.5f, float(lineWidth) * 0.3f),
                cv::Scalar(255 - color[0], 255 - color[1], 255 - color[2], 255), std::max(lineWidth, 2));
  }
}

/**
 */
void Marker::calculateExtrinsics(float markerSize, const CameraParameters& CP, bool setYPerpendicular)
{
  if (!CP.isValid())
    throw cv::Exception(9004, "!CP.isValid(): invalid camera parameters. It is not possible to calculate extrinsics",
                        "calculateExtrinsics", __FILE__, __LINE__);
  calculateExtrinsics(markerSize, CP.CameraMatrix, CP.Distorsion, CP.ExtrinsicMatrix, setYPerpendicular);
}

void print(cv::Point3f p, std::string cad)
{
  std::cout << cad << " " << p.x << " " << p.y << " " << p.z << std::endl;
}

/**
 */
void Marker::calculateExtrinsics(float markerSizeMeters, cv::Mat camMatrix, cv::Mat distCoeff, cv::Mat Extrinsics, bool setYPerpendicular, bool correctFisheye)
{
  if (!isValid())
    throw cv::Exception(9004, "!isValid(): invalid marker. It is not possible to calculate extrinsics",
                        "calculateExtrinsics", __FILE__, __LINE__);
  if (markerSizeMeters <= 0)
    throw cv::Exception(9004, "markerSize<=0: invalid markerSize", "calculateExtrinsics", __FILE__, __LINE__);
  if (camMatrix.rows == 0 || camMatrix.cols == 0)
    throw cv::Exception(9004, "CameraMatrix is empty", "calculateExtrinsics", __FILE__, __LINE__);

  std::vector<cv::Point3f> objpoints = get3DPoints(markerSizeMeters);

  cv::Mat raux, taux;
  if (correctFisheye)
  {
    std::vector<cv::Point2f> undistorted;
    cv::fisheye::undistortPoints(*this, undistorted, camMatrix, distCoeff);
    cv::solvePnP(objpoints, undistorted, cv::Mat::eye(camMatrix.size(), camMatrix.type()), cv::Mat::zeros(distCoeff.size(), distCoeff.type()), raux, taux);
  }
  else
  {
    cv::solvePnP(objpoints, *this, camMatrix, distCoeff, raux, taux);
  }
  raux.convertTo(Rvec, CV_32F);
  taux.convertTo(Tvec, CV_32F);
  float tx = -1 * (Extrinsics.at<double>(0,0) / camMatrix.at<float>(0,0));
  Tvec.at<float>(0) += tx;
  float ty = -1 * (Extrinsics.at<double>(0,1) / camMatrix.at<float>(1,1));
  Tvec.at<float>(1) += ty;
  float tz = -1 * (Extrinsics.at<double>(0,2) / camMatrix.at<float>(2,2));
  Tvec.at<float>(2) += tz;
  
  // rotate the X axis so that Y is perpendicular to the marker plane
  if (setYPerpendicular)
    rotateXAxis(Rvec);
  ssize = markerSizeMeters;
//  cout << (*this) << endl;
}

std::vector<cv::Point3f> Marker::get3DPoints(float msize)
{
  float halfSize = msize / 2.f;
//  std::vector<cv::Point3f> res(4);
//  res[0] = cv::Point3f(-halfSize, halfSize, 0);
//  res[1] = cv::Point3f(halfSize, halfSize, 0);
//  res[2] = cv::Point3f(halfSize, -halfSize, 0);
//  res[3] = cv::Point3f(-halfSize, -halfSize, 0);
//  return res;
  return
  { cv::Point3f(-halfSize, halfSize, 0),cv::Point3f(halfSize, halfSize, 0),cv::Point3f(halfSize,-halfSize, 0),cv::Point3f(-halfSize, -halfSize, 0)};
}

void Marker::rotateXAxis(cv::Mat& rotation)
{
  cv::Mat R(3, 3, CV_32F);
  cv::Rodrigues(rotation, R);

  // create a rotation matrix for x axis
  cv::Mat RX = cv::Mat::eye(3, 3, CV_32F);
  const float angleRad = 3.14159265359f / 2.f;
  RX.at<float>(1, 1) = cos(angleRad);
  RX.at<float>(1, 2) = -sin(angleRad);
  RX.at<float>(2, 1) = sin(angleRad);
  RX.at<float>(2, 2) = cos(angleRad);

  // now multiply
  R = R * RX;

  // finally, get the Rodrigues back
  Rodrigues(R, rotation);
}

/**
 */
cv::Point2f Marker::getCenter() const
{
  cv::Point2f cent(0, 0);
  for (std::size_t i = 0; i < size(); i++)
  {
    cent.x += (*this)[i].x;
    cent.y += (*this)[i].y;
  }
  cent.x /= float(size());
  cent.y /= float(size());
  return cent;
}

/**
 */
float Marker::getArea() const
{
  assert(size() == 4);
  // use the cross products
  cv::Point2f v01 = (*this)[1] - (*this)[0];
  cv::Point2f v03 = (*this)[3] - (*this)[0];
  float area1 = fabs(v01.x * v03.y - v01.y * v03.x);
  cv::Point2f v21 = (*this)[1] - (*this)[2];
  cv::Point2f v23 = (*this)[3] - (*this)[2];
  float area2 = fabs(v21.x * v23.y - v21.y * v23.x);
  return (area2 + area1) / 2.f;
}

/**
 */
float Marker::getPerimeter() const
{
  assert(size() == 4);
  float sum = 0;
  for (int i = 0; i < 4; i++)
    sum += static_cast<float>(norm((*this)[i] - (*this)[(i + 1) % 4]));
  return sum;
}

// saves to a binary stream
void Marker::toStream(std::ostream& str) const
{
  assert(Rvec.type() == CV_32F && Tvec.type() == CV_32F);
  str.write((char*)&id, sizeof(id));
  str.write((char*)&ssize, sizeof(ssize));
  str.write((char*)Rvec.ptr<float>(0), 3 * sizeof(float));
  str.write((char*)Tvec.ptr<float>(0), 3 * sizeof(float));

  // write the 2D points
  uint32_t np = static_cast<uint32_t>(size());
  str.write((char*)&np, sizeof(np));
  for (std::size_t i = 0; i < size(); i++)
    str.write((char*)&at(i), sizeof(cv::Point2f));

  // write the additional info
  uint32_t s = dict_info.size();
  str.write((char*)&s, sizeof(s));
  str.write((char*)&dict_info[0], dict_info.size());
}

// reads from a binary stream
void Marker::fromStream(std::istream& str)
{
  Rvec.create(1, 3, CV_32F);
  Tvec.create(1, 3, CV_32F);
  str.read((char*)&id, sizeof(id));
  str.read((char*)&ssize, sizeof(ssize));
  str.read((char*)Rvec.ptr<float>(0), 3 * sizeof(float));
  str.read((char*)Tvec.ptr<float>(0), 3 * sizeof(float));
  uint32_t np;
  str.read((char*)&np, sizeof(np));
  resize(np);
  for (std::size_t i = 0; i < size(); i++)
    str.read((char*)&(*this)[i], sizeof(cv::Point2f));

  //read the additional info
  uint32_t s;
  str.read((char*)&s, sizeof(s));
  dict_info.resize(s);
  str.read((char*)&dict_info[0], dict_info.size());
}

} // namespace aruco
