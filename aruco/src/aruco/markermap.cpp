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

#include "markermap.h"
#include "dictionary.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <fstream>

using namespace std;
using namespace cv;

namespace aruco
{
Marker3DInfo::Marker3DInfo()
{
}
Marker3DInfo::Marker3DInfo(int _id) : id(_id)
{
}
/**
 *
 *
 */
MarkerMap::MarkerMap()
{
  mInfoType = NONE;
}
/**
 *
 *
 */
MarkerMap::MarkerMap(string filePath)
{
  mInfoType = NONE;
  readFromFile(filePath);
}

/**
 *
 *
 */
void MarkerMap::saveToFile(string sfile)
{
  cv::FileStorage fs(sfile, cv::FileStorage::WRITE);
  saveToFile(fs);
}
/**Saves the board info to a file
 */
void MarkerMap::saveToFile(cv::FileStorage& fs)
{
  fs << "aruco_bc_dict" << dictionary;
  fs << "aruco_bc_nmarkers" << (int)size();
  fs << "aruco_bc_mInfoType" << (int)mInfoType;
  fs << "aruco_bc_markers"
     << "[";
  for (size_t i = 0; i < size(); i++)
  {
    fs << "{:"
       << "id" << at(i).id;

    fs << "corners"
       << "[:";
    for (size_t c = 0; c < at(i).size(); c++)
      fs << at(i)[c];
    fs << "]";
    fs << "}";
  }
  fs << "]";
}

/**
 *
 *
 */
void MarkerMap::readFromFile(string sfile)
{
  try
  {
    cv::FileStorage fs(sfile, cv::FileStorage::READ);
    if (!fs.isOpened())
      throw cv::Exception(81818, "MarkerMap::readFromFile",
                          string(" file not opened ") + sfile, __FILE__, __LINE__);
    readFromFile(fs);
  }
  catch (std::exception& ex)
  {
    throw cv::Exception(81818, "MarkerMap::readFromFile",
                        ex.what() + string(" file=)") + sfile, __FILE__, __LINE__);
  }
}

/**Reads board info from a file
 */
void MarkerMap::readFromFile(cv::FileStorage& fs)
{
  int aux = 0;
  // look for the nmarkers
  if (fs["aruco_bc_nmarkers"].name() != "aruco_bc_nmarkers")
    throw cv::Exception(81818, "MarkerMap::readFromFile", "invalid file type", __FILE__, __LINE__);
  fs["aruco_bc_nmarkers"] >> aux;
  resize(aux);
  fs["aruco_bc_mInfoType"] >> mInfoType;
  cv::FileNode markers = fs["aruco_bc_markers"];
  int i = 0;
  for (FileNodeIterator it = markers.begin(); it != markers.end(); ++it, i++)
  {
    at(i).id = (*it)["id"];
    FileNode FnCorners = (*it)["corners"];
    for (FileNodeIterator itc = FnCorners.begin(); itc != FnCorners.end(); ++itc)
    {
      vector<float> coordinates3d;
      (*itc) >> coordinates3d;
      if (coordinates3d.size() != 3)
        throw cv::Exception(81818, "MarkerMap::readFromFile", "invalid file type 3",
                            __FILE__, __LINE__);
      cv::Point3f point(coordinates3d[0], coordinates3d[1], coordinates3d[2]);
      at(i).push_back(point);
    }
  }

  if (fs["aruco_bc_dict"].name() == "aruco_bc_dict")
    fs["aruco_bc_dict"] >> dictionary;
}

/**
 */
int MarkerMap::getIndexOfMarkerId(int id) const
{
  for (size_t i = 0; i < size(); i++)
    if (at(i).id == id)
      return static_cast<int>(i);
  return -1;
}

/**
 */
const Marker3DInfo& MarkerMap::getMarker3DInfo(int id) const
{
  for (size_t i = 0; i < size(); i++)
    if (at(i).id == id)
      return at(i);
  throw cv::Exception(111, "MarkerMap::getMarker3DInfo",
                      "Marker with the id given is not found", __FILE__, __LINE__);
}

void __glGetModelViewMatrix(double modelview_matrix[16], const cv::Mat& Rvec, const cv::Mat& Tvec)
{
  // check if paremeters are valid
  bool invalid = false;
  for (int i = 0; i < 3 && !invalid; i++)
  {
    if (Tvec.at<float>(i, 0) != -999999)
      invalid |= false;
    if (Rvec.at<float>(i, 0) != -999999)
      invalid |= false;
  }
  if (invalid)
    throw cv::Exception(9002, "extrinsic parameters are not set",
                        "Marker::getModelViewMatrix", __FILE__, __LINE__);
  Mat Rot(3, 3, CV_32FC1), Jacob;
  Rodrigues(Rvec, Rot, Jacob);

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

/****
 *
 */
void __OgreGetPoseParameters(double position[3], double orientation[4],
                             const cv::Mat& Rvec, const cv::Mat& Tvec)
{
  // check if paremeters are valid
  bool invalid = false;
  for (int i = 0; i < 3 && !invalid; i++)
  {
    if (Tvec.at<float>(i, 0) != -999999)
      invalid |= false;
    if (Rvec.at<float>(i, 0) != -999999)
      invalid |= false;
  }
  if (invalid)
    throw cv::Exception(9003, "extrinsic parameters are not set",
                        "Marker::getModelViewMatrix", __FILE__, __LINE__);

  // calculate position vector
  position[0] = -Tvec.ptr<float>(0)[0];
  position[1] = -Tvec.ptr<float>(0)[1];
  position[2] = +Tvec.ptr<float>(0)[2];

  // now calculare orientation quaternion
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
    fRoot = sqrt(fTrace + 1.0);  // 2w
    orientation[0] = 0.5 * fRoot;
    fRoot = 0.5 / fRoot;  // 1/(4w)
    orientation[1] = (axes[2][1] - axes[1][2]) * fRoot;
    orientation[2] = (axes[0][2] - axes[2][0]) * fRoot;
    orientation[3] = (axes[1][0] - axes[0][1]) * fRoot;
  }
  else
  {
    // |w| <= 1/2
    static unsigned int s_iNext[3] = { 1, 2, 0 };
    unsigned int i = 0;
    if (axes[1][1] > axes[0][0])
      i = 1;
    if (axes[2][2] > axes[i][i])
      i = 2;
    unsigned int j = s_iNext[i];
    unsigned int k = s_iNext[j];

    fRoot = sqrt(axes[i][i] - axes[j][j] - axes[k][k] + 1.0);
    double* apkQuat[3] = { &orientation[1], &orientation[2], &orientation[3] };
    *apkQuat[i] = 0.5 * fRoot;
    fRoot = 0.5 / fRoot;
    orientation[0] = (axes[k][j] - axes[j][k]) * fRoot;
    *apkQuat[j] = (axes[j][i] + axes[i][j]) * fRoot;
    *apkQuat[k] = (axes[k][i] + axes[i][k]) * fRoot;
  }
}

/**
 */
void MarkerMap::getIdList(std::vector<int>& ids, bool append) const
{
  if (!append)
    ids.clear();
  for (size_t i = 0; i < size(); i++)
    ids.push_back(at(i).id);
}

MarkerMap MarkerMap::convertToMeters(float markerSize_meters) const
{
  if (!isExpressedInPixels())
    throw cv::Exception(-1, "The board is not expressed in pixels",
                        "MarkerMap::convertToMeters", __FILE__, __LINE__);
  // first, we are assuming all markers are equally sized. So, lets get the size in pixels
  int markerSizePix = static_cast<int>(cv::norm(at(0)[0] - at(0)[1]));
  MarkerMap BInfo(*this);
  BInfo.mInfoType = MarkerMap::METERS;
  // now, get the size of a pixel, and change scale
  float pixSize = markerSize_meters / float(markerSizePix);
  //        cout << markerSize_meters << " " << float(markerSizePix) << " " << pixSize <<
  //        endl;
  for (size_t i = 0; i < BInfo.size(); i++)
    for (int c = 0; c < 4; c++)
    {
      BInfo[i][c] *= pixSize;
    }
  return BInfo;
}
cv::Mat MarkerMap::getImage(float METER2PIX) const
{
  if (mInfoType == NONE)
    throw cv::Exception(-1, "The board is not valid mInfoType==NONE  ",
                        "MarkerMap::getImage", __FILE__, __LINE__);
  if (METER2PIX <= 0 && mInfoType != PIX)
    throw cv::Exception(-1, "The board is not expressed in pixels and not METER2PIX indicated",
                        "MarkerMap::getImage", __FILE__, __LINE__);

  auto Dict = Dictionary::loadPredefined(dictionary);

  // get image limits
  cv::Point pmin(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()),
      pmax(std::numeric_limits<int>::lowest(), std::numeric_limits<int>::lowest());
  for (auto b : *this)
  {
    for (auto p : b.points)
    {
      pmin.x = min(int(p.x), pmin.x);
      pmin.y = min(int(p.y), pmin.y);
      pmax.x = max(int(p.x + 0.5), pmax.x);
      pmax.y = max(int(p.y + 0.5), pmax.y);
      assert(p.z == 0);
    }
  }

  cv::Point psize = pmax - pmin;
  cv::Mat image(cv::Size(psize.x, psize.y), CV_8UC1);
  image.setTo(cv::Scalar::all(255));

  vector<Marker3DInfo> p3d = *this;
  // the points must be moved from a real reference system to image reference sysmte (y
  // positive is inverse)
  for (auto& m : p3d)
    for (auto& p : m.points)
    {
      p -= cv::Point3f(static_cast<float>(pmin.x), static_cast<float>(pmax.y), 0.f);
      // now, use inverse y
      p.y = -p.y;
    }
  for (auto m : p3d)
  {
    // get size and find size of this
    const float size = static_cast<float>(cv::norm(m[0] - m[1]));
    auto im1 = Dict.getMarkerImage_id(m.id, int(size / 8));
    cv::Mat im2;
    // now resize to fit
    cv::resize(im1, im2, cv::Size(static_cast<int>(size), static_cast<int>(size)));
    // copy in correct position
    auto ry = cv::Range(int(m[0].y), int(m[2].y));
    auto rx = cv::Range(int(m[0].x), int(m[2].x));
    cv::Mat sub = image(ry, rx);
    im2.copyTo(sub);
  }
  return image;
}

std::vector<int> MarkerMap::getIndices(const vector<aruco::Marker>& markers) const
{
  std::vector<int> indices;
  for (size_t i = 0; i < markers.size(); i++)
  {
    bool found = false;
    for (size_t j = 0; j < size() && !found; j++)
    {
      if (markers[i].id == at(j).id)
      {
        found = true;
        indices.push_back(static_cast<int>(i));
      }
    }
  }
  return indices;
}
void MarkerMap::toStream(std::ostream& str)
{
  str << mInfoType << " " << size() << " ";
  for (size_t i = 0; i < size(); i++)
  {
    at(i).toStream(str);
  }
  // write dic string info
  str << dictionary;
}
void MarkerMap::fromStream(std::istream& str)
{
  int s;
  str >> mInfoType >> s;
  resize(s);
  for (size_t i = 0; i < size(); i++)
    at(i).fromStream(str);
  str >> dictionary;
}
pair<cv::Mat, cv::Mat> MarkerMap::calculateExtrinsics(const std::vector<aruco::Marker>& markers,
                                                      float markerSize, cv::Mat CameraMatrix,
                                                      cv::Mat Distorsion)
{
  vector<cv::Point2f> p2d;
  MarkerMap m_meters;
  if (isExpressedInPixels())
    m_meters = convertToMeters(markerSize);
  else
    m_meters = *this;
  vector<cv::Point3f> p3d;
  for (auto marker : markers)
  {
    auto it = find(m_meters.begin(), m_meters.end(), marker.id);
    if (it != m_meters.end())
    {  // is the marker part of the map?
      for (auto p : marker)
        p2d.push_back(p);
      for (auto p : it->points)
        p3d.push_back(p);
    }
  }

  cv::Mat rvec, tvec;
  if (p2d.size() != 0)
  {  // no points in the vector
    cv::solvePnPRansac(p3d, p2d, CameraMatrix, Distorsion, rvec, tvec);
  }
  if (rvec.type() == CV_64F)
    rvec.convertTo(rvec, CV_64F);
  if (tvec.type() == CV_64F)
    tvec.convertTo(tvec, CV_64F);
  return make_pair(rvec, tvec);
}
};  // namespace aruco
