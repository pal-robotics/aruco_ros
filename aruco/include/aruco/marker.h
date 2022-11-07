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

#ifndef _Aruco_Marker_H
#define _Aruco_Marker_H

#include "aruco_export.h"

#include <opencv2/core/core.hpp>

#include <cstdint>
#include <iostream>
#include <vector>

namespace aruco
{
/**\brief This class represents a marker. It is a vector of the fours corners ot the marker
 *
 */

class CameraParameters;
class ARUCO_EXPORT Marker : public std::vector<cv::Point2f>
{
public:
  // id of  the marker
  int id;
  // size of the markers sides in meters
  float ssize;
  // matrices of rotation and translation respect to the camera
  cv::Mat Rvec, Tvec;
  // additional info about the dictionary
  std::string dict_info;
  // points of the contour
  vector<cv::Point> contourPoints;

  /**
   */
  Marker();
  /**
   */
  Marker(int id);
  /**
   */
  Marker(const Marker& M);
  /**
   */
  Marker(const std::vector<cv::Point2f>& corners, int _id = -1);
  /**
   */
  ~Marker()
  {
  }
  /**Indicates if this object is valid
   */
  bool isValid() const
  {
    return id != -1 && size() == 4;
  }

  bool isPoseValid() const
  {
    return !Rvec.empty() && !Tvec.empty();
  }
  /**Draws this marker in the input image
   */
  void draw(cv::Mat& in, cv::Scalar color = cv::Scalar(0, 0, 255), int lineWidth = -1,
            bool writeId = true, bool writeInfo = false) const;

  /**Calculates the extrinsics (Rvec and Tvec) of the marker with respect to the camera
   * @param markerSize size of the marker side expressed in meters
   * @param CP parmeters of the camera
   * @param setYPerpendicular If set the Y axis will be perpendicular to the surface.
   * Otherwise, it will be the Z axis
   */
  void calculateExtrinsics(float markerSize, const CameraParameters& CP,
                           bool setYPerpendicular = true);
  /**
   * Calculates the extrinsics (Rvec and Tvec) of the marker with respect to the camera
   * @param markerSize size of the marker side expressed in meters
   * @param CameraMatrix matrix with camera parameters (fx, fy, cx, cy)
   * @param Distortion matrix with distortion parameters (k1, k2, p1, p2)
   * @param setYPerpendicular If set the Y axis will be perpendicular to the surface.
   * Otherwise, it will be the Z axis
   * @param correctFisheye Correct fisheye distortion
   */
  void calculateExtrinsics(float markerSize, cv::Mat CameraMatrix,
                           cv::Mat Distorsion = cv::Mat(), cv::Mat Extrinsics = cv::Mat(),
                           bool setYPerpendicular = true, bool correctFisheye = false);

  /**Given the extrinsic camera parameters returns the GL_MODELVIEW matrix for opengl.
   * Setting this matrix, the reference coordinate system will be set in this marker
   */
  void glGetModelViewMatrix(double modelview_matrix[16]);

  /**
   * Returns position vector and orientation quaternion for an Ogre scene node or entity.
   * 	Use:
   * ...
   * Ogre::Vector3 ogrePos (position[0], position[1], position[2]);
   * Ogre::Quaternion  ogreOrient (orientation[0], orientation[1], orientation[2], orientation[3]);
   * mySceneNode->setPosition( ogrePos  );
   * mySceneNode->setOrientation( ogreOrient  );
   * ...
   */
  void OgreGetPoseParameters(double position[3], double orientation[4]);

  /**Returns the centroid of the marker
   */
  cv::Point2f getCenter() const;
  /**Returns the perimeter of the marker
   */
  float getPerimeter() const;
  /**Returns the area
   */
  float getArea() const;
  /**Returns radius of enclosing circle
   */
  float getRadius() const;
  /**compares ids
   */
  bool operator==(const Marker& m) const
  {
    return m.id == id;
  }

  void copyTo(Marker& m) const;
  /**compares ids
   */
  Marker& operator=(const Marker& m);

  /**
   */
  friend bool operator<(const Marker& M1, const Marker& M2)
  {
    return M1.id < M2.id;
  }
  /**
   */
  friend std::ostream& operator<<(std::ostream& str, const Marker& M)
  {
    str << M.id << "=";
    for (int i = 0; i < 4; i++)
      str << "(" << M[i].x << "," << M[i].y << ") ";
    if (!M.Tvec.empty() && !M.Rvec.empty())
    {
      str << "Txyz=";
      for (int i = 0; i < 3; i++)
        str << M.Tvec.ptr<float>(0)[i] << " ";
      str << "Rxyz=";
      for (int i = 0; i < 3; i++)
        str << M.Rvec.ptr<float>(0)[i] << " ";
    }
    return str;
  }


  // saves to a binary stream
  void toStream(std::ostream& str) const;
  // reads from a binary stream
  void fromStream(std::istream& str);

  // returns the 3d points of a marker wrt its center
  static vector<cv::Point3f> get3DPoints(float msize);
  // returns the 3d points of this marker wrt its center
  inline vector<cv::Point3f> get3DPoints() const
  {
    return get3DPoints(ssize);
  }

  // returns the SE3 (4x4) transform matrix

  cv::Mat getTransformMatrix() const;

private:
  void rotateXAxis(cv::Mat& rotation);
};
}  // namespace aruco
#endif
