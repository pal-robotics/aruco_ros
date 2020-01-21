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

#ifndef _Aruco_Marker_H
#define _Aruco_Marker_H

#include "aruco_export.h"

#include <opencv2/core.hpp>

#include <cstdint>
#include <iostream>
#include <vector>

namespace aruco
{

/**
 * \brief This class represents a marker. It is a vector of the fours corners of the marker
 */
class CameraParameters;

class ARUCO_EXPORT Marker : public std::vector<cv::Point2f>
{
public:
  // id of the marker
  int id;
  // size of the markers' sides in meters
  float ssize;
  // rotation and translation matrices with respect to the camera
  cv::Mat Rvec, Tvec;
  // additional info about the dictionary
  std::string dict_info;

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

  /**
   * Indicates if this object is valid
   */
  bool isValid() const
  {
    return id != -1 && size() == 4;
  }

  bool isPoseValid() const
  {
    return !Rvec.empty() && !Tvec.empty();
  }

  /**
   * Draws this marker in the input image
   */
  void draw(cv::Mat& in, cv::Scalar color = cv::Scalar(0, 0, 255), int lineWidth = -1, bool writeId = true,
            bool writeInfo = false) const;

  /**
   * Calculates the extrinsics (Rvec and Tvec) of the marker with respect to the camera
   * @param markerSize size of the marker side expressed in meters
   * @param CP parameters of the camera
   * @param setYPerpendicular If set the Y axis will be perpendicular to the surface. Otherwise, it will be the Z axis
   */
  void calculateExtrinsics(float markerSize, const CameraParameters& CP, bool setYPerpendicular = true);

  /**
   * Calculates the extrinsics (Rvec and Tvec) of the marker with respect to the camera
   * @param markerSize size of the marker side expressed in meters
   * @param CameraMatrix matrix with camera parameters (fx, fy, cx, cy)
   * @param Distortion matrix with distortion parameters (k1, k2, p1, p2)
   * @param setYPerpendicular If set the Y axis will be perpendicular to the surface. Otherwise, it will be the Z axis
   * @param correctFisheye Correct fisheye distortion
   */
  void calculateExtrinsics(float markerSize, cv::Mat CameraMatrix, cv::Mat Distorsion = cv::Mat(), cv::Mat Extrinsics = cv::Mat(),
                           bool setYPerpendicular = true, bool correctFisheye = false);

  /**
   * Given the extrinsic camera parameters returns the GL_MODELVIEW matrix for OpenGL.
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

  /**
   * Returns the centroid of the marker
   */
  cv::Point2f getCenter() const;

  /**
   * Returns the perimeter of the marker
   */
  float getPerimeter() const;

  /**
   * Returns the area
   */
  float getArea() const;

  /**
   * Compares ids
   */
  bool operator==(const Marker& m) const
  {
    return m.id == id;
  }

  void copyTo(Marker &m) const;

  /**
   * Compares ids
   */
  Marker & operator=(const Marker& m);

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

  // returns the 3D points of a marker wrt its center
  static std::vector<cv::Point3f> get3DPoints(float msize);

  //returns the 3D points of this marker wrt its center
  inline std::vector<cv::Point3f> get3DPoints() const
  {
    return get3DPoints(ssize);
  }

private:
  void rotateXAxis(cv::Mat& rotation);
};

} // namespace aruco

#endif /* _Aruco_Marker_H */
