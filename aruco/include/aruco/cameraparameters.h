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
#ifndef _Aruco_CameraParameters_H
#define _Aruco_CameraParameters_H

#include "aruco_export.h"
#include <opencv2/core/core.hpp>
#include <string>
#include <stdexcept>

namespace aruco
{
/**\brief Parameters of the camera
 */

class ARUCO_EXPORT CameraParameters
{
public:
  // 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
  cv::Mat CameraMatrix;
  //  distortion matrix
  cv::Mat Distorsion;
  // size of the image
  cv::Size CamSize;

  // 3x1 matrix (Tx, Ty, Tz), usually 0 for non-stereo cameras or stereo left cameras
  cv::Mat ExtrinsicMatrix;

  /**Empty constructor
   */
  CameraParameters();
  /**Creates the object from the info passed
   * @param cameraMatrix 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
   * @param distorsionCoeff 4x1 matrix (k1,k2,p1,p2)
   * @param size image size
   */
  CameraParameters(cv::Mat cameraMatrix, cv::Mat distorsionCoeff, cv::Size size);
  /**Sets the parameters
   * @param cameraMatrix 3x3 matrix (fx 0 cx, 0 fy cy, 0 0 1)
   * @param distorsionCoeff 4x1 matrix (k1,k2,p1,p2)
   * @param size image size
   */
  void setParams(cv::Mat cameraMatrix, cv::Mat distorsionCoeff, cv::Size size);
  /**Copy constructor
   */
  CameraParameters(const CameraParameters &CI);

  /**Indicates whether this object is valid
   */
  bool isValid() const
  {
    return CameraMatrix.rows != 0 && CameraMatrix.cols != 0 && Distorsion.rows != 0 &&
           Distorsion.cols != 0 && CamSize.width != -1 && CamSize.height != -1;
  }
  /**Assign operator
   */
  CameraParameters &operator=(const CameraParameters &CI);

  /**Saves this to a file
   */
  void saveToFile(std::string path, bool inXML = true);

  /**Reads from a YAML file generated with the opencv2.2 calibration utility
   */
  void readFromXMLFile(std::string filePath);

  /**Adjust the parameters to the size of the image indicated
   */
  void resize(cv::Size size);

  /**Returns the location of the camera in the reference system of the marker.
   *
   * Rvec and Tvec are the transform from the marker to the camera as calculated in other
   * parts of the library NOT TESTED
   */
  static cv::Point3f getCameraLocation(const cv::Mat &Rvec, const cv::Mat &Tvec);

  /**Given the intrinsic camera parameters returns the GL_PROJECTION matrix for opengl.
   * PLease NOTE that when using OpenGL, it is assumed no camera distorsion! So, if it is
   *not true, you should have undistor image
   *
   * @param orgImgSize size of the original image
   * @param size of the image/window where to render (can be different from the real
   *camera image). Please not that it must be related to CamMatrix
   * @param proj_matrix output projection matrix to give to opengl
   * @param gnear,gfar: visible rendering range
   * @param invert: indicates if the output projection matrix has to yield a horizontally
   *inverted image because image data has not been stored in the order of glDrawPixels:
   *bottom-to-top.
   */
  void glGetProjectionMatrix(cv::Size orgImgSize, cv::Size size, double proj_matrix[16],
                             double gnear, double gfar, bool invert = false);

  /**
   * setup camera for an Ogre project.
   * 	Use:
   * ...
   * Ogre::Matrix4 PM(proj_matrix[0], proj_matrix[1], ... , proj_matrix[15]);
   * yourCamera->setCustomProjectionMatrix(true, PM);
   * yourCamera->setCustomViewMatrix(true, Ogre::Matrix4::IDENTITY);
   * ...
   * As in OpenGL, it assumes no camera distorsion
   */
  void OgreGetProjectionMatrix(cv::Size orgImgSize, cv::Size size, double proj_matrix[16],
                               double gnear, double gfar, bool invert = false);

  /**Returns the 4x4 homogeneous transform matrix from the R and T vectors computed
   */
  static cv::Mat getRTMatrix(const cv::Mat &R_, const cv::Mat &T_, int forceType);


  /**Makes this invalid
   */
  void clear();

  ARUCO_EXPORT friend std::ostream &operator<<(std::ostream &str, const CameraParameters &cp);
  ARUCO_EXPORT friend std::istream &operator>>(std::istream &str, CameraParameters &cp);

private:
  // GL routines

  static void argConvGLcpara2(double cparam[3][4], int width, int height, double gnear,
                              double gfar, double m[16], bool invert);
  static int arParamDecompMat(double source[3][4], double cpara[3][4], double trans[3][4]);
  static double norm(double a, double b, double c);
  static double dot(double a1, double a2, double a3, double b1, double b2, double b3);
};
}  // namespace aruco
#endif
