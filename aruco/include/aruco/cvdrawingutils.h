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

#ifndef _ArUco_DrawUtils_H_
#define _ArUco_DrawUtils_H_

#include "aruco.h"
#include "aruco_export.h"

namespace aruco
{
/**\brief A set of functions to draw in opencv images
 */
class ARUCO_EXPORT CvDrawingUtils
{
public:
  static void draw3dAxis(cv::Mat& Image, const CameraParameters& CP, const cv::Mat& Rvec,
                         const cv::Mat& Tvec, float axis_size);
  static void draw3dAxis(cv::Mat& Image, Marker& m, const CameraParameters& CP, int lineSize = 1);

  static void draw3dCube(cv::Mat& Image, Marker& m, const CameraParameters& CP,
                         int lineSize = 1, bool setYperpendicular = false);

  //    static void draw3dAxis(cv::Mat &Image, MarkerMap &m, const CameraParameters &CP);
  //    static void draw3dCube(cv::Mat &Image, MarkerMap &m, const CameraParameters &CP,
  //    bool setYperpendicular = false);
};
}  // namespace aruco

#endif
