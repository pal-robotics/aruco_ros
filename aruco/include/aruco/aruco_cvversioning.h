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

#ifndef ARUCO_CV_VERSIONING
#define ARUCO_CV_VERSIONING
#include <opencv2/core/core.hpp>
#if CV_MAJOR_VERSION >= 4
#include <opencv2/imgproc.hpp>

#define CV_CAP_PROP_FRAME_COUNT cv::CAP_PROP_FRAME_COUNT
#define CV_CAP_PROP_POS_FRAMES cv::CAP_PROP_POS_FRAMES
#define CV_BGR2GRAY cv::COLOR_BGR2GRAY
#define CV_GRAY2BGR cv::COLOR_GRAY2BGR
#define CV_FONT_HERSHEY_COMPLEX cv::FONT_HERSHEY_COMPLEX
#define CV_FILLED cv::FILLED
#define CV_RANSAC cv::RANSAC
#define CV_REDUCE_SUM cv::REDUCE_SUM
#endif

#endif
