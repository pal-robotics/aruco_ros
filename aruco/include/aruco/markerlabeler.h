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

#ifndef _aruco_MarkerLabeler_
#define _aruco_MarkerLabeler_

#include "aruco_export.h"
#include "dictionary.h"
#include <opencv2/core/core.hpp>

namespace aruco
{
/**\brief Base class of labelers. A labelers receive a square of the image and determines
 * if it has a valid marker, its id and rotation Additionally, it implements the factory
 * model
 */

class Marker;
class ARUCO_EXPORT MarkerLabeler
{
public:
  /** Factory function that returns a labeler for a given dictionary
   * @param dict_type type of dictionary
   * @param error_correction_rate some dictionaries are subsceptible of error correction.
   * This params specify the correction rate. 0 means no correction at all. 1 means full
   * correction (maximum correction bits = (tau-1) /2, tau= predefined mimum intermarker
   * distance).
   *
   * If you want correction capabilities and not sure how much, use 0.5 in this parameter
   */
  static cv::Ptr<MarkerLabeler> create(Dictionary::DICT_TYPES dict_type,
                                       float error_correction_rate = 0);

  /** Factory function that returns the desired detector

   *
   * @brief create Factory function that returns the desired detector
   * @param detector
   *      * Possible names implemented are:
   * ARUCO,CHILITAGS....: original aruco markers (0-1024)
   http://www.sciencedirect.com/science/article/pii/S0031320314000235
   * SVM:
   * @return
   */
  static cv::Ptr<MarkerLabeler> create(std::string detector, std::string params = "");

  /** function that identifies a marker.
   * @param in input image to analyze
   * @param marker_id id of the marker (if valid)
   * @param nRotations :   output parameter nRotations must indicate how many times the
   * marker  must be rotated clockwise 90 deg  to be in its ideal position. (The way you
   * would see it when you print it). This is employed to know always which is the corner
   * that acts as reference system.
   * @return true marker valid, false otherwise
   */
  virtual bool detect(const cv::Mat& in, int& marker_id, int& nRotations,
                      std::string& additionalInfo) = 0;
  /**
   * @brief getBestInputSize if desired, you can set the desired input size to the detect function
   * @return -1 if detect accept any type of input, or a size otherwise
   */
  virtual int getBestInputSize()
  {
    return -1;
  }

  /**
   * @brief getNSubdivisions returns the number of subdivisions in each axis that the
   * iamge will be subject to. This is in dictionary based labelers, equals to the number
   * of bits in each dimension plus the border bits.
   * @return
   */
  virtual int getNSubdivisions() const
  {
    return -1;
  }

  // returns an string that describes the labeler and can be used to create it
  virtual std::string getName() const = 0;
  virtual ~MarkerLabeler()
  {
  }
};
};  // namespace aruco
#endif
