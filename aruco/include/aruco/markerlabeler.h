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

#ifndef _aruco_MarkerLabeler_
#define _aruco_MarkerLabeler_

#include "aruco_export.h"
#include "dictionary.h"
#include <opencv2/core.hpp>

namespace aruco
{

/**
 * \brief Base class of labelers. A labelers receive a square of the image and determines if it has a valid marker,
 * its id and rotation
 * Additionally, it implements the factory model
 */
class Marker;

class ARUCO_EXPORT MarkerLabeler
{
public:
  /**
   * Factory function that returns a labeler for a given dictionary
   * @param dict_type type of dictionary
   * @param error_correction_rate some dictionaries are susceptible of error correction. These params specify the
   * correction rate: 0 means no correction at all, 1 means full correction (maximum correction bits = (tau - 1) / 2,
   * tau = predefined minimum intermarker distance).
   *
   * If you want correction capabilities and not sure how much, use 0.5 in this parameter
   */
  static cv::Ptr<MarkerLabeler> create(Dictionary::DICT_TYPES dict_type, float error_correction_rate = 0);

  /**
   * Factory function that returns the desired detector
   *
   * @brief create Factory function that returns the desired detector
   * @param detector
   *      * Possible names implemented are:
   * ARUCO,CHILITAGS....: original ArUco markers (0-1024)
   http://www.sciencedirect.com/science/article/pii/S0031320314000235
   * SVM:
   * @return
   */
  static cv::Ptr<MarkerLabeler> create(std::string detector, std::string params = "");

  /**
   * Function that identifies a marker.
   * @param in input image to analyze
   * @param marker_id id of the marker (if valid)
   * @param nRotations :   output parameter nRotations must indicate how many times the marker  must be rotated
   * clockwise 90 deg  to be in its ideal position. (The way you would see it when you print it). This is employed
   * to know always which is the corner that acts as reference system.
   * @return true marker valid, false otherwise
   */
  virtual bool detect(const cv::Mat& in, int& marker_id, int& nRotations, std::string &additionalInfo) = 0;

  /**
   * @brief getBestInputSize if desired, you can set the desired input size to the detect function
   * @return -1 if detect accept any type of input, or a size otherwise
   */
  virtual int getBestInputSize()
  {
    return -1;
  }

  /**
   * @brief getNSubdivisions returns the number of subdivisions in each axis that the iamge will be subject to.
   * This is in dictionary based labelers, equals to the number of bits in each dimension plus the border bits.
   * @return
   */
  virtual int getNSubdivisions() const
  {
    return -1;
  }

  // returns a string that describes the labeler and can be used to create it
  virtual std::string getName() const = 0;

  virtual ~MarkerLabeler()
  {
  }
};

} // namespace aruco

#endif /* _aruco_MarkerLabeler_ */
