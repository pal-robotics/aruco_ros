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

#ifndef SVMMARKERS_H
#define SVMMARKERS_H

#include "markerlabeler.h"

namespace aruco
{

/**
 * SVM Marker Detector Class
 */
namespace impl
{

class SVMMarkers;

}

class SVMMarkers : public MarkerLabeler
{
  impl::SVMMarkers *_impl;
public:

  SVMMarkers();
  virtual ~SVMMarkers()
  {
  }

  /**
   * @brief getName
   * @return
   */
  std::string getName() const
  {
    return "SVM";
  }

  // loads the svm file that detects the markers
  bool load(std::string path = "");

  /**
   * Detect marker in a canonical image.
   * Return marker id in 0 rotation, or -1 if not found
   * Assign the detected rotation of the marker to nRotation
   */
  bool detect(const cv::Mat &in, int & marker_id, int &nRotations, std::string &additionalInfo);
  int getBestInputSize();
};

} // namespace aruco

#endif /* SVMMARKERS_H */
