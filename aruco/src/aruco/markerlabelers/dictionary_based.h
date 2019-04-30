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

#ifndef ArucoDictionaryBasedMarkerDetector_H
#define ArucoDictionaryBasedMarkerDetector_H

#include "dictionary.h"
#include "markerlabeler.h"

#include <opencv2/core/core.hpp>

namespace aruco
{

/**Labeler using a dictionary
 */
class DictionaryBased : public MarkerLabeler
{
public:
  virtual ~DictionaryBased()
  {
  }

  // first, dictionary, second the maximum correction rate [0,1]. If 0, no correction, if 1, maximum allowed correction
  void setParams(const Dictionary& dic, float max_correction_rate);

  // main virtual class to o detection
  bool detect(const cv::Mat& in, int& marker_id, int& nRotations, std::string &additionalInfo);

  // returns the dictionary name
  std::string getName() const;

  int getNSubdivisions() const
  {
    return _nsubdivisions;
  }

private:
  bool getInnerCode(const cv::Mat& thres_img, int total_nbits, std::vector<uint64_t>& ids);
  cv::Mat rotate(const cv::Mat& in);
  uint64_t touulong(const cv::Mat& code);
  std::vector<Dictionary> vdic;
  void toMat(uint64_t code, int nbits_sq, cv::Mat& out);
  int _nsubdivisions = 0;
  float _max_correction_rate;
  std::string dicttypename;
  std::map<uint32_t, std::vector<Dictionary*>> nbits_dict;
};

} // namespace aruco

#endif /* ArucoDictionaryBasedMarkerDetector_H */
