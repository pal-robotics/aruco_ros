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
  // first, dictionary, second the maximum correction rate [0,1]. If 0,no correction, if
  // 1, maximum allowed correction
  void setParams(const Dictionary& dic, float max_correction_rate);

  // main virtual class to o detection
  virtual bool detect(const cv::Mat& in, int& marker_id, int& nRotations,
                      std::string& additionalInfo);
  // returns the dictionary name
  virtual std::string getName() const;

  virtual int getNSubdivisions() const
  {
    return _nsubdivisions;
  }  //

  std::vector<Dictionary> getDictionaries() const
  {
    return vdic;
  }

private:
  // obfuscate start
  bool getInnerCode(const cv::Mat& thres_img, int total_nbits, std::vector<uint64_t>& ids);
  cv::Mat rotate(const cv::Mat& in);
  uint64_t touulong(const cv::Mat& code);
  std::vector<Dictionary> vdic;
  void toMat(uint64_t code, int nbits_sq, cv::Mat& out);
  int _nsubdivisions = 0;
  float _max_correction_rate;
  std::string dicttypename;
  std::map<uint32_t, std::vector<Dictionary*>> nbits_dict;
  // obfuscate end
};
}  // namespace aruco
#endif
