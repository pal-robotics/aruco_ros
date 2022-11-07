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
#include "markerlabeler.h"
#include "dictionary_based.h"
namespace aruco
{
cv::Ptr<MarkerLabeler> MarkerLabeler::create(Dictionary::DICT_TYPES dict_type,
                                             float error_correction_rate)
{
  Dictionary dict = Dictionary::loadPredefined(dict_type);
  DictionaryBased* db = new DictionaryBased();
  db->setParams(dict, error_correction_rate);
  return db;
}
cv::Ptr<MarkerLabeler> MarkerLabeler::create(std::string detector, std::string params)
{
  auto _stof = [](std::string str)
  {
    float f;
    sscanf(str.c_str(), "%f", &f);
    return f;
  };
  (void)params;
  Dictionary dict = Dictionary::load(detector);
  // try with one from file
  DictionaryBased* db = new DictionaryBased();
  db->setParams(dict, _stof(params));
  return db;
}
}  // namespace aruco
