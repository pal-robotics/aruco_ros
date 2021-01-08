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

#include "dictionary.h"
#include <exception>
#include <stdint.h>
#include <fstream>
#include <stdexcept>
#include <vector>
#include <bitset>
#include <opencv2/imgproc.hpp>
#include "markermap.h"
#include <set>
#include <stdexcept>

namespace aruco
{

Dictionary Dictionary::load(std::string info)
{
  if (isPredefinedDictinaryString(info))
    return loadPredefined(info);
  else
    return loadFromFile(info);
}

Dictionary Dictionary::loadFromFile(std::string path)
{
  auto parse = [](const std::string &str)
  {
    std::stringstream sstr(str);
    std::string a, b;
    sstr >> a >> b;
    return b;
  };

  std::ifstream file(path);
  if (!file)
    throw std::runtime_error("Dictionary::loadFromFile could not open file");
  std::string line;
  std::getline(file, line);
  if (line.find("name") == std::string::npos)
    throw std::runtime_error("Dictionary::loadFromFile 'name' not found");
  Dictionary d;
  d._name = parse(line);
  std::getline(file, line);
  if (line.find("nbits") == std::string::npos)
    throw std::runtime_error("Dictionary::loadFromFile 'nbits' not found");
  d._nbits = stoi(parse(line));
  if (d._nbits > 64)
    throw std::runtime_error("Dictionary::loadFromFile no more than 64 bits allowed ");

  d._type = CUSTOM;

  // go reading the data
  while (!file.eof())
  {
    std::getline(file, line);
    if (line.size() == d._nbits)
    {
      std::bitset<64> marker;

      // parse the bits
      int idx = 0;
      for (auto it = line.rbegin(); it != line.rend(); it++)
      {
        marker[idx++] = *it == '1';
      }
      d._code_id.insert( {marker.to_ullong(), static_cast<std::uint16_t>(d._code_id.size())});
    }
  }
  d._tau = static_cast<std::uint32_t>(computeDictionaryDistance(d));
  if (d._tau == 0)
  {
    std::cerr << "IMPORTANT MESSAGE:::: Your dictionary " << d._name << " has a distance of 0" << std::endl;
    std::cerr << "It means that that there are markers that can be confused. Please check" << std::endl;
  }
//  cout << "tau = " << d._tau << endl;
  return d;
}

void Dictionary::fromVector(const std::vector<std::uint64_t> &codes,
                            std::map<std::uint64_t, std::uint16_t> &code_id_map)
{
  code_id_map.clear();
  int id = 0;
  for (auto c : codes)
    code_id_map.insert(std::make_pair(c, id++));
}

Dictionary Dictionary::loadPredefined(std::string type)
{
  return loadPredefined(getTypeFromString(type));
}

Dictionary Dictionary::loadPredefined(DICT_TYPES type)
{
  Dictionary d;
  switch (type)
  {
    case ARUCO:
    {
      std::vector<std::uint64_t> codes = {0x1084210UL, 0x1084217UL, 0x1084209UL, 0x108420eUL, 0x10842f0UL, 0x10842f7UL,
                                          0x10842e9UL, 0x10842eeUL, 0x1084130UL, 0x1084137UL, 0x1084129UL, 0x108412eUL,
                                          0x10841d0UL, 0x10841d7UL, 0x10841c9UL, 0x10841ceUL, 0x1085e10UL, 0x1085e17UL,
                                          0x1085e09UL, 0x1085e0eUL, 0x1085ef0UL, 0x1085ef7UL, 0x1085ee9UL, 0x1085eeeUL,
                                          0x1085d30UL, 0x1085d37UL, 0x1085d29UL, 0x1085d2eUL, 0x1085dd0UL, 0x1085dd7UL,
                                          0x1085dc9UL, 0x1085dceUL, 0x1082610UL, 0x1082617UL, 0x1082609UL, 0x108260eUL,
                                          0x10826f0UL, 0x10826f7UL, 0x10826e9UL, 0x10826eeUL, 0x1082530UL, 0x1082537UL,
                                          0x1082529UL, 0x108252eUL, 0x10825d0UL, 0x10825d7UL, 0x10825c9UL, 0x10825ceUL,
                                          0x1083a10UL, 0x1083a17UL, 0x1083a09UL, 0x1083a0eUL, 0x1083af0UL, 0x1083af7UL,
                                          0x1083ae9UL, 0x1083aeeUL, 0x1083930UL, 0x1083937UL, 0x1083929UL, 0x108392eUL,
                                          0x10839d0UL, 0x10839d7UL, 0x10839c9UL, 0x10839ceUL, 0x10bc210UL, 0x10bc217UL,
                                          0x10bc209UL, 0x10bc20eUL, 0x10bc2f0UL, 0x10bc2f7UL, 0x10bc2e9UL, 0x10bc2eeUL,
                                          0x10bc130UL, 0x10bc137UL, 0x10bc129UL, 0x10bc12eUL, 0x10bc1d0UL, 0x10bc1d7UL,
                                          0x10bc1c9UL, 0x10bc1ceUL, 0x10bde10UL, 0x10bde17UL, 0x10bde09UL, 0x10bde0eUL,
                                          0x10bdef0UL, 0x10bdef7UL, 0x10bdee9UL, 0x10bdeeeUL, 0x10bdd30UL, 0x10bdd37UL,
                                          0x10bdd29UL, 0x10bdd2eUL, 0x10bddd0UL, 0x10bddd7UL, 0x10bddc9UL, 0x10bddceUL,
                                          0x10ba610UL, 0x10ba617UL, 0x10ba609UL, 0x10ba60eUL, 0x10ba6f0UL, 0x10ba6f7UL,
                                          0x10ba6e9UL, 0x10ba6eeUL, 0x10ba530UL, 0x10ba537UL, 0x10ba529UL, 0x10ba52eUL,
                                          0x10ba5d0UL, 0x10ba5d7UL, 0x10ba5c9UL, 0x10ba5ceUL, 0x10bba10UL, 0x10bba17UL,
                                          0x10bba09UL, 0x10bba0eUL, 0x10bbaf0UL, 0x10bbaf7UL, 0x10bbae9UL, 0x10bbaeeUL,
                                          0x10bb930UL, 0x10bb937UL, 0x10bb929UL, 0x10bb92eUL, 0x10bb9d0UL, 0x10bb9d7UL,
                                          0x10bb9c9UL, 0x10bb9ceUL, 0x104c210UL, 0x104c217UL, 0x104c209UL, 0x104c20eUL,
                                          0x104c2f0UL, 0x104c2f7UL, 0x104c2e9UL, 0x104c2eeUL, 0x104c130UL, 0x104c137UL,
                                          0x104c129UL, 0x104c12eUL, 0x104c1d0UL, 0x104c1d7UL, 0x104c1c9UL, 0x104c1ceUL,
                                          0x104de10UL, 0x104de17UL, 0x104de09UL, 0x104de0eUL, 0x104def0UL, 0x104def7UL,
                                          0x104dee9UL, 0x104deeeUL, 0x104dd30UL, 0x104dd37UL, 0x104dd29UL, 0x104dd2eUL,
                                          0x104ddd0UL, 0x104ddd7UL, 0x104ddc9UL, 0x104ddceUL, 0x104a610UL, 0x104a617UL,
                                          0x104a609UL, 0x104a60eUL, 0x104a6f0UL, 0x104a6f7UL, 0x104a6e9UL, 0x104a6eeUL,
                                          0x104a530UL, 0x104a537UL, 0x104a529UL, 0x104a52eUL, 0x104a5d0UL, 0x104a5d7UL,
                                          0x104a5c9UL, 0x104a5ceUL, 0x104ba10UL, 0x104ba17UL, 0x104ba09UL, 0x104ba0eUL,
                                          0x104baf0UL, 0x104baf7UL, 0x104bae9UL, 0x104baeeUL, 0x104b930UL, 0x104b937UL,
                                          0x104b929UL, 0x104b92eUL, 0x104b9d0UL, 0x104b9d7UL, 0x104b9c9UL, 0x104b9ceUL,
                                          0x1074210UL, 0x1074217UL, 0x1074209UL, 0x107420eUL, 0x10742f0UL, 0x10742f7UL,
                                          0x10742e9UL, 0x10742eeUL, 0x1074130UL, 0x1074137UL, 0x1074129UL, 0x107412eUL,
                                          0x10741d0UL, 0x10741d7UL, 0x10741c9UL, 0x10741ceUL, 0x1075e10UL, 0x1075e17UL,
                                          0x1075e09UL, 0x1075e0eUL, 0x1075ef0UL, 0x1075ef7UL, 0x1075ee9UL, 0x1075eeeUL,
                                          0x1075d30UL, 0x1075d37UL, 0x1075d29UL, 0x1075d2eUL, 0x1075dd0UL, 0x1075dd7UL,
                                          0x1075dc9UL, 0x1075dceUL, 0x1072610UL, 0x1072617UL, 0x1072609UL, 0x107260eUL,
                                          0x10726f0UL, 0x10726f7UL, 0x10726e9UL, 0x10726eeUL, 0x1072530UL, 0x1072537UL,
                                          0x1072529UL, 0x107252eUL, 0x10725d0UL, 0x10725d7UL, 0x10725c9UL, 0x10725ceUL,
                                          0x1073a10UL, 0x1073a17UL, 0x1073a09UL, 0x1073a0eUL, 0x1073af0UL, 0x1073af7UL,
                                          0x1073ae9UL, 0x1073aeeUL, 0x1073930UL, 0x1073937UL, 0x1073929UL, 0x107392eUL,
                                          0x10739d0UL, 0x10739d7UL, 0x10739c9UL, 0x10739ceUL, 0x1784210UL, 0x1784217UL,
                                          0x1784209UL, 0x178420eUL, 0x17842f0UL, 0x17842f7UL, 0x17842e9UL, 0x17842eeUL,
                                          0x1784130UL, 0x1784137UL, 0x1784129UL, 0x178412eUL, 0x17841d0UL, 0x17841d7UL,
                                          0x17841c9UL, 0x17841ceUL, 0x1785e10UL, 0x1785e17UL, 0x1785e09UL, 0x1785e0eUL,
                                          0x1785ef0UL, 0x1785ef7UL, 0x1785ee9UL, 0x1785eeeUL, 0x1785d30UL, 0x1785d37UL,
                                          0x1785d29UL, 0x1785d2eUL, 0x1785dd0UL, 0x1785dd7UL, 0x1785dc9UL, 0x1785dceUL,
                                          0x1782610UL, 0x1782617UL, 0x1782609UL, 0x178260eUL, 0x17826f0UL, 0x17826f7UL,
                                          0x17826e9UL, 0x17826eeUL, 0x1782530UL, 0x1782537UL, 0x1782529UL, 0x178252eUL,
                                          0x17825d0UL, 0x17825d7UL, 0x17825c9UL, 0x17825ceUL, 0x1783a10UL, 0x1783a17UL,
                                          0x1783a09UL, 0x1783a0eUL, 0x1783af0UL, 0x1783af7UL, 0x1783ae9UL, 0x1783aeeUL,
                                          0x1783930UL, 0x1783937UL, 0x1783929UL, 0x178392eUL, 0x17839d0UL, 0x17839d7UL,
                                          0x17839c9UL, 0x17839ceUL, 0x17bc210UL, 0x17bc217UL, 0x17bc209UL, 0x17bc20eUL,
                                          0x17bc2f0UL, 0x17bc2f7UL, 0x17bc2e9UL, 0x17bc2eeUL, 0x17bc130UL, 0x17bc137UL,
                                          0x17bc129UL, 0x17bc12eUL, 0x17bc1d0UL, 0x17bc1d7UL, 0x17bc1c9UL, 0x17bc1ceUL,
                                          0x17bde10UL, 0x17bde17UL, 0x17bde09UL, 0x17bde0eUL, 0x17bdef0UL, 0x17bdef7UL,
                                          0x17bdee9UL, 0x17bdeeeUL, 0x17bdd30UL, 0x17bdd37UL, 0x17bdd29UL, 0x17bdd2eUL,
                                          0x17bddd0UL, 0x17bddd7UL, 0x17bddc9UL, 0x17bddceUL, 0x17ba610UL, 0x17ba617UL,
                                          0x17ba609UL, 0x17ba60eUL, 0x17ba6f0UL, 0x17ba6f7UL, 0x17ba6e9UL, 0x17ba6eeUL,
                                          0x17ba530UL, 0x17ba537UL, 0x17ba529UL, 0x17ba52eUL, 0x17ba5d0UL, 0x17ba5d7UL,
                                          0x17ba5c9UL, 0x17ba5ceUL, 0x17bba10UL, 0x17bba17UL, 0x17bba09UL, 0x17bba0eUL,
                                          0x17bbaf0UL, 0x17bbaf7UL, 0x17bbae9UL, 0x17bbaeeUL, 0x17bb930UL, 0x17bb937UL,
                                          0x17bb929UL, 0x17bb92eUL, 0x17bb9d0UL, 0x17bb9d7UL, 0x17bb9c9UL, 0x17bb9ceUL,
                                          0x174c210UL, 0x174c217UL, 0x174c209UL, 0x174c20eUL, 0x174c2f0UL, 0x174c2f7UL,
                                          0x174c2e9UL, 0x174c2eeUL, 0x174c130UL, 0x174c137UL, 0x174c129UL, 0x174c12eUL,
                                          0x174c1d0UL, 0x174c1d7UL, 0x174c1c9UL, 0x174c1ceUL, 0x174de10UL, 0x174de17UL,
                                          0x174de09UL, 0x174de0eUL, 0x174def0UL, 0x174def7UL, 0x174dee9UL, 0x174deeeUL,
                                          0x174dd30UL, 0x174dd37UL, 0x174dd29UL, 0x174dd2eUL, 0x174ddd0UL, 0x174ddd7UL,
                                          0x174ddc9UL, 0x174ddceUL, 0x174a610UL, 0x174a617UL, 0x174a609UL, 0x174a60eUL,
                                          0x174a6f0UL, 0x174a6f7UL, 0x174a6e9UL, 0x174a6eeUL, 0x174a530UL, 0x174a537UL,
                                          0x174a529UL, 0x174a52eUL, 0x174a5d0UL, 0x174a5d7UL, 0x174a5c9UL, 0x174a5ceUL,
                                          0x174ba10UL, 0x174ba17UL, 0x174ba09UL, 0x174ba0eUL, 0x174baf0UL, 0x174baf7UL,
                                          0x174bae9UL, 0x174baeeUL, 0x174b930UL, 0x174b937UL, 0x174b929UL, 0x174b92eUL,
                                          0x174b9d0UL, 0x174b9d7UL, 0x174b9c9UL, 0x174b9ceUL, 0x1774210UL, 0x1774217UL,
                                          0x1774209UL, 0x177420eUL, 0x17742f0UL, 0x17742f7UL, 0x17742e9UL, 0x17742eeUL,
                                          0x1774130UL, 0x1774137UL, 0x1774129UL, 0x177412eUL, 0x17741d0UL, 0x17741d7UL,
                                          0x17741c9UL, 0x17741ceUL, 0x1775e10UL, 0x1775e17UL, 0x1775e09UL, 0x1775e0eUL,
                                          0x1775ef0UL, 0x1775ef7UL, 0x1775ee9UL, 0x1775eeeUL, 0x1775d30UL, 0x1775d37UL,
                                          0x1775d29UL, 0x1775d2eUL, 0x1775dd0UL, 0x1775dd7UL, 0x1775dc9UL, 0x1775dceUL,
                                          0x1772610UL, 0x1772617UL, 0x1772609UL, 0x177260eUL, 0x17726f0UL, 0x17726f7UL,
                                          0x17726e9UL, 0x17726eeUL, 0x1772530UL, 0x1772537UL, 0x1772529UL, 0x177252eUL,
                                          0x17725d0UL, 0x17725d7UL, 0x17725c9UL, 0x17725ceUL, 0x1773a10UL, 0x1773a17UL,
                                          0x1773a09UL, 0x1773a0eUL, 0x1773af0UL, 0x1773af7UL, 0x1773ae9UL, 0x1773aeeUL,
                                          0x1773930UL, 0x1773937UL, 0x1773929UL, 0x177392eUL, 0x17739d0UL, 0x17739d7UL,
                                          0x17739c9UL, 0x17739ceUL, 0x984210UL, 0x984217UL, 0x984209UL, 0x98420eUL,
                                          0x9842f0UL, 0x9842f7UL, 0x9842e9UL, 0x9842eeUL, 0x984130UL, 0x984137UL,
                                          0x984129UL, 0x98412eUL, 0x9841d0UL, 0x9841d7UL, 0x9841c9UL, 0x9841ceUL,
                                          0x985e10UL, 0x985e17UL, 0x985e09UL, 0x985e0eUL, 0x985ef0UL, 0x985ef7UL,
                                          0x985ee9UL, 0x985eeeUL, 0x985d30UL, 0x985d37UL, 0x985d29UL, 0x985d2eUL,
                                          0x985dd0UL, 0x985dd7UL, 0x985dc9UL, 0x985dceUL, 0x982610UL, 0x982617UL,
                                          0x982609UL, 0x98260eUL, 0x9826f0UL, 0x9826f7UL, 0x9826e9UL, 0x9826eeUL,
                                          0x982530UL, 0x982537UL, 0x982529UL, 0x98252eUL, 0x9825d0UL, 0x9825d7UL,
                                          0x9825c9UL, 0x9825ceUL, 0x983a10UL, 0x983a17UL, 0x983a09UL, 0x983a0eUL,
                                          0x983af0UL, 0x983af7UL, 0x983ae9UL, 0x983aeeUL, 0x983930UL, 0x983937UL,
                                          0x983929UL, 0x98392eUL, 0x9839d0UL, 0x9839d7UL, 0x9839c9UL, 0x9839ceUL,
                                          0x9bc210UL, 0x9bc217UL, 0x9bc209UL, 0x9bc20eUL, 0x9bc2f0UL, 0x9bc2f7UL,
                                          0x9bc2e9UL, 0x9bc2eeUL, 0x9bc130UL, 0x9bc137UL, 0x9bc129UL, 0x9bc12eUL,
                                          0x9bc1d0UL, 0x9bc1d7UL, 0x9bc1c9UL, 0x9bc1ceUL, 0x9bde10UL, 0x9bde17UL,
                                          0x9bde09UL, 0x9bde0eUL, 0x9bdef0UL, 0x9bdef7UL, 0x9bdee9UL, 0x9bdeeeUL,
                                          0x9bdd30UL, 0x9bdd37UL, 0x9bdd29UL, 0x9bdd2eUL, 0x9bddd0UL, 0x9bddd7UL,
                                          0x9bddc9UL, 0x9bddceUL, 0x9ba610UL, 0x9ba617UL, 0x9ba609UL, 0x9ba60eUL,
                                          0x9ba6f0UL, 0x9ba6f7UL, 0x9ba6e9UL, 0x9ba6eeUL, 0x9ba530UL, 0x9ba537UL,
                                          0x9ba529UL, 0x9ba52eUL, 0x9ba5d0UL, 0x9ba5d7UL, 0x9ba5c9UL, 0x9ba5ceUL,
                                          0x9bba10UL, 0x9bba17UL, 0x9bba09UL, 0x9bba0eUL, 0x9bbaf0UL, 0x9bbaf7UL,
                                          0x9bbae9UL, 0x9bbaeeUL, 0x9bb930UL, 0x9bb937UL, 0x9bb929UL, 0x9bb92eUL,
                                          0x9bb9d0UL, 0x9bb9d7UL, 0x9bb9c9UL, 0x9bb9ceUL, 0x94c210UL, 0x94c217UL,
                                          0x94c209UL, 0x94c20eUL, 0x94c2f0UL, 0x94c2f7UL, 0x94c2e9UL, 0x94c2eeUL,
                                          0x94c130UL, 0x94c137UL, 0x94c129UL, 0x94c12eUL, 0x94c1d0UL, 0x94c1d7UL,
                                          0x94c1c9UL, 0x94c1ceUL, 0x94de10UL, 0x94de17UL, 0x94de09UL, 0x94de0eUL,
                                          0x94def0UL, 0x94def7UL, 0x94dee9UL, 0x94deeeUL, 0x94dd30UL, 0x94dd37UL,
                                          0x94dd29UL, 0x94dd2eUL, 0x94ddd0UL, 0x94ddd7UL, 0x94ddc9UL, 0x94ddceUL,
                                          0x94a610UL, 0x94a617UL, 0x94a609UL, 0x94a60eUL, 0x94a6f0UL, 0x94a6f7UL,
                                          0x94a6e9UL, 0x94a6eeUL, 0x94a530UL, 0x94a537UL, 0x94a529UL, 0x94a52eUL,
                                          0x94a5d0UL, 0x94a5d7UL, 0x94a5c9UL, 0x94a5ceUL, 0x94ba10UL, 0x94ba17UL,
                                          0x94ba09UL, 0x94ba0eUL, 0x94baf0UL, 0x94baf7UL, 0x94bae9UL, 0x94baeeUL,
                                          0x94b930UL, 0x94b937UL, 0x94b929UL, 0x94b92eUL, 0x94b9d0UL, 0x94b9d7UL,
                                          0x94b9c9UL, 0x94b9ceUL, 0x974210UL, 0x974217UL, 0x974209UL, 0x97420eUL,
                                          0x9742f0UL, 0x9742f7UL, 0x9742e9UL, 0x9742eeUL, 0x974130UL, 0x974137UL,
                                          0x974129UL, 0x97412eUL, 0x9741d0UL, 0x9741d7UL, 0x9741c9UL, 0x9741ceUL,
                                          0x975e10UL, 0x975e17UL, 0x975e09UL, 0x975e0eUL, 0x975ef0UL, 0x975ef7UL,
                                          0x975ee9UL, 0x975eeeUL, 0x975d30UL, 0x975d37UL, 0x975d29UL, 0x975d2eUL,
                                          0x975dd0UL, 0x975dd7UL, 0x975dc9UL, 0x975dceUL, 0x972610UL, 0x972617UL,
                                          0x972609UL, 0x97260eUL, 0x9726f0UL, 0x9726f7UL, 0x9726e9UL, 0x9726eeUL,
                                          0x972530UL, 0x972537UL, 0x972529UL, 0x97252eUL, 0x9725d0UL, 0x9725d7UL,
                                          0x9725c9UL, 0x9725ceUL, 0x973a10UL, 0x973a17UL, 0x973a09UL, 0x973a0eUL,
                                          0x973af0UL, 0x973af7UL, 0x973ae9UL, 0x973aeeUL, 0x973930UL, 0x973937UL,
                                          0x973929UL, 0x97392eUL, 0x9739d0UL, 0x9739d7UL, 0x9739c9UL, 0x9739ceUL,
                                          0xe84210UL, 0xe84217UL, 0xe84209UL, 0xe8420eUL, 0xe842f0UL, 0xe842f7UL,
                                          0xe842e9UL, 0xe842eeUL, 0xe84130UL, 0xe84137UL, 0xe84129UL, 0xe8412eUL,
                                          0xe841d0UL, 0xe841d7UL, 0xe841c9UL, 0xe841ceUL, 0xe85e10UL, 0xe85e17UL,
                                          0xe85e09UL, 0xe85e0eUL, 0xe85ef0UL, 0xe85ef7UL, 0xe85ee9UL, 0xe85eeeUL,
                                          0xe85d30UL, 0xe85d37UL, 0xe85d29UL, 0xe85d2eUL, 0xe85dd0UL, 0xe85dd7UL,
                                          0xe85dc9UL, 0xe85dceUL, 0xe82610UL, 0xe82617UL, 0xe82609UL, 0xe8260eUL,
                                          0xe826f0UL, 0xe826f7UL, 0xe826e9UL, 0xe826eeUL, 0xe82530UL, 0xe82537UL,
                                          0xe82529UL, 0xe8252eUL, 0xe825d0UL, 0xe825d7UL, 0xe825c9UL, 0xe825ceUL,
                                          0xe83a10UL, 0xe83a17UL, 0xe83a09UL, 0xe83a0eUL, 0xe83af0UL, 0xe83af7UL,
                                          0xe83ae9UL, 0xe83aeeUL, 0xe83930UL, 0xe83937UL, 0xe83929UL, 0xe8392eUL,
                                          0xe839d0UL, 0xe839d7UL, 0xe839c9UL, 0xe839ceUL, 0xebc210UL, 0xebc217UL,
                                          0xebc209UL, 0xebc20eUL, 0xebc2f0UL, 0xebc2f7UL, 0xebc2e9UL, 0xebc2eeUL,
                                          0xebc130UL, 0xebc137UL, 0xebc129UL, 0xebc12eUL, 0xebc1d0UL, 0xebc1d7UL,
                                          0xebc1c9UL, 0xebc1ceUL, 0xebde10UL, 0xebde17UL, 0xebde09UL, 0xebde0eUL,
                                          0xebdef0UL, 0xebdef7UL, 0xebdee9UL, 0xebdeeeUL, 0xebdd30UL, 0xebdd37UL,
                                          0xebdd29UL, 0xebdd2eUL, 0xebddd0UL, 0xebddd7UL, 0xebddc9UL, 0xebddceUL,
                                          0xeba610UL, 0xeba617UL, 0xeba609UL, 0xeba60eUL, 0xeba6f0UL, 0xeba6f7UL,
                                          0xeba6e9UL, 0xeba6eeUL, 0xeba530UL, 0xeba537UL, 0xeba529UL, 0xeba52eUL,
                                          0xeba5d0UL, 0xeba5d7UL, 0xeba5c9UL, 0xeba5ceUL, 0xebba10UL, 0xebba17UL,
                                          0xebba09UL, 0xebba0eUL, 0xebbaf0UL, 0xebbaf7UL, 0xebbae9UL, 0xebbaeeUL,
                                          0xebb930UL, 0xebb937UL, 0xebb929UL, 0xebb92eUL, 0xebb9d0UL, 0xebb9d7UL,
                                          0xebb9c9UL, 0xebb9ceUL, 0xe4c210UL, 0xe4c217UL, 0xe4c209UL, 0xe4c20eUL,
                                          0xe4c2f0UL, 0xe4c2f7UL, 0xe4c2e9UL, 0xe4c2eeUL, 0xe4c130UL, 0xe4c137UL,
                                          0xe4c129UL, 0xe4c12eUL, 0xe4c1d0UL, 0xe4c1d7UL, 0xe4c1c9UL, 0xe4c1ceUL,
                                          0xe4de10UL, 0xe4de17UL, 0xe4de09UL, 0xe4de0eUL, 0xe4def0UL, 0xe4def7UL,
                                          0xe4dee9UL, 0xe4deeeUL, 0xe4dd30UL, 0xe4dd37UL, 0xe4dd29UL, 0xe4dd2eUL,
                                          0xe4ddd0UL, 0xe4ddd7UL, 0xe4ddc9UL, 0xe4ddceUL, 0xe4a610UL, 0xe4a617UL,
                                          0xe4a609UL, 0xe4a60eUL, 0xe4a6f0UL, 0xe4a6f7UL, 0xe4a6e9UL, 0xe4a6eeUL,
                                          0xe4a530UL, 0xe4a537UL, 0xe4a529UL, 0xe4a52eUL, 0xe4a5d0UL, 0xe4a5d7UL,
                                          0xe4a5c9UL, 0xe4a5ceUL, 0xe4ba10UL, 0xe4ba17UL, 0xe4ba09UL, 0xe4ba0eUL,
                                          0xe4baf0UL, 0xe4baf7UL, 0xe4bae9UL, 0xe4baeeUL, 0xe4b930UL, 0xe4b937UL,
                                          0xe4b929UL, 0xe4b92eUL, 0xe4b9d0UL, 0xe4b9d7UL, 0xe4b9c9UL, 0xe4b9ceUL,
                                          0xe74210UL, 0xe74217UL, 0xe74209UL, 0xe7420eUL, 0xe742f0UL, 0xe742f7UL,
                                          0xe742e9UL, 0xe742eeUL, 0xe74130UL, 0xe74137UL, 0xe74129UL, 0xe7412eUL,
                                          0xe741d0UL, 0xe741d7UL, 0xe741c9UL, 0xe741ceUL, 0xe75e10UL, 0xe75e17UL,
                                          0xe75e09UL, 0xe75e0eUL, 0xe75ef0UL, 0xe75ef7UL, 0xe75ee9UL, 0xe75eeeUL,
                                          0xe75d30UL, 0xe75d37UL, 0xe75d29UL, 0xe75d2eUL, 0xe75dd0UL, 0xe75dd7UL,
                                          0xe75dc9UL, 0xe75dceUL, 0xe72610UL, 0xe72617UL, 0xe72609UL, 0xe7260eUL,
                                          0xe726f0UL, 0xe726f7UL, 0xe726e9UL, 0xe726eeUL, 0xe72530UL, 0xe72537UL,
                                          0xe72529UL, 0xe7252eUL, 0xe725d0UL, 0xe725d7UL, 0xe725c9UL, 0xe725ceUL,
                                          0xe73a10UL, 0xe73a17UL, 0xe73a09UL, 0xe73a0eUL, 0xe73af0UL, 0xe73af7UL,
                                          0xe73ae9UL, 0xe73aeeUL, 0xe73930UL, 0xe73937UL, 0xe73929UL, 0xe7392eUL,
                                          0xe739d0UL, 0xe739d7UL, 0xe739c9UL};
      fromVector(codes, d._code_id);
      d._nbits = 25;
      d._tau = 1; //
      d._type = ARUCO;
      d._name = "ARUCO";
    }
      break;
    case ARTAG:
    {
      std::vector<std::uint64_t> codes = {0xf89c68ea2UL, 0xf021c83fdUL, 0x9b2f835a2UL, 0xf8ffdb019UL, 0xf2d12b272UL,
                                          0xf0e6afe8bUL, 0xe19dee435UL, 0xdbe424132UL, 0xa9885a341UL, 0x3add1e6caUL,
                                          0x600aa0d15UL, 0xf9d5c0938UL, 0xf85b0f3d4UL, 0xf838bcd6fUL, 0xfa6c8bf2dUL,
                                          0xfb469060cUL, 0xfb25238b7UL, 0xff8d4dc33UL, 0xfc3406a26UL, 0xfc57b549dUL,
                                          0xfcf361750UL, 0xfd7daedbcUL, 0xf42d724b4UL, 0xf1ccb47aaUL, 0xe1fe5da8eUL,
                                          0xe2e3c2f56UL, 0xe280711edUL, 0xe224a5220UL, 0xe36d0d5baUL, 0xed2cf4e23UL,
                                          0xecc188a74UL, 0xcf7ea3892UL, 0xcca45b03cUL, 0xc4de9c015UL, 0xc0b1959e7UL,
                                          0xd027a870eUL, 0xd1a967de2UL, 0xd3fd50fa0UL, 0xd67f25205UL, 0xdcf5013a3UL,
                                          0xdea1361e1UL, 0xdb8797f89UL, 0xd8f9bb4eaUL, 0x98321c07aUL, 0x9dd3da364UL,
                                          0x9162c0972UL, 0x91c614abfUL, 0x81339aaedUL, 0x3f3cd85d4UL, 0x3907e6e64UL,
                                          0x2231e480aUL, 0x287ca74daUL, 0xee52d854UL, 0x3b94b615UL, 0x1ab8cdc82UL,
                                          0x4463c9014UL, 0x6588d50b0UL, 0xf912a744eUL, 0xf97114af5UL, 0xfb81f7b7aUL,
                                          0xfbe2445c1UL, 0xff4a2a145UL, 0xfea756512UL, 0xfe03826dfUL, 0xfc90d29ebUL,
                                          0xfdbac90caUL, 0xfdd97ae71UL, 0xf564da32eUL, 0xf5c00e0e3UL, 0xf679456f6UL,
                                          0xf6be22b80UL, 0xf6dd9153bUL, 0xf7f78ac1aUL, 0xf35fe489eUL, 0xf33c57625UL,
                                          0xf398835e8UL, 0xf3fb30b53UL, 0xf2b298cc9UL, 0xf2164cf04UL, 0xf275ff1bfUL,
                                          0xf10bd3adcUL, 0xf16860467UL, 0xe1393a7f8UL, 0xe07092062UL, 0xe0b7f5d14UL,
                                          0xe0d4463afUL, 0xe7c56313eUL, 0xe7a6d0f85UL, 0xe70204c48UL, 0xe6281f569UL,
                                          0xe47c2872bUL, 0xed8820deeUL, 0xedeb93355UL, 0xed4f47098UL, 0xec06ef702UL,
                                          0xeef60c68dUL, 0xef78c3c61UL, 0xefbfa4117UL, 0xebb31e65eUL, 0xebd0ad8e5UL,
                                          0xeb7479b28UL, 0xeb17ca593UL, 0xea5e62209UL, 0xeafab61c4UL, 0xe8ae81386UL,
                                          0xe9204e96aUL, 0xe9e72941cUL, 0xe9849aaa7UL, 0xc86f86a03UL, 0xc9e1490efUL,
                                          0xcbb57e2adUL, 0xcb7219fdbUL, 0xca58026faUL, 0xca9f65b8cUL, 0xcef06c27eUL,
                                          0xce54b81b3UL, 0xcf1d10629UL, 0xcd8e4091dUL, 0xcc633cd4aUL, 0xccc7e8e87UL,
                                          0xc7c3035cdUL, 0xc767d7600UL, 0xc62e7f19aUL, 0xc64dccf21UL, 0xc6e918cecUL,
                                          0xc68aab257UL, 0xc2861151eUL, 0xc24176868UL, 0xc15ce9db0UL, 0xc0d22675cUL,
                                          0xd0e0cfa78UL, 0xd0837c4c3UL, 0xd16e00094UL, 0xd2739f54cUL, 0xd2102cbf7UL,
                                          0xd6dbf11c8UL, 0xd79259652UL, 0xd7368d59fUL, 0xd562ba7ddUL, 0xd5c66e410UL,
                                          0xd5a5ddaabUL, 0xd48fc638aUL, 0xd4ec75d31UL, 0xdc96b2d18UL, 0xddbca9439UL,
                                          0xdf4c4a5b6UL, 0xde05e222cUL, 0xdaad8c6a8UL, 0xdace3f813UL, 0xd9b013370UL,
                                          0xd9d3a0dcbUL, 0xd85d6f727UL, 0x9aa14cf4eUL, 0x9ac2ff1f5UL, 0x9b4c30b19UL,
                                          0x9be8e48d4UL, 0x9b8b5766fUL, 0x99bcd3a96UL, 0x99df6042dUL, 0x96b482695UL,
                                          0x979e99fb4UL, 0x95caaedf6UL, 0x950dc9080UL, 0x956e7ae3bUL, 0x9427d29a1UL,
                                          0x908fbcd25UL, 0x9392238fdUL, 0x92b8381dcUL, 0x824db618eUL, 0x83c379b62UL,
                                          0x83041e614UL, 0x81f4fd79bUL, 0x80dee6ebaUL, 0x8019813ccUL, 0x853f20da4UL,
                                          0x87cfc3c2bUL, 0x8e5878855UL, 0x8fb504c02UL, 0x8b1d6a886UL, 0x8a93a526aUL,
                                          0xabdc6d103UL, 0xaa3111554UL, 0xaa52a2befUL, 0xa8a241a60UL, 0xa8c1f24dbUL,
                                          0xa92c8e08cUL, 0xad438797eUL, 0xa156490a5UL, 0xa0d886a49UL, 0xb04ebb4a0UL,
                                          0xb1c074e4cUL, 0xb616361abUL, 0xb73c2d88aUL, 0xbf2559618UL, 0xbd716e45aUL,
                                          0xb8347c489UL, 0xb890a8744UL, 0xba604b6cbUL, 0xbb29e3151UL, 0x3c42f4eb7UL,
                                          0x388929488UL, 0x3b94b6150UL, 0x33ee71179UL, 0x311e920f6UL, 0x30905da1aUL,
                                          0x343833e9eUL, 0x35b6fc472UL, 0x360fb7267UL, 0x26998ac8eUL, 0x271745662UL,
                                          0x20a2b473eUL, 0x20c107985UL, 0x2188afe1fUL, 0x23bf2b2e6UL, 0x229530bc7UL,
                                          0x2a2890698UL, 0x2ba65fc74UL, 0x281f14a61UL, 0x2fc956586UL, 0x2f6d8264bUL,
                                          0x2e80fe21cUL, 0xe869e6efUL, 0xc767d760UL, 0x8bda0d5fUL, 0xae997f1dUL,
                                          0xa2ef026bUL, 0xb04ebb4aUL, 0xb67585f1UL, 0x37e2cb63UL, 0x25437242UL,
                                          0x149a879aUL, 0x69fea87dUL, 0x71125291UL, 0x1609d7694UL, 0x143e53a6dUL,
                                          0x11bc267c8UL, 0x12c20acabUL, 0x132f768fcUL, 0x134cc5647UL, 0x1a1c19f4fUL,
                                          0x182b9d3b6UL, 0x1c27274ffUL, 0x5a10d96a9UL, 0x50f94e9b4UL, 0x509afd70fUL,
                                          0x57e86bb25UL, 0x56c270204UL, 0x477e565ccUL, 0x43115fc3eUL, 0x493fafe55UL,
                                          0x48b1604b9UL, 0x4a2230b8dUL, 0x6cd8099b8UL, 0x6d3575defUL, 0x6bc92cb29UL,
                                          0x687067d3cUL, 0x652c0137dUL, 0x757d5b0e2UL, 0x76a7a384cUL, 0x72ab19f05UL,
                                          0x73e2b189fUL, 0x709c9d3fcUL, 0x7885e9d6eUL, 0x7bfbc560dUL, 0x7e79b0ba8UL,
                                          0x7dc0fbdbdUL, 0x7d642fe70UL, 0x7c4e34751UL, 0xf9b673783UL, 0xfa0f38196UL,
                                          0xfac85fce0UL, 0xfaabec25bUL, 0xff2999ffeUL, 0xfec4e5ba9UL, 0xfd1e1d307UL,
                                          0xf50769d95UL, 0xf5a3bde58UL, 0xf489a6779UL, 0xf44ec1a0fUL, 0xf794392a1UL,
                                          0xf7535efd7UL, 0xf0427bd46UL, 0xf1af07911UL, 0xe15a89943UL, 0xe01321ed9UL,
                                          0xe24716c9bUL, 0xe30ebeb01UL, 0xe3aa6a8ccUL, 0xe3c9d9677UL, 0xe761b72f3UL,
                                          0xe64bacbd2UL, 0xe68ccb6a4UL, 0xe4d8fc4e6UL, 0xe4bb4fa5dUL, 0xe41f9b990UL,
                                          0xe55633e0aUL, 0xe535800b1UL, 0xe5915437cUL, 0xe5f2e7dc7UL, 0xec655c9b9UL,
                                          0xeca23b4cfUL, 0xee95bf836UL, 0xee52d8540UL, 0xee316bbfbUL, 0xef1b702daUL,
                                          0xefdc17facUL, 0xea3dd1cb2UL, 0xea9905f7fUL, 0xe869e6ef0UL, 0xe80a5504bUL,
                                          0xe943fd7d1UL, 0xc8cb529ceUL, 0xc8a8e1775UL, 0xc80c354b8UL, 0xc9459d322UL,
                                          0xc982fae54UL, 0xcbd6cdc16UL, 0xcb11aa160UL, 0xca3bb1841UL, 0xcafcd6537UL,
                                          0xce93dfcc5UL, 0xce370bf08UL, 0xcfb9c45e4UL, 0xcfda77b5fUL, 0xcdedf37a6UL,
                                          0xcd2a94ad0UL, 0xcd492746bUL, 0xcc008f3f1UL, 0xc4bd2feaeUL, 0xc419fbd63UL,
                                          0xc533e0442UL, 0xc55053af9UL, 0xc5f487934UL, 0xc5973478fUL, 0xc7a0b0b76UL,
                                          0xc704648bbUL, 0xc2e5a2ba5UL, 0xc308deff2UL, 0xc19b8e0c6UL, 0xc01541a2aUL,
                                          0xd0441b9b5UL, 0xd1cad4359UL, 0xd10db3e2fUL, 0xd33a372d6UL, 0xd35984c6dUL,
                                          0xd2b4f883aUL, 0xd2d74b681UL, 0xd50109966UL, 0xd448a1efcUL, 0xdc51d506eUL,
                                          0xdd187d7f4UL, 0xdd7bce94fUL, 0xdf2ff9b0dUL, 0xdf8b2d8c0UL, 0xdfe89e67bUL,
                                          0xdec285f5aUL, 0xde6651c97UL, 0xda6aebbdeUL, 0xda0958565UL, 0xdb2343c44UL,
                                          0xd97774e06UL, 0xd914c70bdUL, 0xd89a08a51UL, 0xd83edc99cUL, 0x9a662b238UL,
                                          0x9a0598c83UL, 0x997bb47e0UL, 0x99180795bUL, 0x9851afec1UL, 0x9896c83b7UL,
                                          0x9c9a724feUL, 0x9cf9c1a45UL, 0x9c5d15988UL, 0x9c3ea6733UL, 0x9d14bde12UL,
                                          0x9d770e0a9UL, 0x9db069ddfUL, 0x9f87ed126UL, 0x9fe45ef9dUL, 0x9f408ac50UL,
                                          0x9f23392ebUL, 0x9e6a91571UL, 0x9ece456bcUL, 0x96d73182eUL, 0x961056558UL,
                                          0x9673e5be3UL, 0x9759fe2c2UL, 0x973a4dc79UL, 0x95a91d34dUL, 0x94446171aUL,
                                          0x94e0b54d7UL, 0x90ec0f39eUL, 0x902b68ee8UL, 0x9048db053UL, 0x9101737c9UL,
                                          0x93f190646UL, 0x93554458bUL, 0x921cec211UL, 0x82e962243UL, 0x83a0ca5d9UL,
                                          0x8367ad8afUL, 0x815029456UL, 0x81974e920UL, 0x80bd55001UL, 0x807a32d77UL,
                                          0x847688a3eUL, 0x84153b485UL, 0x84b1ef748UL, 0x84d25c9f3UL, 0x85f8470d2UL,
                                          0x855c9331fUL, 0x876b17fe6UL, 0x87ac70290UL, 0x86e5d850aUL, 0x8622bf87cUL,
                                          0x8e3bcb6eeUL, 0x8efcacb98UL, 0x8f7263174UL, 0x8f11d0fcfUL, 0x8d2654336UL,
                                          0x8d45e7d8dUL, 0x8de133e40UL, 0x8d82800fbUL, 0x8ca89b9daUL, 0x8c6ffc4acUL,
                                          0x8c0c4fa17UL, 0x8800f5d5eUL, 0x8863463e5UL, 0x88c792028UL, 0x88a421e93UL,
                                          0x898e3a7b2UL, 0x89ed89909UL, 0x89495dac4UL, 0x892aee47fUL, 0x8bda0d5f0UL,
                                          0x8bb9beb4bUL, 0x8af016cd1UL, 0x8a54c2f1cUL, 0x8a37711a7UL, 0xab1b0ac75UL,
                                          0xaa95c5699UL, 0xa86526716UL, 0xa806959adUL, 0xa9ebe9dfaUL, 0xa94f3de37UL,
                                          0xad20347c5UL, 0xade753ab3UL, 0xaccd48392UL, 0xacaefbd29UL, 0xac0a2fee4UL,
                                          0xae5e18ca6UL, 0xae3dab21dUL, 0xae997f1d0UL, 0xafd0d764aUL, 0xafb3648f1UL,
                                          0xa70ec45aeUL, 0xa76d77b15UL, 0xa7c9a38d8UL, 0xa7aa10663UL, 0xa6800bf42UL,
                                          0xa6e3b81f9UL, 0xa624dfc8fUL, 0xa4135b076UL, 0xa4d43cd00UL, 0xa59d94a9aUL,
                                          0xa5fe27421UL, 0xa55af37ecUL, 0xa135fae1eUL, 0xa1f29d368UL, 0xa1912edd3UL,
                                          0xa0bb354f2UL, 0xa24bd657dUL, 0xa2ef026b0UL, 0xa28cb180bUL, 0xa3a6aa12aUL,
                                          0xa361cdc5cUL, 0xa3027e2e7UL, 0xb39443c0eUL, 0xb3f7f02b5UL, 0xb35324178UL,
                                          0xb33097fc3UL, 0xb2793f859UL, 0xb2ddebb94UL, 0xb089dc9d6UL, 0xb0ea6f76dUL,
                                          0xb02d08a1bUL, 0xb1071333aUL, 0xb164a0d81UL, 0xb5af7d7beUL, 0xb5ccce905UL,
                                          0xb5681aac8UL, 0xb50ba9473UL, 0xb421b2d52UL, 0xb442013e9UL, 0xb4e6d5024UL,
                                          0xb48566e9fUL, 0xb6b2e2266UL, 0xb6d151cddUL, 0xb67585f10UL, 0xb75f9e631UL,
                                          0xb7fb4a5fcUL, 0xbfe23eb6eUL, 0xbf46ea8a3UL, 0xbe6cf1182UL, 0xbeab96cf4UL,
                                          0xbec82524fUL, 0xbc9c1200dUL, 0xbc38c63c0UL, 0xbc5b75d7bUL, 0xbdb60992cUL,
                                          0xbdd5ba797UL, 0xb9d9000deUL, 0xb9bab3e65UL, 0xb91e67da8UL, 0xb857cfa32UL,
                                          0xb8f31b9ffUL, 0xbac49f506UL, 0xbaa72cbbdUL, 0xba03f8870UL, 0xbb4a50feaUL,
                                          0xbb8d3729cUL, 0xbbee84c27UL, 0x3e757024eUL, 0x3e16c3cf5UL, 0x3eb217f38UL,
                                          0x3ed1a4183UL, 0x3ffbbf8a2UL, 0x3f980c619UL, 0x3f5f6bb6fUL, 0x3d0b5c92dUL,
                                          0x3dcc3b45bUL, 0x3ce620d7aUL, 0x3c85933c1UL, 0x3c214700cUL, 0x384e4e9feUL,
                                          0x38ea9aa33UL, 0x39c081312UL, 0x39a332da9UL, 0x3964550dfUL, 0x3b53d1c26UL,
                                          0x3b306229dUL, 0x3bf705febUL, 0x3a1a79bbcUL, 0x3a79ca507UL, 0x32030d52eUL,
                                          0x3260beb95UL, 0x32c46a858UL, 0x32a7d96e3UL, 0x338dc2fc2UL, 0x334aa52b4UL,
                                          0x332916c0fUL, 0x317d21e4dUL, 0x31d9f5d80UL, 0x31ba4633bUL, 0x30f3ee4a1UL,
                                          0x30573a76cUL, 0x3034899d7UL, 0x345b80025UL, 0x34ff543e8UL, 0x349ce7d53UL,
                                          0x35d54fac9UL, 0x3512287bfUL, 0x37461f5fdUL, 0x37e2cb630UL, 0x37817888bUL,
                                          0x36ab631aaUL, 0x36c8d0f11UL, 0x366c04cdcUL, 0x265eed1f8UL, 0x2774f68d9UL,
                                          0x27b3915afUL, 0x258415956UL, 0x25e7a67edUL, 0x254372420UL, 0x2520c1a9bUL,
                                          0x240ada3baUL, 0x246969d01UL, 0x24cdbdeccUL, 0x24ae0e077UL, 0x2006604f3UL,
                                          0x23dc98c5dUL, 0x23784cf90UL, 0x231bff12bUL, 0x2252576b1UL, 0x22f68357cUL,
                                          0x2aeff7beeUL, 0x2a8c44555UL, 0x2a4b23823UL, 0x2b6138102UL, 0x29f268e36UL,
                                          0x29350f340UL, 0x2956bcdfbUL, 0x28d873717UL, 0x2cd4c905eUL, 0x2cb77aee5UL,
                                          0x2c13aed28UL, 0x2d5a06ab2UL, 0x2faae5b3dUL, 0x2f0e318f0UL, 0x2e4799f6aUL,
                                          0x2ee34dca7UL, 0xfac85fceUL, 0xf6be22b8UL, 0xe224a522UL, 0xe41f9b99UL,
                                          0xcd2a94adUL, 0xd3fd50faUL, 0x9f408ac5UL, 0x950dc908UL, 0x9336f7b3UL,
                                          0x81974e92UL, 0x87ac7029UL, 0x8de133e4UL, 0xa4d43cd0UL, 0xbc38c63cUL,
                                          0xba03f887UL, 0x3daf88aeUL, 0x31d9f5d8UL, 0x29350f34UL, 0xc767d76UL,
                                          0xa4d43cdUL, 0x63b3ebbUL, 0x12a1b921UL, 0x1ed7c457UL, 0x526a1e68UL,
                                          0x545120d3UL, 0x40cba749UL, 0x4a86e484UL, 0x6fc596c6UL, 0x63b3ebb0UL,
                                          0x77296c2aUL, 0x7b5f115cUL, 0x7d642fe7UL, 0x17407f10eUL, 0x1723ccfb5UL,
                                          0x17e4ab2c3UL, 0x16ceb0be2UL, 0x16ad03559UL, 0x14f93471bUL, 0x15d32fe3aUL,
                                          0x15b09c081UL, 0x15144834cUL, 0x1577fbdf7UL, 0x117b41abeUL, 0x1118f2405UL,
                                          0x11df95973UL, 0x10963dee9UL, 0x1032e9d24UL, 0x1266def66UL, 0x12056d1ddUL,
                                          0x12a1b9210UL, 0x13e81158aUL, 0x1b55b18d5UL, 0x1bf165b18UL, 0x1b92d65a3UL,
                                          0x1adb7e239UL, 0x1a7faa1f4UL, 0x18482ed0dUL, 0x18ecfaec0UL, 0x188f4907bUL,
                                          0x19a55295aUL, 0x190186a97UL, 0x1dca5b0a8UL, 0x1da9e8e13UL, 0x1c83f3732UL,
                                          0x1ce040989UL, 0x1c4494a44UL, 0x1e10a3806UL, 0x1ed7c4570UL, 0x1eb477bcbUL,
                                          0x1f9e6c2eaUL, 0x1ffddfc51UL, 0x1f590bf9cUL, 0x1f3ab8127UL, 0x5da5287f5UL,
                                          0x5d624fa83UL, 0x5c2be7d19UL, 0x5edb04c96UL, 0x5e1c631e0UL, 0x5e7fd0f5bUL,
                                          0x5f92acb0cUL, 0x5ff11f5b7UL, 0x5b9e16c45UL, 0x5b5971133UL, 0x5a736a812UL,
                                          0x5ab40d564UL, 0x58e03a726UL, 0x58838999dUL, 0x596ef5dcaUL, 0x59a9920bcUL,
                                          0x59ca21e07UL, 0x51b0e6e2eUL, 0x51d355095UL, 0x511432de3UL, 0x503e294c2UL,
                                          0x52ceca54dUL, 0x526a1e680UL, 0x5209ad83bUL, 0x5323b611aUL, 0x53e4d1c6cUL,
                                          0x5387622d7UL, 0x574cbf8e8UL, 0x572f0c653UL, 0x560517f72UL, 0x56a1c3cbfUL,
                                          0x549647046UL, 0x54f5f4efdUL, 0x545120d30UL, 0x557b3b411UL, 0x55bc5c967UL,
                                          0x45ed06af8UL, 0x458eb5443UL, 0x44a4aed62UL, 0x44007aeafUL, 0x4637fe256UL,
                                          0x46f099f20UL, 0x47da82601UL, 0x4372ec285UL, 0x429f906d2UL, 0x4258f7ba4UL,
                                          0x423b4451fUL, 0x40cba7490UL, 0x40a814a2bUL, 0x41820f30aUL, 0x41e1bcdb1UL,
                                          0x414568e7cUL, 0x495c1c0eeUL, 0x499b7bd98UL, 0x49f8c8323UL, 0x4815b4774UL,
                                          0x4a4183536UL, 0x4a86e4840UL, 0x4ae5576fbUL, 0x4bcf4cfdaUL, 0x4bacff161UL,
                                          0x4b082b2acUL, 0x4b6b98c17UL, 0x4f04915e5UL, 0x4fc3f6893UL, 0x4ee9ed1b2UL,
                                          0x4e8a5ef09UL, 0x4e2e8acc4UL, 0x4e4d3927fUL, 0x4c7abde86UL, 0x4c190e03dUL,
                                          0x4cbdda3f0UL, 0x4cde69d4bUL, 0x4df47246aUL, 0x4d97c1ad1UL, 0x4d331591cUL,
                                          0x6c1f6e4ceUL, 0x6c7cdda75UL, 0x6cbbba703UL, 0x6d91a1e22UL, 0x6df212099UL,
                                          0x6d56c6354UL, 0x6f6142fadUL, 0x6fc596c60UL, 0x6e8c3ebfaUL, 0x6eef8d541UL,
                                          0x6e28ea837UL, 0x6a2450f7eUL, 0x6a47e31c5UL, 0x6ae337208UL, 0x6baa9f592UL,
                                          0x6b6df88e4UL, 0x6b0e4b65fUL, 0x6939cfaa6UL, 0x69fea87d0UL, 0x699d1b96bUL,
                                          0x68b70004aUL, 0x68d4b3ef1UL, 0x6813d4387UL, 0x6069133aeUL, 0x60ae74ed8UL,
                                          0x60cdc7063UL, 0x61846f7f9UL, 0x6120bb434UL, 0x614308a8fUL, 0x63748c676UL,
                                          0x63173f8cdUL, 0x63d0585bbUL, 0x62fa43c9aUL, 0x6299f0221UL, 0x66522d81eUL,
                                          0x66319e6a5UL, 0x66954a568UL, 0x66f6f9bd3UL, 0x67bf51c49UL, 0x671b85f84UL,
                                          0x67783613fUL, 0x654fb2dc6UL, 0x64c17d72aUL, 0x64a2ce991UL, 0x64061aa5cUL,
                                          0x6465a94e7UL, 0x74f394a0eUL, 0x7490274b5UL, 0x7434f3778UL, 0x77ee0bfd6UL,
                                          0x778db816dUL, 0x774adfc1bUL, 0x7660c453aUL, 0x760377b81UL, 0x720fcdcc8UL,
                                          0x726c7e273UL, 0x734665b52UL, 0x7325d65e9UL, 0x738102624UL, 0x71d535466UL,
                                          0x711252910UL, 0x7171e17abUL, 0x705bfae8aUL, 0x70ff2ed47UL, 0x78213dea3UL,
                                          0x790b26782UL, 0x79cc41af4UL, 0x7b98768b6UL, 0x7b5f115c0UL, 0x7b3ca2b7bUL,
                                          0x7a16b925aUL, 0x7a750ace1UL, 0x7ad1def2cUL, 0x7ab26d197UL, 0x7ebed76deUL,
                                          0x7edd64865UL, 0x7e1a03513UL, 0x7f3018c32UL, 0x7f53ab289UL, 0x7ff77f144UL,
                                          0x7f94ccfffUL, 0x7da348306UL, 0x7d079c0cbUL, 0x7c8953a27UL, 0xffeefe288UL,
                                          0xfe6031864UL, 0xf4ea159c2UL, 0xf61af684dUL, 0xf730ed16cUL, 0xf0851c030UL,
                                          0xe6ef7881fUL, 0xe8cd32d3dUL, 0xc9262ed99UL, 0xc47a483d8UL, 0xc222c56d3UL,
                                          0xc36b6d149UL, 0xc3ac0ac3fUL, 0xc1f83de7dUL, 0xc13f5a30bUL, 0xc076f2491UL,
                                          0xd39ee311bUL, 0xd61c96cbeUL, 0xd6b842f73UL, 0xd7f1ea8e9UL, 0xd7553eb24UL,
                                          0xd42b12047UL, 0xdc3266ed5UL, 0xdddf1aa82UL, 0x9e0922bcaUL, 0x9eadf6807UL,
                                          0x97fd2a10fUL, 0x948306a6cUL, 0x91a5a7404UL, 0x9336f7b30UL, 0x927f5fcaaUL,
                                          0x92db8bf67UL, 0x822e05f35UL, 0x828ad1cf8UL, 0x859bf4e69UL, 0x8708a415dUL,
                                          0x86866bbb1UL, 0x86410c6c7UL, 0x8e9f1f523UL, 0x8fd6b72b9UL, 0x8ccb28761UL,
                                          0xabbfdefb8UL, 0xaaf676822UL, 0xad84e0408UL, 0xac699c05fUL, 0xaefaccf6bUL,
                                          0xaf17b0b3cUL, 0xaf7403587UL, 0xa6476c234UL, 0xa470e8ecdUL, 0xa4b78f3bbUL,
                                          0xa53940957UL, 0xa07c52984UL, 0xa01fe173fUL, 0xa22865bc6UL, 0xa3c519f91UL,
                                          0xb21a8c6e2UL, 0xb2be5852fUL, 0xb1a3c70f7UL, 0xb798f9b47UL, 0xbf818d5d5UL,
                                          0xbe0f42f39UL, 0xbcffa1eb6UL, 0xbd12ddae1UL, 0xb97dd4313UL, 0x3d68ef796UL,
                                          0x3daf88ae0UL, 0x382dfd745UL, 0x3abead871UL, 0x35719b904UL, 0x3725acb46UL,
                                          0x26fa39235UL, 0x263d5ef43UL, 0x27d022b14UL, 0x2065d3a48UL, 0x212c7bdd2UL,
                                          0x214fc8369UL, 0x21eb1c0a4UL, 0x2b028bfb9UL, 0x2bc5ec2cfUL, 0x2991db08dUL,
                                          0x28bbc09acUL, 0x2c701d393UL, 0x2d39b5409UL, 0x2d9d617c4UL, 0x2dfed297fUL,
                                          0x2e242a1d1UL, 0xfcf36175UL, 0xc15ce9dbUL, 0xd5c66e41UL, 0xdf8b2d8cUL,
                                          0x997bb47eUL, 0xa8a241a6UL, 0x23784cf9UL, 0x2f0e318fUL, 0x5e1c631eUL,
                                          0x58275da5UL, 0x4cbdda3fUL, 0x6588d50bUL, 0x178718c78UL, 0x166a6482fUL,
                                          0x145de04d6UL, 0x149a879a0UL, 0x10f58e052UL, 0x10515a39fUL, 0x138ba2b31UL,
                                          0x19c6e17e1UL, 0x19623542cUL, 0x1d0d3cddeUL, 0x1d6e8f365UL, 0x1e73106bdUL,
                                          0x5dc69b94eUL, 0x5d01fc438UL, 0x5c48543a2UL, 0x5c8f33ed4UL, 0x5cec8006fUL,
                                          0x5eb8b722dUL, 0x5f55cb67aUL, 0x5f36788c1UL, 0x5bfda52feUL, 0x5b3ac2f88UL,
                                          0x58275da50UL, 0x5844ee4ebUL, 0x590d46371UL, 0x517781358UL, 0x505d9aa79UL,
                                          0x52ad79bf6UL, 0x534005fa1UL, 0x578bd859eUL, 0x5666a41c9UL, 0x54329338bUL,
                                          0x55dfef7dcUL, 0x452a6178eUL, 0x4549d2935UL, 0x44c71d3d9UL, 0x46544dcedUL,
                                          0x46932a19bUL, 0x47b9318baUL, 0x471de5b77UL, 0x43d638148UL, 0x43b58bff3UL,
                                          0x42fc23869UL, 0x400cc09e6UL, 0x406f7375dUL, 0x4126db0c7UL, 0x48d2d3a02UL,
                                          0x4876079cfUL, 0x4f6722b5eUL, 0x4fa045628UL, 0x4d50a67a7UL, 0x6f02f1116UL,
                                          0x6fa6252dbUL, 0x6e4b5968cUL, 0x6a8084cb3UL, 0x695a7c41dUL, 0x61e7dc942UL,
                                          0x623d241ecUL, 0x625e97f57UL, 0x67dce22f2UL, 0x65eb66e0bUL, 0x7457409c3UL,
                                          0x751ee8e59UL, 0x75d98f32fUL, 0x77296c2a0UL, 0x76c4106f7UL, 0x72c8aa1beUL,
                                          0x71b686addUL, 0x703849031UL, 0x78e65a3d5UL, 0x78428e018UL, 0x796895939UL,
                                          0x79aff244fUL, 0x7c2d879eaUL, 0x7ceae049cUL, 0xc3cfb9284UL, 0xdb40f02ffUL,
                                          0x8b7ed963dUL, 0xab78b92ceUL, 0xf0851c03UL, 0xcb11aa16UL, 0xd9b01337UL,
                                          0x18ecfaecUL, 0x46f099f2UL, 0x1b360266eUL, 0x5ad7bebdfUL, 0x551888aaaUL,
                                          0x63b3ebb00UL, 0x75ba3cd94UL, 0x98f57bd0cUL, 0xf912a744eUL};
      fromVector(codes, d._code_id);
      d._nbits = 36;
      d._tau = 0;
      d._type = ARTAG;
      d._name = "ARTAG";
    }
      break;
    case ARTOOLKITPLUS:
    {
      std::vector<std::uint64_t> codes = {0x6dc269c27UL, 0x6d4229e26UL, 0x6cc2e9825UL, 0x6c42a9a24UL, 0x6fc369423UL,
                                          0x6f4329622UL, 0x6ec3e9021UL, 0x6e43a9220UL, 0x69c068c2fUL, 0x694028e2eUL,
                                          0x68c0e882dUL, 0x6840a8a2cUL, 0x6bc16842bUL, 0x6b412862aUL, 0x6ac1e8029UL,
                                          0x6a41a8228UL, 0x65c66bc37UL, 0x65462be36UL, 0x64c6eb835UL, 0x6446aba34UL,
                                          0x67c76b433UL, 0x67472b632UL, 0x66c7eb031UL, 0x6647ab230UL, 0x61c46ac3fUL,
                                          0x61442ae3eUL, 0x60c4ea83dUL, 0x6044aaa3cUL, 0x63c56a43bUL, 0x63452a63aUL,
                                          0x62c5ea039UL, 0x6245aa238UL, 0x7dca6dc07UL, 0x7d4a2de06UL, 0x7ccaed805UL,
                                          0x7c4aada04UL, 0x7fcb6d403UL, 0x7f4b2d602UL, 0x7ecbed001UL, 0x7e4bad200UL,
                                          0x79c86cc0fUL, 0x79482ce0eUL, 0x78c8ec80dUL, 0x7848aca0cUL, 0x7bc96c40bUL,
                                          0x7b492c60aUL, 0x7ac9ec009UL, 0x7a49ac208UL, 0x75ce6fc17UL, 0x754e2fe16UL,
                                          0x74ceef815UL, 0x744eafa14UL, 0x77cf6f413UL, 0x774f2f612UL, 0x76cfef011UL,
                                          0x764faf210UL, 0x71cc6ec1fUL, 0x714c2ee1eUL, 0x70ccee81dUL, 0x704caea1cUL,
                                          0x73cd6e41bUL, 0x734d2e61aUL, 0x72cdee019UL, 0x724dae218UL, 0x4dd261c67UL,
                                          0x4d5221e66UL, 0x4cd2e1865UL, 0x4c52a1a64UL, 0x4fd361463UL, 0x4f5321662UL,
                                          0x4ed3e1061UL, 0x4e53a1260UL, 0x49d060c6fUL, 0x495020e6eUL, 0x48d0e086dUL,
                                          0x4850a0a6cUL, 0x4bd16046bUL, 0x4b512066aUL, 0x4ad1e0069UL, 0x4a51a0268UL,
                                          0x45d663c77UL, 0x455623e76UL, 0x44d6e3875UL, 0x4456a3a74UL, 0x47d763473UL,
                                          0x475723672UL, 0x46d7e3071UL, 0x4657a3270UL, 0x41d462c7fUL, 0x415422e7eUL,
                                          0x40d4e287dUL, 0x4054a2a7cUL, 0x43d56247bUL, 0x43552267aUL, 0x42d5e2079UL,
                                          0x4255a2278UL, 0x5dda65c47UL, 0x5d5a25e46UL, 0x5cdae5845UL, 0x5c5aa5a44UL,
                                          0x5fdb65443UL, 0x5f5b25642UL, 0x5edbe5041UL, 0x5e5ba5240UL, 0x59d864c4fUL,
                                          0x595824e4eUL, 0x58d8e484dUL, 0x5858a4a4cUL, 0x5bd96444bUL, 0x5b592464aUL,
                                          0x5ad9e4049UL, 0x5a59a4248UL, 0x55de67c57UL, 0x555e27e56UL, 0x54dee7855UL,
                                          0x545ea7a54UL, 0x57df67453UL, 0x575f27652UL, 0x56dfe7051UL, 0x565fa7250UL,
                                          0x51dc66c5fUL, 0x515c26e5eUL, 0x50dce685dUL, 0x505ca6a5cUL, 0x53dd6645bUL,
                                          0x535d2665aUL, 0x52dde6059UL, 0x525da6258UL, 0x2de279ca7UL, 0x2d6239ea6UL,
                                          0x2ce2f98a5UL, 0x2c62b9aa4UL, 0x2fe3794a3UL, 0x2f63396a2UL, 0x2ee3f90a1UL,
                                          0x2e63b92a0UL, 0x29e078cafUL, 0x296038eaeUL, 0x28e0f88adUL, 0x2860b8aacUL,
                                          0x2be1784abUL, 0x2b61386aaUL, 0x2ae1f80a9UL, 0x2a61b82a8UL, 0x25e67bcb7UL,
                                          0x25663beb6UL, 0x24e6fb8b5UL, 0x2466bbab4UL, 0x27e77b4b3UL, 0x27673b6b2UL,
                                          0x26e7fb0b1UL, 0x2667bb2b0UL, 0x21e47acbfUL, 0x21643aebeUL, 0x20e4fa8bdUL,
                                          0x2064baabcUL, 0x23e57a4bbUL, 0x23653a6baUL, 0x22e5fa0b9UL, 0x2265ba2b8UL,
                                          0x3dea7dc87UL, 0x3d6a3de86UL, 0x3ceafd885UL, 0x3c6abda84UL, 0x3feb7d483UL,
                                          0x3f6b3d682UL, 0x3eebfd081UL, 0x3e6bbd280UL, 0x39e87cc8fUL, 0x39683ce8eUL,
                                          0x38e8fc88dUL, 0x3868bca8cUL, 0x3be97c48bUL, 0x3b693c68aUL, 0x3ae9fc089UL,
                                          0x3a69bc288UL, 0x35ee7fc97UL, 0x356e3fe96UL, 0x34eeff895UL, 0x346ebfa94UL,
                                          0x37ef7f493UL, 0x376f3f692UL, 0x36efff091UL, 0x366fbf290UL, 0x31ec7ec9fUL,
                                          0x316c3ee9eUL, 0x30ecfe89dUL, 0x306cbea9cUL, 0x33ed7e49bUL, 0x336d3e69aUL,
                                          0x32edfe099UL, 0x326dbe298UL, 0xdf271ce7UL, 0xd7231ee6UL, 0xcf2f18e5UL,
                                          0xc72b1ae4UL, 0xff3714e3UL, 0xf73316e2UL, 0xef3f10e1UL, 0xe73b12e0UL,
                                          0x9f070cefUL, 0x97030eeeUL, 0x8f0f08edUL, 0x870b0aecUL, 0xbf1704ebUL,
                                          0xb71306eaUL, 0xaf1f00e9UL, 0xa71b02e8UL, 0x5f673cf7UL, 0x57633ef6UL,
                                          0x4f6f38f5UL, 0x476b3af4UL, 0x7f7734f3UL, 0x777336f2UL, 0x6f7f30f1UL,
                                          0x677b32f0UL, 0x1f472cffUL, 0x17432efeUL, 0xf4f28fdUL, 0x74b2afcUL,
                                          0x3f5724fbUL, 0x375326faUL, 0x2f5f20f9UL, 0x275b22f8UL, 0x1dfa75cc7UL,
                                          0x1d7a35ec6UL, 0x1cfaf58c5UL, 0x1c7ab5ac4UL, 0x1ffb754c3UL, 0x1f7b356c2UL,
                                          0x1efbf50c1UL, 0x1e7bb52c0UL, 0x19f874ccfUL, 0x197834eceUL, 0x18f8f48cdUL,
                                          0x1878b4accUL, 0x1bf9744cbUL, 0x1b79346caUL, 0x1af9f40c9UL, 0x1a79b42c8UL,
                                          0x15fe77cd7UL, 0x157e37ed6UL, 0x14fef78d5UL, 0x147eb7ad4UL, 0x17ff774d3UL,
                                          0x177f376d2UL, 0x16fff70d1UL, 0x167fb72d0UL, 0x11fc76cdfUL, 0x117c36edeUL,
                                          0x10fcf68ddUL, 0x107cb6adcUL, 0x13fd764dbUL, 0x137d366daUL, 0x12fdf60d9UL,
                                          0x127db62d8UL, 0xed8249d27UL, 0xed0209f26UL, 0xec82c9925UL, 0xec0289b24UL,
                                          0xef8349523UL, 0xef0309722UL, 0xee83c9121UL, 0xee0389320UL, 0xe98048d2fUL,
                                          0xe90008f2eUL, 0xe880c892dUL, 0xe80088b2cUL, 0xeb814852bUL, 0xeb010872aUL,
                                          0xea81c8129UL, 0xea0188328UL, 0xe5864bd37UL, 0xe5060bf36UL, 0xe486cb935UL,
                                          0xe4068bb34UL, 0xe7874b533UL, 0xe7070b732UL, 0xe687cb131UL, 0xe6078b330UL,
                                          0xe1844ad3fUL, 0xe1040af3eUL, 0xe084ca93dUL, 0xe0048ab3cUL, 0xe3854a53bUL,
                                          0xe3050a73aUL, 0xe285ca139UL, 0xe2058a338UL, 0xfd8a4dd07UL, 0xfd0a0df06UL,
                                          0xfc8acd905UL, 0xfc0a8db04UL, 0xff8b4d503UL, 0xff0b0d702UL, 0xfe8bcd101UL,
                                          0xfe0b8d300UL, 0xf9884cd0fUL, 0xf9080cf0eUL, 0xf888cc90dUL, 0xf8088cb0cUL,
                                          0xfb894c50bUL, 0xfb090c70aUL, 0xfa89cc109UL, 0xfa098c308UL, 0xf58e4fd17UL,
                                          0xf50e0ff16UL, 0xf48ecf915UL, 0xf40e8fb14UL, 0xf78f4f513UL, 0xf70f0f712UL,
                                          0xf68fcf111UL, 0xf60f8f310UL, 0xf18c4ed1fUL, 0xf10c0ef1eUL, 0xf08cce91dUL,
                                          0xf00c8eb1cUL, 0xf38d4e51bUL, 0xf30d0e71aUL, 0xf28dce119UL, 0xf20d8e318UL,
                                          0xcd9241d67UL, 0xcd1201f66UL, 0xcc92c1965UL, 0xcc1281b64UL, 0xcf9341563UL,
                                          0xcf1301762UL, 0xce93c1161UL, 0xce1381360UL, 0xc99040d6fUL, 0xc91000f6eUL,
                                          0xc890c096dUL, 0xc81080b6cUL, 0xcb914056bUL, 0xcb110076aUL, 0xca91c0169UL,
                                          0xca1180368UL, 0xc59643d77UL, 0xc51603f76UL, 0xc496c3975UL, 0xc41683b74UL,
                                          0xc79743573UL, 0xc71703772UL, 0xc697c3171UL, 0xc61783370UL, 0xc19442d7fUL,
                                          0xc11402f7eUL, 0xc094c297dUL, 0xc01482b7cUL, 0xc3954257bUL, 0xc3150277aUL,
                                          0xc295c2179UL, 0xc21582378UL, 0xdd9a45d47UL, 0xdd1a05f46UL, 0xdc9ac5945UL,
                                          0xdc1a85b44UL, 0xdf9b45543UL, 0xdf1b05742UL, 0xde9bc5141UL, 0xde1b85340UL,
                                          0xd99844d4fUL, 0xd91804f4eUL, 0xd898c494dUL, 0xd81884b4cUL, 0xdb994454bUL,
                                          0xdb190474aUL, 0xda99c4149UL, 0xda1984348UL, 0xd59e47d57UL, 0xd51e07f56UL,
                                          0xd49ec7955UL, 0xd41e87b54UL, 0xd79f47553UL, 0xd71f07752UL, 0xd69fc7151UL,
                                          0xd61f87350UL, 0xd19c46d5fUL, 0xd11c06f5eUL, 0xd09cc695dUL, 0xd01c86b5cUL,
                                          0xd39d4655bUL, 0xd31d0675aUL, 0xd29dc6159UL, 0xd21d86358UL, 0xada259da7UL,
                                          0xad2219fa6UL, 0xaca2d99a5UL, 0xac2299ba4UL, 0xafa3595a3UL, 0xaf23197a2UL,
                                          0xaea3d91a1UL, 0xae23993a0UL, 0xa9a058dafUL, 0xa92018faeUL, 0xa8a0d89adUL,
                                          0xa82098bacUL, 0xaba1585abUL, 0xab21187aaUL, 0xaaa1d81a9UL, 0xaa21983a8UL,
                                          0xa5a65bdb7UL, 0xa5261bfb6UL, 0xa4a6db9b5UL, 0xa4269bbb4UL, 0xa7a75b5b3UL,
                                          0xa7271b7b2UL, 0xa6a7db1b1UL, 0xa6279b3b0UL, 0xa1a45adbfUL, 0xa1241afbeUL,
                                          0xa0a4da9bdUL, 0xa0249abbcUL, 0xa3a55a5bbUL, 0xa3251a7baUL, 0xa2a5da1b9UL,
                                          0xa2259a3b8UL, 0xbdaa5dd87UL, 0xbd2a1df86UL, 0xbcaadd985UL, 0xbc2a9db84UL,
                                          0xbfab5d583UL, 0xbf2b1d782UL, 0xbeabdd181UL, 0xbe2b9d380UL, 0xb9a85cd8fUL,
                                          0xb9281cf8eUL, 0xb8a8dc98dUL, 0xb8289cb8cUL, 0xbba95c58bUL, 0xbb291c78aUL,
                                          0xbaa9dc189UL, 0xba299c388UL, 0xb5ae5fd97UL, 0xb52e1ff96UL, 0xb4aedf995UL,
                                          0xb42e9fb94UL, 0xb7af5f593UL, 0xb72f1f792UL, 0xb6afdf191UL, 0xb62f9f390UL,
                                          0xb1ac5ed9fUL, 0xb12c1ef9eUL, 0xb0acde99dUL, 0xb02c9eb9cUL, 0xb3ad5e59bUL,
                                          0xb32d1e79aUL, 0xb2adde199UL, 0xb22d9e398UL, 0x8db251de7UL, 0x8d3211fe6UL,
                                          0x8cb2d19e5UL, 0x8c3291be4UL, 0x8fb3515e3UL, 0x8f33117e2UL, 0x8eb3d11e1UL,
                                          0x8e33913e0UL, 0x89b050defUL, 0x893010feeUL, 0x88b0d09edUL, 0x883090becUL,
                                          0x8bb1505ebUL, 0x8b31107eaUL, 0x8ab1d01e9UL, 0x8a31903e8UL, 0x85b653df7UL,
                                          0x853613ff6UL, 0x84b6d39f5UL, 0x843693bf4UL, 0x87b7535f3UL, 0x8737137f2UL,
                                          0x86b7d31f1UL, 0x8637933f0UL, 0x81b452dffUL, 0x813412ffeUL, 0x80b4d29fdUL,
                                          0x803492bfcUL, 0x83b5525fbUL, 0x8335127faUL, 0x82b5d21f9UL, 0x8235923f8UL,
                                          0x9dba55dc7UL, 0x9d3a15fc6UL, 0x9cbad59c5UL, 0x9c3a95bc4UL, 0x9fbb555c3UL,
                                          0x9f3b157c2UL, 0x9ebbd51c1UL, 0x9e3b953c0UL, 0x99b854dcfUL, 0x993814fceUL,
                                          0x98b8d49cdUL, 0x983894bccUL, 0x9bb9545cbUL, 0x9b39147caUL, 0x9ab9d41c9UL,
                                          0x9a39943c8UL, 0x95be57dd7UL, 0x953e17fd6UL, 0x94bed79d5UL, 0x943e97bd4UL,
                                          0x97bf575d3UL, 0x973f177d2UL, 0x96bfd71d1UL, 0x963f973d0UL, 0x91bc56ddfUL,
                                          0x913c16fdeUL, 0x90bcd69ddUL, 0x903c96bdcUL, 0x93bd565dbUL, 0x933d167daUL,
                                          0x92bdd61d9UL, 0x923d963d8UL};
      fromVector(codes, d._code_id);
      d._nbits = 36;
      d._tau = 4; //
      d._type = ARTOOLKITPLUS;
      d._name = "ARTOOLKITPLUS";
    }
      break;
    case ARTOOLKITPLUSBCH:
    {
      std::vector<std::uint64_t> codes = {0x8f80b8750UL, 0x8f9d0a027UL, 0x8fa66eec9UL, 0x8fbbdc9beUL, 0x8fcd15462UL,
                                          0x8fd0a7315UL, 0x8febc3dfbUL, 0x8ff671a8cUL, 0x8f0650643UL, 0x8f1be2134UL,
                                          0x8f2086fdaUL, 0x8f3d348adUL, 0x8f4bfd571UL, 0x8f564f206UL, 0x8f6d2bce8UL,
                                          0x8f7099b9fUL, 0x8e8d68576UL, 0x8e90da201UL, 0x8eabbecefUL, 0x8eb60cb98UL,
                                          0x8ec0c5644UL, 0x8edd77133UL, 0x8ee613fddUL, 0x8efba18aaUL, 0x8e0b80465UL,
                                          0x8e1632312UL, 0x8e2d56dfcUL, 0x8e30e4a8bUL, 0x8e462d757UL, 0x8e5b9f020UL,
                                          0x8e60fbeceUL, 0x8e7d499b9UL, 0x8d86aa46bUL, 0x8d9b1831cUL, 0x8da07cdf2UL,
                                          0x8dbdcea85UL, 0x8dcb07759UL, 0x8dd6b502eUL, 0x8dedd1ec0UL, 0x8df0639b7UL,
                                          0x8d0042578UL, 0x8d1df020fUL, 0x8d2694ce1UL, 0x8d3b26b96UL, 0x8d4def64aUL,
                                          0x8d505d13dUL, 0x8d6b39fd3UL, 0x8d768b8a4UL, 0x8c8b7a64dUL, 0x8c96c813aUL,
                                          0x8cadacfd4UL, 0x8cb01e8a3UL, 0x8cc6d757fUL, 0x8cdb65208UL, 0x8ce001ce6UL,
                                          0x8cfdb3b91UL, 0x8c0d9275eUL, 0x8c1020029UL, 0x8c2b44ec7UL, 0x8c36f69b0UL,
                                          0x8c403f46cUL, 0x8c5d8d31bUL, 0x8c66e9df5UL, 0x8c7b5ba82UL, 0x8b8c9c126UL,
                                          0x8b912e651UL, 0x8baa4a8bfUL, 0x8bb7f8fc8UL, 0x8bc131214UL, 0x8bdc83563UL,
                                          0x8be7e7b8dUL, 0x8bfa55cfaUL, 0x8b0a74035UL, 0x8b17c6742UL, 0x8b2ca29acUL,
                                          0x8b3110edbUL, 0x8b47d9307UL, 0x8b5a6b470UL, 0x8b610fa9eUL, 0x8b7cbdde9UL,
                                          0x8a814c300UL, 0x8a9cfe477UL, 0x8aa79aa99UL, 0x8aba28deeUL, 0x8acce1032UL,
                                          0x8ad153745UL, 0x8aea379abUL, 0x8af785edcUL, 0x8a07a4213UL, 0x8a1a16564UL,
                                          0x8a2172b8aUL, 0x8a3cc0cfdUL, 0x8a4a09121UL, 0x8a57bb656UL, 0x8a6cdf8b8UL,
                                          0x8a716dfcfUL, 0x898a8e21dUL, 0x89973c56aUL, 0x89ac58b84UL, 0x89b1eacf3UL,
                                          0x89c72312fUL, 0x89da91658UL, 0x89e1f58b6UL, 0x89fc47fc1UL, 0x890c6630eUL,
                                          0x8911d4479UL, 0x892ab0a97UL, 0x893702de0UL, 0x8941cb03cUL, 0x895c7974bUL,
                                          0x89671d9a5UL, 0x897aafed2UL, 0x88875e03bUL, 0x889aec74cUL, 0x88a1889a2UL,
                                          0x88bc3aed5UL, 0x88caf3309UL, 0x88d74147eUL, 0x88ec25a90UL, 0x88f197de7UL,
                                          0x8801b6128UL, 0x881c0465fUL, 0x8827608b1UL, 0x883ad2fc6UL, 0x884c1b21aUL,
                                          0x8851a956dUL, 0x886acdb83UL, 0x88777fcf4UL, 0x878542ccbUL, 0x8798f0bbcUL,
                                          0x87a394552UL, 0x87be26225UL, 0x87c8efff9UL, 0x87d55d88eUL, 0x87ee39660UL,
                                          0x87f38b117UL, 0x8703aadd8UL, 0x871e18aafUL, 0x87257c441UL, 0x8738ce336UL,
                                          0x874e07eeaUL, 0x8753b599dUL, 0x8768d1773UL, 0x877563004UL, 0x868892eedUL,
                                          0x86952099aUL, 0x86ae44774UL, 0x86b3f6003UL, 0x86c53fddfUL, 0x86d88daa8UL,
                                          0x86e3e9446UL, 0x86fe5b331UL, 0x860e7affeUL, 0x8613c8889UL, 0x8628ac667UL,
                                          0x86351e110UL, 0x8643d7cccUL, 0x865e65bbbUL, 0x866501555UL, 0x8678b3222UL,
                                          0x858350ff0UL, 0x859ee2887UL, 0x85a586669UL, 0x85b83411eUL, 0x85cefdcc2UL,
                                          0x85d34fbb5UL, 0x85e82b55bUL, 0x85f59922cUL, 0x8505b8ee3UL, 0x85180a994UL,
                                          0x85236e77aUL, 0x853edc00dUL, 0x854815dd1UL, 0x8555a7aa6UL, 0x856ec3448UL,
                                          0x85737133fUL, 0x848e80dd6UL, 0x849332aa1UL, 0x84a85644fUL, 0x84b5e4338UL,
                                          0x84c32dee4UL, 0x84de9f993UL, 0x84e5fb77dUL, 0x84f84900aUL, 0x840868cc5UL,
                                          0x8415dabb2UL, 0x842ebe55cUL, 0x84330c22bUL, 0x8445c5ff7UL, 0x845877880UL,
                                          0x84631366eUL, 0x847ea1119UL, 0x838966abdUL, 0x8394d4dcaUL, 0x83afb0324UL,
                                          0x83b202453UL, 0x83c4cb98fUL, 0x83d979ef8UL, 0x83e21d016UL, 0x83ffaf761UL,
                                          0x830f8ebaeUL, 0x83123ccd9UL, 0x832958237UL, 0x8334ea540UL, 0x83422389cUL,
                                          0x835f91febUL, 0x8364f5105UL, 0x837947672UL, 0x8284b689bUL, 0x829904fecUL,
                                          0x82a260102UL, 0x82bfd2675UL, 0x82c91bba9UL, 0x82d4a9cdeUL, 0x82efcd230UL,
                                          0x82f27f547UL, 0x82025e988UL, 0x821feceffUL, 0x822488011UL, 0x82393a766UL,
                                          0x824ff3abaUL, 0x825241dcdUL, 0x826925323UL, 0x827497454UL, 0x818f74986UL,
                                          0x8192c6ef1UL, 0x81a9a201fUL, 0x81b410768UL, 0x81c2d9ab4UL, 0x81df6bdc3UL,
                                          0x81e40f32dUL, 0x81f9bd45aUL, 0x81099c895UL, 0x81142efe2UL, 0x812f4a10cUL,
                                          0x8132f867bUL, 0x814431ba7UL, 0x815983cd0UL, 0x8162e723eUL, 0x817f55549UL,
                                          0x8082a4ba0UL, 0x809f16cd7UL, 0x80a472239UL, 0x80b9c054eUL, 0x80cf09892UL,
                                          0x80d2bbfe5UL, 0x80e9df10bUL, 0x80f46d67cUL, 0x80044cab3UL, 0x8019fedc4UL,
                                          0x80229a32aUL, 0x803f2845dUL, 0x8049e1981UL, 0x805453ef6UL, 0x806f37018UL,
                                          0x80728576fUL, 0x9f8b4d066UL, 0x9f96ff711UL, 0x9fad9b9ffUL, 0x9fb029e88UL,
                                          0x9fc6e0354UL, 0x9fdb52423UL, 0x9fe036acdUL, 0x9ffd84dbaUL, 0x9f0da5175UL,
                                          0x9f1017602UL, 0x9f2b738ecUL, 0x9f36c1f9bUL, 0x9f4008247UL, 0x9f5dba530UL,
                                          0x9f66debdeUL, 0x9f7b6cca9UL, 0x9e869d240UL, 0x9e9b2f537UL, 0x9ea04bbd9UL,
                                          0x9ebdf9caeUL, 0x9ecb30172UL, 0x9ed682605UL, 0x9eede68ebUL, 0x9ef054f9cUL,
                                          0x9e0075353UL, 0x9e1dc7424UL, 0x9e26a3acaUL, 0x9e3b11dbdUL, 0x9e4dd8061UL,
                                          0x9e506a716UL, 0x9e6b0e9f8UL, 0x9e76bce8fUL, 0x9d8d5f35dUL, 0x9d90ed42aUL,
                                          0x9dab89ac4UL, 0x9db63bdb3UL, 0x9dc0f206fUL, 0x9ddd40718UL, 0x9de6249f6UL,
                                          0x9dfb96e81UL, 0x9d0bb724eUL, 0x9d1605539UL, 0x9d2d61bd7UL, 0x9d30d3ca0UL,
                                          0x9d461a17cUL, 0x9d5ba860bUL, 0x9d60cc8e5UL, 0x9d7d7ef92UL, 0x9c808f17bUL,
                                          0x9c9d3d60cUL, 0x9ca6598e2UL, 0x9cbbebf95UL, 0x9ccd22249UL, 0x9cd09053eUL,
                                          0x9cebf4bd0UL, 0x9cf646ca7UL, 0x9c0667068UL, 0x9c1bd571fUL, 0x9c20b19f1UL,
                                          0x9c3d03e86UL, 0x9c4bca35aUL, 0x9c567842dUL, 0x9c6d1cac3UL, 0x9c70aedb4UL,
                                          0x9b8769610UL, 0x9b9adb167UL, 0x9ba1bff89UL, 0x9bbc0d8feUL, 0x9bcac4522UL,
                                          0x9bd776255UL, 0x9bec12cbbUL, 0x9bf1a0bccUL, 0x9b0181703UL, 0x9b1c33074UL,
                                          0x9b2757e9aUL, 0x9b3ae59edUL, 0x9b4c2c431UL, 0x9b519e346UL, 0x9b6afada8UL,
                                          0x9b7748adfUL, 0x9a8ab9436UL, 0x9a970b341UL, 0x9aac6fdafUL, 0x9ab1ddad8UL,
                                          0x9ac714704UL, 0x9adaa6073UL, 0x9ae1c2e9dUL, 0x9afc709eaUL, 0x9a0c51525UL,
                                          0x9a11e3252UL, 0x9a2a87cbcUL, 0x9a3735bcbUL, 0x9a41fc617UL, 0x9a5c4e160UL,
                                          0x9a672af8eUL, 0x9a7a988f9UL, 0x99817b52bUL, 0x999cc925cUL, 0x99a7adcb2UL,
                                          0x99ba1fbc5UL, 0x99ccd6619UL, 0x99d16416eUL, 0x99ea00f80UL, 0x99f7b28f7UL,
                                          0x990793438UL, 0x991a2134fUL, 0x992145da1UL, 0x993cf7ad6UL, 0x994a3e70aUL,
                                          0x99578c07dUL, 0x996ce8e93UL, 0x99715a9e4UL, 0x988cab70dUL, 0x98911907aUL,
                                          0x98aa7de94UL, 0x98b7cf9e3UL, 0x98c10643fUL, 0x98dcb4348UL, 0x98e7d0da6UL,
                                          0x98fa62ad1UL, 0x980a4361eUL, 0x9817f1169UL, 0x982c95f87UL, 0x9831278f0UL,
                                          0x9847ee52cUL, 0x985a5c25bUL, 0x986138cb5UL, 0x987c8abc2UL, 0x978eb7bfdUL,
                                          0x979305c8aUL, 0x97a861264UL, 0x97b5d3513UL, 0x97c31a8cfUL, 0x97dea8fb8UL,
                                          0x97e5cc156UL, 0x97f87e621UL, 0x97085faeeUL, 0x9715edd99UL, 0x972e89377UL,
                                          0x97333b400UL, 0x9745f29dcUL, 0x975840eabUL, 0x976324045UL, 0x977e96732UL,
                                          0x9683679dbUL, 0x969ed5eacUL, 0x96a5b1042UL, 0x96b803735UL, 0x96cecaae9UL,
                                          0x96d378d9eUL, 0x96e81c370UL, 0x96f5ae407UL, 0x96058f8c8UL, 0x96183dfbfUL,
                                          0x962359151UL, 0x963eeb626UL, 0x964822bfaUL, 0x965590c8dUL, 0x966ef4263UL,
                                          0x967346514UL, 0x9588a58c6UL, 0x959517fb1UL, 0x95ae7315fUL, 0x95b3c1628UL,
                                          0x95c508bf4UL, 0x95d8bac83UL, 0x95e3de26dUL, 0x95fe6c51aUL, 0x950e4d9d5UL,
                                          0x9513ffea2UL, 0x95289b04cUL, 0x95352973bUL, 0x9543e0ae7UL, 0x955e52d90UL,
                                          0x95653637eUL, 0x957884409UL, 0x948575ae0UL, 0x9498c7d97UL, 0x94a3a3379UL,
                                          0x94be1140eUL, 0x94c8d89d2UL, 0x94d56aea5UL, 0x94ee0e04bUL, 0x94f3bc73cUL,
                                          0x94039dbf3UL, 0x941e2fc84UL, 0x94254b26aUL, 0x9438f951dUL, 0x944e308c1UL,
                                          0x945382fb6UL, 0x9468e6158UL, 0x94755462fUL, 0x938293d8bUL, 0x939f21afcUL,
                                          0x93a445412UL, 0x93b9f7365UL, 0x93cf3eeb9UL, 0x93d28c9ceUL, 0x93e9e8720UL,
                                          0x93f45a057UL, 0x93047bc98UL, 0x9319c9befUL, 0x9322ad501UL, 0x933f1f276UL,
                                          0x9349d6faaUL, 0x9354648ddUL, 0x936f00633UL, 0x9372b2144UL, 0x928f43fadUL,
                                          0x9292f18daUL, 0x92a995634UL, 0x92b427143UL, 0x92c2eec9fUL, 0x92df5cbe8UL,
                                          0x92e438506UL, 0x92f98a271UL, 0x9209abebeUL, 0x9214199c9UL, 0x922f7d727UL,
                                          0x9232cf050UL, 0x924406d8cUL, 0x9259b4afbUL, 0x9262d0415UL, 0x927f62362UL,
                                          0x918481eb0UL, 0x9199339c7UL, 0x91a257729UL, 0x91bfe505eUL, 0x91c92cd82UL,
                                          0x91d49eaf5UL, 0x91effa41bUL, 0x91f24836cUL, 0x910269fa3UL, 0x911fdb8d4UL,
                                          0x9124bf63aUL, 0x91390d14dUL, 0x914fc4c91UL, 0x915276be6UL, 0x916912508UL,
                                          0x9174a027fUL, 0x908951c96UL, 0x9094e3be1UL, 0x90af8750fUL, 0x90b235278UL,
                                          0x90c4fcfa4UL, 0x90d94e8d3UL, 0x90e22a63dUL, 0x90ff9814aUL, 0x900fb9d85UL,
                                          0x90120baf2UL, 0x90296f41cUL, 0x9034dd36bUL, 0x904214eb7UL, 0x905fa69c0UL,
                                          0x9064c272eUL, 0x907970059UL, 0xaf8ae0e4bUL, 0xaf975293cUL, 0xafac367d2UL,
                                          0xafb1840a5UL, 0xafc74dd79UL, 0xafdaffa0eUL, 0xafe19b4e0UL, 0xaffc29397UL,
                                          0xaf0c08f58UL, 0xaf11ba82fUL, 0xaf2ade6c1UL, 0xaf376c1b6UL, 0xaf41a5c6aUL,
                                          0xaf5c17b1dUL, 0xaf67735f3UL, 0xaf7ac1284UL, 0xae8730c6dUL, 0xae9a82b1aUL,
                                          0xaea1e65f4UL, 0xaebc54283UL, 0xaeca9df5fUL, 0xaed72f828UL, 0xaeec4b6c6UL,
                                          0xaef1f91b1UL, 0xae01d8d7eUL, 0xae1c6aa09UL, 0xae270e4e7UL, 0xae3abc390UL,
                                          0xae4c75e4cUL, 0xae51c793bUL, 0xae6aa37d5UL, 0xae77110a2UL, 0xad8cf2d70UL,
                                          0xad9140a07UL, 0xadaa244e9UL, 0xadb79639eUL, 0xadc15fe42UL, 0xaddced935UL,
                                          0xade7897dbUL, 0xadfa3b0acUL, 0xad0a1ac63UL, 0xad17a8b14UL, 0xad2ccc5faUL,
                                          0xad317e28dUL, 0xad47b7f51UL, 0xad5a05826UL, 0xad61616c8UL, 0xad7cd31bfUL,
                                          0xac8122f56UL, 0xac9c90821UL, 0xaca7f46cfUL, 0xacba461b8UL, 0xaccc8fc64UL,
                                          0xacd13db13UL, 0xacea595fdUL, 0xacf7eb28aUL, 0xac07cae45UL, 0xac1a78932UL,
                                          0xac211c7dcUL, 0xac3cae0abUL, 0xac4a67d77UL, 0xac57d5a00UL, 0xac6cb14eeUL,
                                          0xac7103399UL, 0xab86c483dUL, 0xab9b76f4aUL, 0xaba0121a4UL, 0xabbda06d3UL,
                                          0xabcb69b0fUL, 0xabd6dbc78UL, 0xabedbf296UL, 0xabf00d5e1UL, 0xab002c92eUL,
                                          0xab1d9ee59UL, 0xab26fa0b7UL, 0xab3b487c0UL, 0xab4d81a1cUL, 0xab5033d6bUL,
                                          0xab6b57385UL, 0xab76e54f2UL, 0xaa8b14a1bUL, 0xaa96a6d6cUL, 0xaaadc2382UL,
                                          0xaab0704f5UL, 0xaac6b9929UL, 0xaadb0be5eUL, 0xaae06f0b0UL, 0xaafddd7c7UL,
                                          0xaa0dfcb08UL, 0xaa104ec7fUL, 0xaa2b2a291UL, 0xaa36985e6UL, 0xaa405183aUL,
                                          0xaa5de3f4dUL, 0xaa66871a3UL, 0xaa7b356d4UL, 0xa980d6b06UL, 0xa99d64c71UL,
                                          0xa9a60029fUL, 0xa9bbb25e8UL, 0xa9cd7b834UL, 0xa9d0c9f43UL, 0xa9ebad1adUL,
                                          0xa9f61f6daUL, 0xa9063ea15UL, 0xa91b8cd62UL, 0xa920e838cUL, 0xa93d5a4fbUL,
                                          0xa94b93927UL, 0xa95621e50UL, 0xa96d450beUL, 0xa970f77c9UL, 0xa88d06920UL,
                                          0xa890b4e57UL, 0xa8abd00b9UL, 0xa8b6627ceUL, 0xa8c0aba12UL, 0xa8dd19d65UL,
                                          0xa8e67d38bUL, 0xa8fbcf4fcUL, 0xa80bee833UL, 0xa8165cf44UL, 0xa82d381aaUL,
                                          0xa8308a6ddUL, 0xa84643b01UL, 0xa85bf1c76UL, 0xa86095298UL, 0xa87d275efUL,
                                          0xa78f1a5d0UL, 0xa792a82a7UL, 0xa7a9ccc49UL, 0xa7b47eb3eUL, 0xa7c2b76e2UL,
                                          0xa7df05195UL, 0xa7e461f7bUL, 0xa7f9d380cUL, 0xa709f24c3UL, 0xa714403b4UL,
                                          0xa72f24d5aUL, 0xa73296a2dUL, 0xa7445f7f1UL, 0xa759ed086UL, 0xa76289e68UL,
                                          0xa77f3b91fUL, 0xa682ca7f6UL, 0xa69f78081UL, 0xa6a41ce6fUL, 0xa6b9ae918UL,
                                          0xa6cf674c4UL, 0xa6d2d53b3UL, 0xa6e9b1d5dUL, 0xa6f403a2aUL, 0xa604226e5UL,
                                          0xa61990192UL, 0xa622f4f7cUL, 0xa63f4680bUL, 0xa6498f5d7UL, 0xa6543d2a0UL,
                                          0xa66f59c4eUL, 0xa672ebb39UL, 0xa589086ebUL, 0xa594ba19cUL, 0xa5afdef72UL,
                                          0xa5b26c805UL, 0xa5c4a55d9UL, 0xa5d9172aeUL, 0xa5e273c40UL, 0xa5ffc1b37UL,
                                          0xa50fe07f8UL, 0xa5125208fUL, 0xa52936e61UL, 0xa53484916UL, 0xa5424d4caUL,
                                          0xa55fff3bdUL, 0xa5649bd53UL, 0xa57929a24UL, 0xa484d84cdUL, 0xa4996a3baUL,
                                          0xa4a20ed54UL, 0xa4bfbca23UL, 0xa4c9757ffUL, 0xa4d4c7088UL, 0xa4efa3e66UL,
                                          0xa4f211911UL, 0xa402305deUL, 0xa41f822a9UL, 0xa424e6c47UL, 0xa43954b30UL,
                                          0xa44f9d6ecUL, 0xa4522f19bUL, 0xa4694bf75UL, 0xa474f9802UL, 0xa3833e3a6UL,
                                          0xa39e8c4d1UL, 0xa3a5e8a3fUL, 0xa3b85ad48UL, 0xa3ce93094UL, 0xa3d3217e3UL,
                                          0xa3e84590dUL, 0xa3f5f7e7aUL, 0xa305d62b5UL, 0xa318645c2UL, 0xa32300b2cUL,
                                          0xa33eb2c5bUL, 0xa3487b187UL, 0xa355c96f0UL, 0xa36ead81eUL, 0xa3731ff69UL,
                                          0xa28eee180UL, 0xa2935c6f7UL, 0xa2a838819UL, 0xa2b58af6eUL, 0xa2c3432b2UL,
                                          0xa2def15c5UL, 0xa2e595b2bUL, 0xa2f827c5cUL, 0xa20806093UL, 0xa215b47e4UL,
                                          0xa22ed090aUL, 0xa23362e7dUL, 0xa245ab3a1UL, 0xa258194d6UL, 0xa2637da38UL,
                                          0xa27ecfd4fUL, 0xa1852c09dUL, 0xa1989e7eaUL, 0xa1a3fa904UL, 0xa1be48e73UL,
                                          0xa1c8813afUL, 0xa1d5334d8UL, 0xa1ee57a36UL, 0xa1f3e5d41UL, 0xa103c418eUL,
                                          0xa11e766f9UL, 0xa12512817UL, 0xa138a0f60UL, 0xa14e692bcUL, 0xa153db5cbUL,
                                          0xa168bfb25UL, 0xa1750dc52UL, 0xa088fc2bbUL, 0xa0954e5ccUL, 0xa0ae2ab22UL,
                                          0xa0b398c55UL, 0xa0c551189UL, 0xa0d8e36feUL, 0xa0e387810UL, 0xa0fe35f67UL,
                                          0xa00e143a8UL, 0xa013a64dfUL, 0xa028c2a31UL, 0xa03570d46UL, 0xa043b909aUL,
                                          0xa05e0b7edUL, 0xa0656f903UL, 0xa078dde74UL, 0xbf811597dUL, 0xbf9ca7e0aUL,
                                          0xbfa7c30e4UL, 0xbfba71793UL, 0xbfccb8a4fUL, 0xbfd10ad38UL, 0xbfea6e3d6UL,
                                          0xbff7dc4a1UL, 0xbf07fd86eUL, 0xbf1a4ff19UL, 0xbf212b1f7UL, 0xbf3c99680UL,
                                          0xbf4a50b5cUL, 0xbf57e2c2bUL, 0xbf6c862c5UL, 0xbf71345b2UL, 0xbe8cc5b5bUL,
                                          0xbe9177c2cUL, 0xbeaa132c2UL, 0xbeb7a15b5UL, 0xbec168869UL, 0xbedcdaf1eUL,
                                          0xbee7be1f0UL, 0xbefa0c687UL, 0xbe0a2da48UL, 0xbe179fd3fUL, 0xbe2cfb3d1UL,
                                          0xbe31494a6UL, 0xbe478097aUL, 0xbe5a32e0dUL, 0xbe61560e3UL, 0xbe7ce4794UL,
                                          0xbd8707a46UL, 0xbd9ab5d31UL, 0xbda1d13dfUL, 0xbdbc634a8UL, 0xbdcaaa974UL,
                                          0xbdd718e03UL, 0xbdec7c0edUL, 0xbdf1ce79aUL, 0xbd01efb55UL, 0xbd1c5dc22UL,
                                          0xbd27392ccUL, 0xbd3a8b5bbUL, 0xbd4c42867UL, 0xbd51f0f10UL, 0xbd6a941feUL,
                                          0xbd7726689UL, 0xbc8ad7860UL, 0xbc9765f17UL, 0xbcac011f9UL, 0xbcb1b368eUL,
                                          0xbcc77ab52UL, 0xbcdac8c25UL, 0xbce1ac2cbUL, 0xbcfc1e5bcUL, 0xbc0c3f973UL,
                                          0xbc118de04UL, 0xbc2ae90eaUL, 0xbc375b79dUL, 0xbc4192a41UL, 0xbc5c20d36UL,
                                          0xbc67443d8UL, 0xbc7af64afUL, 0xbb8d31f0bUL, 0xbb908387cUL, 0xbbabe7692UL,
                                          0xbbb6551e5UL, 0xbbc09cc39UL, 0xbbdd2eb4eUL, 0xbbe64a5a0UL, 0xbbfbf82d7UL,
                                          0xbb0bd9e18UL, 0xbb166b96fUL, 0xbb2d0f781UL, 0xbb30bd0f6UL, 0xbb4674d2aUL,
                                          0xbb5bc6a5dUL, 0xbb60a24b3UL, 0xbb7d103c4UL, 0xba80e1d2dUL, 0xba9d53a5aUL,
                                          0xbaa6374b4UL, 0xbabb853c3UL, 0xbacd4ce1fUL, 0xbad0fe968UL, 0xbaeb9a786UL,
                                          0xbaf6280f1UL, 0xba0609c3eUL, 0xba1bbbb49UL, 0xba20df5a7UL, 0xba3d6d2d0UL,
                                          0xba4ba4f0cUL, 0xba561687bUL, 0xba6d72695UL, 0xba70c01e2UL, 0xb98b23c30UL,
                                          0xb99691b47UL, 0xb9adf55a9UL, 0xb9b0472deUL, 0xb9c68ef02UL, 0xb9db3c875UL,
                                          0xb9e05869bUL, 0xb9fdea1ecUL, 0xb90dcbd23UL, 0xb91079a54UL, 0xb92b1d4baUL,
                                          0xb936af3cdUL, 0xb94066e11UL, 0xb95dd4966UL, 0xb966b0788UL, 0xb97b020ffUL,
                                          0xb886f3e16UL, 0xb89b41961UL, 0xb8a02578fUL, 0xb8bd970f8UL, 0xb8cb5ed24UL,
                                          0xb8d6eca53UL, 0xb8ed884bdUL, 0xb8f03a3caUL, 0xb8001bf05UL, 0xb81da9872UL,
                                          0xb826cd69cUL, 0xb83b7f1ebUL, 0xb84db6c37UL, 0xb85004b40UL, 0xb86b605aeUL,
                                          0xb876d22d9UL, 0xb784ef2e6UL, 0xb7995d591UL, 0xb7a239b7fUL, 0xb7bf8bc08UL,
                                          0xb7c9421d4UL, 0xb7d4f06a3UL, 0xb7ef9484dUL, 0xb7f226f3aUL, 0xb702073f5UL,
                                          0xb71fb5482UL, 0xb724d1a6cUL, 0xb73963d1bUL, 0xb74faa0c7UL, 0xb752187b0UL,
                                          0xb7697c95eUL, 0xb774cee29UL, 0xb6893f0c0UL, 0xb6948d7b7UL, 0xb6afe9959UL,
                                          0xb6b25be2eUL, 0xb6c4923f2UL, 0xb6d920485UL, 0xb6e244a6bUL, 0xb6fff6d1cUL,
                                          0xb60fd71d3UL, 0xb612656a4UL, 0xb6290184aUL, 0xb634b3f3dUL, 0xb6427a2e1UL,
                                          0xb65fc8596UL, 0xb664acb78UL, 0xb6791ec0fUL, 0xb582fd1ddUL, 0xb59f4f6aaUL,
                                          0xb5a42b844UL, 0xb5b999f33UL, 0xb5cf502efUL, 0xb5d2e2598UL, 0xb5e986b76UL,
                                          0xb5f434c01UL, 0xb504150ceUL, 0xb519a77b9UL, 0xb522c3957UL, 0xb53f71e20UL,
                                          0xb549b83fcUL, 0xb5540a48bUL, 0xb56f6ea65UL, 0xb572dcd12UL, 0xb48f2d3fbUL,
                                          0xb4929f48cUL, 0xb4a9fba62UL, 0xb4b449d15UL, 0xb4c2800c9UL, 0xb4df327beUL,
                                          0xb4e456950UL, 0xb4f9e4e27UL, 0xb409c52e8UL, 0xb4147759fUL, 0xb42f13b71UL,
                                          0xb432a1c06UL, 0xb444681daUL, 0xb459da6adUL, 0xb462be843UL, 0xb47f0cf34UL,
                                          0xb388cb490UL, 0xb395793e7UL, 0xb3ae1dd09UL, 0xb3b3afa7eUL, 0xb3c5667a2UL,
                                          0xb3d8d40d5UL, 0xb3e3b0e3bUL, 0xb3fe0294cUL, 0xb30e23583UL, 0xb313912f4UL,
                                          0xb328f5c1aUL, 0xb33547b6dUL, 0xb3438e6b1UL, 0xb35e3c1c6UL, 0xb36558f28UL,
                                          0xb378ea85fUL, 0xb2851b6b6UL, 0xb298a91c1UL, 0xb2a3cdf2fUL, 0xb2be7f858UL,
                                          0xb2c8b6584UL, 0xb2d5042f3UL, 0xb2ee60c1dUL, 0xb2f3d2b6aUL, 0xb203f37a5UL,
                                          0xb21e410d2UL, 0xb22525e3cUL, 0xb2389794bUL, 0xb24e5e497UL, 0xb253ec3e0UL,
                                          0xb26888d0eUL, 0xb2753aa79UL, 0xb18ed97abUL, 0xb1936b0dcUL, 0xb1a80fe32UL,
                                          0xb1b5bd945UL, 0xb1c374499UL, 0xb1dec63eeUL, 0xb1e5a2d00UL, 0xb1f810a77UL,
                                          0xb108316b8UL, 0xb115831cfUL, 0xb12ee7f21UL, 0xb13355856UL, 0xb1459c58aUL,
                                          0xb1582e2fdUL, 0xb1634ac13UL, 0xb17ef8b64UL, 0xb0830958dUL, 0xb09ebb2faUL,
                                          0xb0a5dfc14UL, 0xb0b86db63UL, 0xb0cea46bfUL, 0xb0d3161c8UL, 0xb0e872f26UL,
                                          0xb0f5c0851UL, 0xb005e149eUL, 0xb018533e9UL, 0xb02337d07UL, 0xb03e85a70UL,
                                          0xb0484c7acUL, 0xb055fe0dbUL, 0xb06e9ae35UL, 0xb07328942UL, 0xcf89bb211UL,
                                          0xcf9409566UL, 0xcfaf6db88UL, 0xcfb2dfcffUL, 0xcfc416123UL, 0xcfd9a4654UL,
                                          0xcfe2c08baUL, 0xcfff72fcdUL, 0xcf0f53302UL, 0xcf12e1475UL, 0xcf2985a9bUL,
                                          0xcf3437decUL, 0xcf42fe030UL, 0xcf5f4c747UL, 0xcf64289a9UL, 0xcf799aedeUL,
                                          0xce846b037UL, 0xce99d9740UL, 0xcea2bd9aeUL, 0xcebf0fed9UL, 0xcec9c6305UL,
                                          0xced474472UL, 0xceef10a9cUL, 0xcef2a2debUL, 0xce0283124UL, 0xce1f31653UL,
                                          0xce24558bdUL, 0xce39e7fcaUL, 0xce4f2e216UL, 0xce529c561UL, 0xce69f8b8fUL,
                                          0xce744acf8UL, 0xcd8fa912aUL, 0xcd921b65dUL, 0xcda97f8b3UL, 0xcdb4cdfc4UL,
                                          0xcdc204218UL, 0xcddfb656fUL, 0xcde4d2b81UL, 0xcdf960cf6UL, 0xcd0941039UL,
                                          0xcd14f374eUL, 0xcd2f979a0UL, 0xcd3225ed7UL, 0xcd44ec30bUL, 0xcd595e47cUL,
                                          0xcd623aa92UL, 0xcd7f88de5UL, 0xcc827930cUL, 0xcc9fcb47bUL, 0xcca4afa95UL,
                                          0xccb91dde2UL, 0xcccfd403eUL, 0xccd266749UL, 0xcce9029a7UL, 0xccf4b0ed0UL,
                                          0xcc049121fUL, 0xcc1923568UL, 0xcc2247b86UL, 0xcc3ff5cf1UL, 0xcc493c12dUL,
                                          0xcc548e65aUL, 0xcc6fea8b4UL, 0xcc7258fc3UL, 0xcb859f467UL, 0xcb982d310UL,
                                          0xcba349dfeUL, 0xcbbefba89UL, 0xcbc832755UL, 0xcbd580022UL, 0xcbeee4eccUL,
                                          0xcbf3569bbUL, 0xcb0377574UL, 0xcb1ec5203UL, 0xcb25a1cedUL, 0xcb3813b9aUL,
                                          0xcb4eda646UL, 0xcb5368131UL, 0xcb680cfdfUL, 0xcb75be8a8UL, 0xca884f641UL,
                                          0xca95fd136UL, 0xcaae99fd8UL, 0xcab32b8afUL, 0xcac5e2573UL, 0xcad850204UL,
                                          0xcae334ceaUL, 0xcafe86b9dUL, 0xca0ea7752UL, 0xca1315025UL, 0xca2871ecbUL,
                                          0xca35c39bcUL, 0xca430a460UL, 0xca5eb8317UL, 0xca65dcdf9UL, 0xca786ea8eUL,
                                          0xc9838d75cUL, 0xc99e3f02bUL, 0xc9a55bec5UL, 0xc9b8e99b2UL, 0xc9ce2046eUL,
                                          0xc9d392319UL, 0xc9e8f6df7UL, 0xc9f544a80UL, 0xc9056564fUL, 0xc918d7138UL,
                                          0xc923b3fd6UL, 0xc93e018a1UL, 0xc948c857dUL, 0xc9557a20aUL, 0xc96e1ece4UL,
                                          0xc973acb93UL, 0xc88e5d57aUL, 0xc893ef20dUL, 0xc8a88bce3UL, 0xc8b539b94UL,
                                          0xc8c3f0648UL, 0xc8de4213fUL, 0xc8e526fd1UL, 0xc8f8948a6UL, 0xc808b5469UL,
                                          0xc8150731eUL, 0xc82e63df0UL, 0xc833d1a87UL, 0xc8451875bUL, 0xc858aa02cUL,
                                          0xc863ceec2UL, 0xc87e7c9b5UL, 0xc78c4198aUL, 0xc791f3efdUL, 0xc7aa97013UL,
                                          0xc7b725764UL, 0xc7c1ecab8UL, 0xc7dc5edcfUL, 0xc7e73a321UL, 0xc7fa88456UL,
                                          0xc70aa9899UL, 0xc7171bfeeUL, 0xc72c7f100UL, 0xc731cd677UL, 0xc74704babUL,
                                          0xc75ab6cdcUL, 0xc761d2232UL, 0xc77c60545UL, 0xc68191bacUL, 0xc69c23cdbUL,
                                          0xc6a747235UL, 0xc6baf5542UL, 0xc6cc3c89eUL, 0xc6d18efe9UL, 0xc6eaea107UL,
                                          0xc6f758670UL, 0xc60779abfUL, 0xc61acbdc8UL, 0xc621af326UL, 0xc63c1d451UL,
                                          0xc64ad498dUL, 0xc65766efaUL, 0xc66c02014UL, 0xc671b0763UL, 0xc58a53ab1UL,
                                          0xc597e1dc6UL, 0xc5ac85328UL, 0xc5b13745fUL, 0xc5c7fe983UL, 0xc5da4cef4UL,
                                          0xc5e12801aUL, 0xc5fc9a76dUL, 0xc50cbbba2UL, 0xc51109cd5UL, 0xc52a6d23bUL,
                                          0xc537df54cUL, 0xc54116890UL, 0xc55ca4fe7UL, 0xc567c0109UL, 0xc57a7267eUL,
                                          0xc48783897UL, 0xc49a31fe0UL, 0xc4a15510eUL, 0xc4bce7679UL, 0xc4ca2eba5UL,
                                          0xc4d79ccd2UL, 0xc4ecf823cUL, 0xc4f14a54bUL, 0xc4016b984UL, 0xc41cd9ef3UL,
                                          0xc427bd01dUL, 0xc43a0f76aUL, 0xc44cc6ab6UL, 0xc45174dc1UL, 0xc46a1032fUL,
                                          0xc477a2458UL, 0xc38065ffcUL, 0xc39dd788bUL, 0xc3a6b3665UL, 0xc3bb01112UL,
                                          0xc3cdc8cceUL, 0xc3d07abb9UL, 0xc3eb1e557UL, 0xc3f6ac220UL, 0xc3068deefUL,
                                          0xc31b3f998UL, 0xc3205b776UL, 0xc33de9001UL, 0xc34b20dddUL, 0xc35692aaaUL,
                                          0xc36df6444UL, 0xc37044333UL, 0xc28db5ddaUL, 0xc29007aadUL, 0xc2ab63443UL,
                                          0xc2b6d1334UL, 0xc2c018ee8UL, 0xc2ddaa99fUL, 0xc2e6ce771UL, 0xc2fb7c006UL,
                                          0xc20b5dcc9UL, 0xc216efbbeUL, 0xc22d8b550UL, 0xc23039227UL, 0xc246f0ffbUL,
                                          0xc25b4288cUL, 0xc26026662UL, 0xc27d94115UL, 0xc18677cc7UL, 0xc19bc5bb0UL,
                                          0xc1a0a155eUL, 0xc1bd13229UL, 0xc1cbdaff5UL, 0xc1d668882UL, 0xc1ed0c66cUL,
                                          0xc1f0be11bUL, 0xc1009fdd4UL, 0xc11d2daa3UL, 0xc1264944dUL, 0xc13bfb33aUL,
                                          0xc14d32ee6UL, 0xc15080991UL, 0xc16be477fUL, 0xc17656008UL, 0xc08ba7ee1UL,
                                          0xc09615996UL, 0xc0ad71778UL, 0xc0b0c300fUL, 0xc0c60add3UL, 0xc0dbb8aa4UL,
                                          0xc0e0dc44aUL, 0xc0fd6e33dUL, 0xc00d4fff2UL, 0xc010fd885UL, 0xc02b9966bUL,
                                          0xc0362b11cUL, 0xc040e2cc0UL, 0xc05d50bb7UL, 0xc06634559UL, 0xc07b8622eUL,
                                          0xdf824e527UL, 0xdf9ffc250UL, 0xdfa498cbeUL, 0xdfb92abc9UL, 0xdfcfe3615UL,
                                          0xdfd251162UL, 0xdfe935f8cUL, 0xdff4878fbUL, 0xdf04a6434UL, 0xdf1914343UL,
                                          0xdf2270dadUL, 0xdf3fc2adaUL, 0xdf490b706UL, 0xdf54b9071UL, 0xdf6fdde9fUL,
                                          0xdf726f9e8UL, 0xde8f9e701UL, 0xde922c076UL, 0xdea948e98UL, 0xdeb4fa9efUL,
                                          0xdec233433UL, 0xdedf81344UL, 0xdee4e5daaUL, 0xdef957addUL, 0xde0976612UL,
                                          0xde14c4165UL, 0xde2fa0f8bUL, 0xde32128fcUL, 0xde44db520UL, 0xde5969257UL,
                                          0xde620dcb9UL, 0xde7fbfbceUL, 0xdd845c61cUL, 0xdd99ee16bUL, 0xdda28af85UL,
                                          0xddbf388f2UL, 0xddc9f152eUL, 0xddd443259UL, 0xddef27cb7UL, 0xddf295bc0UL,
                                          0xdd02b470fUL, 0xdd1f06078UL, 0xdd2462e96UL, 0xdd39d09e1UL, 0xdd4f1943dUL,
                                          0xdd52ab34aUL, 0xdd69cfda4UL, 0xdd747dad3UL, 0xdc898c43aUL, 0xdc943e34dUL,
                                          0xdcaf5ada3UL, 0xdcb2e8ad4UL, 0xdcc421708UL, 0xdcd99307fUL, 0xdce2f7e91UL,
                                          0xdcff459e6UL, 0xdc0f64529UL, 0xdc12d625eUL, 0xdc29b2cb0UL, 0xdc3400bc7UL,
                                          0xdc42c961bUL, 0xdc5f7b16cUL, 0xdc641ff82UL, 0xdc79ad8f5UL, 0xdb8e6a351UL,
                                          0xdb93d8426UL, 0xdba8bcac8UL, 0xdbb50edbfUL, 0xdbc3c7063UL, 0xdbde75714UL,
                                          0xdbe5119faUL, 0xdbf8a3e8dUL, 0xdb0882242UL, 0xdb1530535UL, 0xdb2e54bdbUL,
                                          0xdb33e6cacUL, 0xdb452f170UL, 0xdb589d607UL, 0xdb63f98e9UL, 0xdb7e4bf9eUL,
                                          0xda83ba177UL, 0xda9e08600UL, 0xdaa56c8eeUL, 0xdab8def99UL, 0xdace17245UL,
                                          0xdad3a5532UL, 0xdae8c1bdcUL, 0xdaf573cabUL, 0xda0552064UL, 0xda18e0713UL,
                                          0xda23849fdUL, 0xda3e36e8aUL, 0xda48ff356UL, 0xda554d421UL, 0xda6e29acfUL,
                                          0xda739bdb8UL, 0xd9887806aUL, 0xd995ca71dUL, 0xd9aeae9f3UL, 0xd9b31ce84UL,
                                          0xd9c5d5358UL, 0xd9d86742fUL, 0xd9e303ac1UL, 0xd9feb1db6UL, 0xd90e90179UL,
                                          0xd9132260eUL, 0xd928468e0UL, 0xd935f4f97UL, 0xd9433d24bUL, 0xd95e8f53cUL,
                                          0xd965ebbd2UL, 0xd97859ca5UL, 0xd885a824cUL, 0xd8981a53bUL, 0xd8a37ebd5UL,
                                          0xd8beccca2UL, 0xd8c80517eUL, 0xd8d5b7609UL, 0xd8eed38e7UL, 0xd8f361f90UL,
                                          0xd8034035fUL, 0xd81ef2428UL, 0xd82596ac6UL, 0xd83824db1UL, 0xd84eed06dUL,
                                          0xd8535f71aUL, 0xd8683b9f4UL, 0xd87589e83UL, 0xd787b4ebcUL, 0xd79a069cbUL,
                                          0xd7a162725UL, 0xd7bcd0052UL, 0xd7ca19d8eUL, 0xd7d7abaf9UL, 0xd7eccf417UL,
                                          0xd7f17d360UL, 0xd7015cfafUL, 0xd71cee8d8UL, 0xd7278a636UL, 0xd73a38141UL,
                                          0xd74cf1c9dUL, 0xd75143beaUL, 0xd76a27504UL, 0xd77795273UL, 0xd68a64c9aUL,
                                          0xd697d6bedUL, 0xd6acb2503UL, 0xd6b100274UL, 0xd6c7c9fa8UL, 0xd6da7b8dfUL,
                                          0xd6e11f631UL, 0xd6fcad146UL, 0xd60c8cd89UL, 0xd6113eafeUL, 0xd62a5a410UL,
                                          0xd637e8367UL, 0xd64121ebbUL, 0xd65c939ccUL, 0xd667f7722UL, 0xd67a45055UL,
                                          0xd581a6d87UL, 0xd59c14af0UL, 0xd5a77041eUL, 0xd5bac2369UL, 0xd5cc0beb5UL,
                                          0xd5d1b99c2UL, 0xd5eadd72cUL, 0xd5f76f05bUL, 0xd5074ec94UL, 0xd51afcbe3UL,
                                          0xd5219850dUL, 0xd53c2a27aUL, 0xd54ae3fa6UL, 0xd557518d1UL, 0xd56c3563fUL,
                                          0xd57187148UL, 0xd48c76fa1UL, 0xd491c48d6UL, 0xd4aaa0638UL, 0xd4b71214fUL,
                                          0xd4c1dbc93UL, 0xd4dc69be4UL, 0xd4e70d50aUL, 0xd4fabf27dUL, 0xd40a9eeb2UL,
                                          0xd4172c9c5UL, 0xd42c4872bUL, 0xd431fa05cUL, 0xd44733d80UL, 0xd45a81af7UL,
                                          0xd461e5419UL, 0xd47c5736eUL, 0xd38b908caUL, 0xd39622fbdUL, 0xd3ad46153UL,
                                          0xd3b0f4624UL, 0xd3c63dbf8UL, 0xd3db8fc8fUL, 0xd3e0eb261UL, 0xd3fd59516UL,
                                          0xd30d789d9UL, 0xd310caeaeUL, 0xd32bae040UL, 0xd3361c737UL, 0xd340d5aebUL,
                                          0xd35d67d9cUL, 0xd36603372UL, 0xd37bb1405UL, 0xd28640aecUL, 0xd29bf2d9bUL,
                                          0xd2a096375UL, 0xd2bd24402UL, 0xd2cbed9deUL, 0xd2d65fea9UL, 0xd2ed3b047UL,
                                          0xd2f089730UL, 0xd200a8bffUL, 0xd21d1ac88UL, 0xd2267e266UL, 0xd23bcc511UL,
                                          0xd24d058cdUL, 0xd250b7fbaUL, 0xd26bd3154UL, 0xd27661623UL, 0xd18d82bf1UL,
                                          0xd19030c86UL, 0xd1ab54268UL, 0xd1b6e651fUL, 0xd1c02f8c3UL, 0xd1dd9dfb4UL,
                                          0xd1e6f915aUL, 0xd1fb4b62dUL, 0xd10b6aae2UL, 0xd116d8d95UL, 0xd12dbc37bUL,
                                          0xd1300e40cUL, 0xd146c79d0UL, 0xd15b75ea7UL, 0xd16011049UL, 0xd17da373eUL,
                                          0xd080529d7UL, 0xd09de0ea0UL, 0xd0a68404eUL, 0xd0bb36739UL, 0xd0cdffae5UL,
                                          0xd0d04dd92UL, 0xd0eb2937cUL, 0xd0f69b40bUL, 0xd006ba8c4UL, 0xd01b08fb3UL,
                                          0xd0206c15dUL, 0xd03dde62aUL, 0xd04b17bf6UL, 0xd056a5c81UL, 0xd06dc126fUL,
                                          0xd07073518UL, 0xef83e3b0aUL, 0xef9e51c7dUL, 0xefa535293UL, 0xefb8875e4UL,
                                          0xefce4e838UL, 0xefd3fcf4fUL, 0xefe8981a1UL, 0xeff52a6d6UL, 0xef050ba19UL,
                                          0xef18b9d6eUL, 0xef23dd380UL, 0xef3e6f4f7UL, 0xef48a692bUL, 0xef5514e5cUL,
                                          0xef6e700b2UL, 0xef73c27c5UL, 0xee8e3392cUL, 0xee9381e5bUL, 0xeea8e50b5UL,
                                          0xeeb5577c2UL, 0xeec39ea1eUL, 0xeede2cd69UL, 0xeee548387UL, 0xeef8fa4f0UL,
                                          0xee08db83fUL, 0xee1569f48UL, 0xee2e0d1a6UL, 0xee33bf6d1UL, 0xee4576b0dUL,
                                          0xee58c4c7aUL, 0xee63a0294UL, 0xee7e125e3UL, 0xed85f1831UL, 0xed9843f46UL,
                                          0xeda3271a8UL, 0xedbe956dfUL, 0xedc85cb03UL, 0xedd5eec74UL, 0xedee8a29aUL,
                                          0xedf3385edUL, 0xed0319922UL, 0xed1eabe55UL, 0xed25cf0bbUL, 0xed387d7ccUL,
                                          0xed4eb4a10UL, 0xed5306d67UL, 0xed6862389UL, 0xed75d04feUL, 0xec8821a17UL,
                                          0xec9593d60UL, 0xecaef738eUL, 0xecb3454f9UL, 0xecc58c925UL, 0xecd83ee52UL,
                                          0xece35a0bcUL, 0xecfee87cbUL, 0xec0ec9b04UL, 0xec137bc73UL, 0xec281f29dUL,
                                          0xec35ad5eaUL, 0xec4364836UL, 0xec5ed6f41UL, 0xec65b21afUL, 0xec78006d8UL,
                                          0xeb8fc7d7cUL, 0xeb9275a0bUL, 0xeba9114e5UL, 0xebb4a3392UL, 0xebc26ae4eUL,
                                          0xebdfd8939UL, 0xebe4bc7d7UL, 0xebf90e0a0UL, 0xeb092fc6fUL, 0xeb149db18UL,
                                          0xeb2ff95f6UL, 0xeb324b281UL, 0xeb4482f5dUL, 0xeb593082aUL, 0xeb62546c4UL,
                                          0xeb7fe61b3UL, 0xea8217f5aUL, 0xea9fa582dUL, 0xeaa4c16c3UL, 0xeab9731b4UL,
                                          0xeacfbac68UL, 0xead208b1fUL, 0xeae96c5f1UL, 0xeaf4de286UL, 0xea04ffe49UL,
                                          0xea194d93eUL, 0xea22297d0UL, 0xea3f9b0a7UL, 0xea4952d7bUL, 0xea54e0a0cUL,
                                          0xea6f844e2UL, 0xea7236395UL, 0xe989d5e47UL, 0xe99467930UL, 0xe9af037deUL,
                                          0xe9b2b10a9UL, 0xe9c478d75UL, 0xe9d9caa02UL, 0xe9e2ae4ecUL, 0xe9ff1c39bUL,
                                          0xe90f3df54UL, 0xe9128f823UL, 0xe929eb6cdUL, 0xe934591baUL, 0xe94290c66UL,
                                          0xe95f22b11UL, 0xe964465ffUL, 0xe979f4288UL, 0xe88405c61UL, 0xe899b7b16UL,
                                          0xe8a2d35f8UL, 0xe8bf6128fUL, 0xe8c9a8f53UL, 0xe8d41a824UL, 0xe8ef7e6caUL,
                                          0xe8f2cc1bdUL, 0xe802edd72UL, 0xe81f5fa05UL, 0xe8243b4ebUL, 0xe8398939cUL,
                                          0xe84f40e40UL, 0xe852f2937UL, 0xe869967d9UL, 0xe874240aeUL, 0xe78619091UL,
                                          0xe79bab7e6UL, 0xe7a0cf908UL, 0xe7bd7de7fUL, 0xe7cbb43a3UL, 0xe7d6064d4UL,
                                          0xe7ed62a3aUL, 0xe7f0d0d4dUL, 0xe700f1182UL, 0xe71d436f5UL, 0xe7262781bUL,
                                          0xe73b95f6cUL, 0xe74d5c2b0UL, 0xe750ee5c7UL, 0xe76b8ab29UL, 0xe77638c5eUL,
                                          0xe68bc92b7UL, 0xe6967b5c0UL, 0xe6ad1fb2eUL, 0xe6b0adc59UL, 0xe6c664185UL,
                                          0xe6dbd66f2UL, 0xe6e0b281cUL, 0xe6fd00f6bUL, 0xe60d213a4UL, 0xe610934d3UL,
                                          0xe62bf7a3dUL, 0xe63645d4aUL, 0xe6408c096UL, 0xe65d3e7e1UL, 0xe6665a90fUL,
                                          0xe67be8e78UL, 0xe5800b3aaUL, 0xe59db94ddUL, 0xe5a6dda33UL, 0xe5bb6fd44UL,
                                          0xe5cda6098UL, 0xe5d0147efUL, 0xe5eb70901UL, 0xe5f6c2e76UL, 0xe506e32b9UL,
                                          0xe51b515ceUL, 0xe52035b20UL, 0xe53d87c57UL, 0xe54b4e18bUL, 0xe556fc6fcUL,
                                          0xe56d98812UL, 0xe5702af65UL, 0xe48ddb18cUL, 0xe490696fbUL, 0xe4ab0d815UL,
                                          0xe4b6bff62UL, 0xe4c0762beUL, 0xe4ddc45c9UL, 0xe4e6a0b27UL, 0xe4fb12c50UL,
                                          0xe40b3309fUL, 0xe416817e8UL, 0xe42de5906UL, 0xe43057e71UL, 0xe4469e3adUL,
                                          0xe45b2c4daUL, 0xe46048a34UL, 0xe47dfad43UL, 0xe38a3d6e7UL, 0xe3978f190UL,
                                          0xe3acebf7eUL, 0xe3b159809UL, 0xe3c7905d5UL, 0xe3da222a2UL, 0xe3e146c4cUL,
                                          0xe3fcf4b3bUL, 0xe30cd57f4UL, 0xe31167083UL, 0xe32a03e6dUL, 0xe337b191aUL,
                                          0xe341784c6UL, 0xe35cca3b1UL, 0xe367aed5fUL, 0xe37a1ca28UL, 0xe287ed4c1UL,
                                          0xe29a5f3b6UL, 0xe2a13bd58UL, 0xe2bc89a2fUL, 0xe2ca407f3UL, 0xe2d7f2084UL,
                                          0xe2ec96e6aUL, 0xe2f12491dUL, 0xe201055d2UL, 0xe21cb72a5UL, 0xe227d3c4bUL,
                                          0xe23a61b3cUL, 0xe24ca86e0UL, 0xe2511a197UL, 0xe26a7ef79UL, 0xe277cc80eUL,
                                          0xe18c2f5dcUL, 0xe1919d2abUL, 0xe1aaf9c45UL, 0xe1b74bb32UL, 0xe1c1826eeUL,
                                          0xe1dc30199UL, 0xe1e754f77UL, 0xe1fae6800UL, 0xe10ac74cfUL, 0xe117753b8UL,
                                          0xe12c11d56UL, 0xe131a3a21UL, 0xe1476a7fdUL, 0xe15ad808aUL, 0xe161bce64UL,
                                          0xe17c0e913UL, 0xe081ff7faUL, 0xe09c4d08dUL, 0xe0a729e63UL, 0xe0ba9b914UL,
                                          0xe0cc524c8UL, 0xe0d1e03bfUL, 0xe0ea84d51UL, 0xe0f736a26UL, 0xe007176e9UL,
                                          0xe01aa519eUL, 0xe021c1f70UL, 0xe03c73807UL, 0xe04aba5dbUL, 0xe057082acUL,
                                          0xe06c6cc42UL, 0xe071deb35UL, 0xff8816c3cUL, 0xff95a4b4bUL, 0xffaec05a5UL,
                                          0xffb3722d2UL, 0xffc5bbf0eUL, 0xffd809879UL, 0xffe36d697UL, 0xfffedf1e0UL,
                                          0xff0efed2fUL, 0xff134ca58UL, 0xff28284b6UL, 0xff359a3c1UL, 0xff4353e1dUL,
                                          0xff5ee196aUL, 0xff6585784UL, 0xff78370f3UL, 0xfe85c6e1aUL, 0xfe987496dUL,
                                          0xfea310783UL, 0xfebea20f4UL, 0xfec86bd28UL, 0xfed5d9a5fUL, 0xfeeebd4b1UL,
                                          0xfef30f3c6UL, 0xfe032ef09UL, 0xfe1e9c87eUL, 0xfe25f8690UL, 0xfe384a1e7UL,
                                          0xfe4e83c3bUL, 0xfe5331b4cUL, 0xfe68555a2UL, 0xfe75e72d5UL, 0xfd8e04f07UL,
                                          0xfd93b6870UL, 0xfda8d269eUL, 0xfdb5601e9UL, 0xfdc3a9c35UL, 0xfdde1bb42UL,
                                          0xfde57f5acUL, 0xfdf8cd2dbUL, 0xfd08ece14UL, 0xfd155e963UL, 0xfd2e3a78dUL,
                                          0xfd33880faUL, 0xfd4541d26UL, 0xfd58f3a51UL, 0xfd63974bfUL, 0xfd7e253c8UL,
                                          0xfc83d4d21UL, 0xfc9e66a56UL, 0xfca5024b8UL, 0xfcb8b03cfUL, 0xfcce79e13UL,
                                          0xfcd3cb964UL, 0xfce8af78aUL, 0xfcf51d0fdUL, 0xfc053cc32UL, 0xfc188eb45UL,
                                          0xfc23ea5abUL, 0xfc3e582dcUL, 0xfc4891f00UL, 0xfc5523877UL, 0xfc6e47699UL,
                                          0xfc73f51eeUL, 0xfb8432a4aUL, 0xfb9980d3dUL, 0xfba2e43d3UL, 0xfbbf564a4UL,
                                          0xfbc99f978UL, 0xfbd42de0fUL, 0xfbef490e1UL, 0xfbf2fb796UL, 0xfb02dab59UL,
                                          0xfb1f68c2eUL, 0xfb240c2c0UL, 0xfb39be5b7UL, 0xfb4f7786bUL, 0xfb52c5f1cUL,
                                          0xfb69a11f2UL, 0xfb7413685UL, 0xfa89e286cUL, 0xfa9450f1bUL, 0xfaaf341f5UL,
                                          0xfab286682UL, 0xfac44fb5eUL, 0xfad9fdc29UL, 0xfae2992c7UL, 0xfaff2b5b0UL,
                                          0xfa0f0a97fUL, 0xfa12b8e08UL, 0xfa29dc0e6UL, 0xfa346e791UL, 0xfa42a7a4dUL,
                                          0xfa5f15d3aUL, 0xfa64713d4UL, 0xfa79c34a3UL, 0xf98220971UL, 0xf99f92e06UL,
                                          0xf9a4f60e8UL, 0xf9b94479fUL, 0xf9cf8da43UL, 0xf9d23fd34UL, 0xf9e95b3daUL,
                                          0xf9f4e94adUL, 0xf904c8862UL, 0xf9197af15UL, 0xf9221e1fbUL, 0xf93fac68cUL,
                                          0xf94965b50UL, 0xf954d7c27UL, 0xf96fb32c9UL, 0xf972015beUL, 0xf88ff0b57UL,
                                          0xf89242c20UL, 0xf8a9262ceUL, 0xf8b4945b9UL, 0xf8c25d865UL, 0xf8dfeff12UL,
                                          0xf8e48b1fcUL, 0xf8f93968bUL, 0xf80918a44UL, 0xf814aad33UL, 0xf82fce3ddUL,
                                          0xf8327c4aaUL, 0xf844b5976UL, 0xf85907e01UL, 0xf862630efUL, 0xf87fd1798UL,
                                          0xf78dec7a7UL, 0xf7905e0d0UL, 0xf7ab3ae3eUL, 0xf7b688949UL, 0xf7c041495UL,
                                          0xf7ddf33e2UL, 0xf7e697d0cUL, 0xf7fb25a7bUL, 0xf70b046b4UL, 0xf716b61c3UL,
                                          0xf72dd2f2dUL, 0xf7306085aUL, 0xf746a9586UL, 0xf75b1b2f1UL, 0xf7607fc1fUL,
                                          0xf77dcdb68UL, 0xf6803c581UL, 0xf69d8e2f6UL, 0xf6a6eac18UL, 0xf6bb58b6fUL,
                                          0xf6cd916b3UL, 0xf6d0231c4UL, 0xf6eb47f2aUL, 0xf6f6f585dUL, 0xf606d4492UL,
                                          0xf61b663e5UL, 0xf62002d0bUL, 0xf63db0a7cUL, 0xf64b797a0UL, 0xf656cb0d7UL,
                                          0xf66dafe39UL, 0xf6701d94eUL, 0xf58bfe49cUL, 0xf5964c3ebUL, 0xf5ad28d05UL,
                                          0xf5b09aa72UL, 0xf5c6537aeUL, 0xf5dbe10d9UL, 0xf5e085e37UL, 0xf5fd37940UL,
                                          0xf50d1658fUL, 0xf510a42f8UL, 0xf52bc0c16UL, 0xf53672b61UL, 0xf540bb6bdUL,
                                          0xf55d091caUL, 0xf5666df24UL, 0xf57bdf853UL, 0xf4862e6baUL, 0xf49b9c1cdUL,
                                          0xf4a0f8f23UL, 0xf4bd4a854UL, 0xf4cb83588UL, 0xf4d6312ffUL, 0xf4ed55c11UL,
                                          0xf4f0e7b66UL, 0xf400c67a9UL, 0xf41d740deUL, 0xf42610e30UL, 0xf43ba2947UL,
                                          0xf44d6b49bUL, 0xf450d93ecUL, 0xf46bbdd02UL, 0xf4760fa75UL, 0xf381c81d1UL,
                                          0xf39c7a6a6UL, 0xf3a71e848UL, 0xf3baacf3fUL, 0xf3cc652e3UL, 0xf3d1d7594UL,
                                          0xf3eab3b7aUL, 0xf3f701c0dUL, 0xf307200c2UL, 0xf31a927b5UL, 0xf321f695bUL,
                                          0xf33c44e2cUL, 0xf34a8d3f0UL, 0xf3573f487UL, 0xf36c5ba69UL, 0xf371e9d1eUL,
                                          0xf28c183f7UL, 0xf291aa480UL, 0xf2aacea6eUL, 0xf2b77cd19UL, 0xf2c1b50c5UL,
                                          0xf2dc077b2UL, 0xf2e76395cUL, 0xf2fad1e2bUL, 0xf20af02e4UL, 0xf21742593UL,
                                          0xf22c26b7dUL, 0xf23194c0aUL, 0xf2475d1d6UL, 0xf25aef6a1UL, 0xf2618b84fUL,
                                          0xf27c39f38UL, 0xf187da2eaUL, 0xf19a6859dUL, 0xf1a10cb73UL, 0xf1bcbec04UL,
                                          0xf1ca771d8UL, 0xf1d7c56afUL, 0xf1eca1841UL, 0xf1f113f36UL, 0xf101323f9UL,
                                          0xf11c8048eUL, 0xf127e4a60UL, 0xf13a56d17UL, 0xf14c9f0cbUL, 0xf1512d7bcUL,
                                          0xf16a49952UL, 0xf177fbe25UL, 0xf08a0a0ccUL, 0xf097b87bbUL, 0xf0acdc955UL,
                                          0xf0b16ee22UL, 0xf0c7a73feUL, 0xf0da15489UL, 0xf0e171a67UL, 0xf0fcc3d10UL,
                                          0xf00ce21dfUL, 0xf011506a8UL, 0xf02a34846UL, 0xf03786f31UL, 0xf0414f2edUL,
                                          0xf05cfd59aUL, 0xf06799b74UL, 0xf07a2bc03UL, 0xf8f0caa5UL, 0xf92bedd2UL,
                                          0xfa9da33cUL, 0xfb46844bUL, 0xfc2a1997UL, 0xfdf13ee0UL, 0xfe47700eUL,
                                          0xff9c5779UL, 0xf09e4bb6UL, 0xf1456cc1UL, 0xf2f3222fUL, 0xf3280558UL,
                                          0xf4449884UL, 0xf59fbff3UL, 0xf629f11dUL, 0xf7f2d66aUL, 0xe82dc883UL,
                                          0xe9f6eff4UL, 0xea40a11aUL, 0xeb9b866dUL, 0xecf71bb1UL, 0xed2c3cc6UL,
                                          0xee9a7228UL, 0xef41555fUL, 0xe0434990UL, 0xe1986ee7UL, 0xe22e2009UL,
                                          0xe3f5077eUL, 0xe4999aa2UL, 0xe542bdd5UL, 0xe6f4f33bUL, 0xe72fd44cUL,
                                          0xd891e99eUL, 0xd94acee9UL, 0xdafc8007UL, 0xdb27a770UL, 0xdc4b3aacUL,
                                          0xdd901ddbUL, 0xde265335UL, 0xdffd7442UL, 0xd0ff688dUL, 0xd1244ffaUL,
                                          0xd2920114UL, 0xd3492663UL, 0xd425bbbfUL, 0xd5fe9cc8UL, 0xd648d226UL,
                                          0xd793f551UL, 0xc84cebb8UL, 0xc997cccfUL, 0xca218221UL, 0xcbfaa556UL,
                                          0xcc96388aUL, 0xcd4d1ffdUL, 0xcefb5113UL, 0xcf207664UL, 0xc0226aabUL,
                                          0xc1f94ddcUL, 0xc24f0332UL, 0xc3942445UL, 0xc4f8b999UL, 0xc5239eeeUL,
                                          0xc695d000UL, 0xc74ef777UL, 0xb8328cd3UL, 0xb9e9aba4UL, 0xba5fe54aUL,
                                          0xbb84c23dUL, 0xbce85fe1UL, 0xbd337896UL, 0xbe853678UL, 0xbf5e110fUL,
                                          0xb05c0dc0UL, 0xb1872ab7UL, 0xb2316459UL, 0xb3ea432eUL, 0xb486def2UL,
                                          0xb55df985UL, 0xb6ebb76bUL, 0xb730901cUL, 0xa8ef8ef5UL, 0xa934a982UL,
                                          0xaa82e76cUL, 0xab59c01bUL, 0xac355dc7UL, 0xadee7ab0UL, 0xae58345eUL,
                                          0xaf831329UL, 0xa0810fe6UL, 0xa15a2891UL, 0xa2ec667fUL, 0xa3374108UL,
                                          0xa45bdcd4UL, 0xa580fba3UL, 0xa636b54dUL, 0xa7ed923aUL, 0x9853afe8UL,
                                          0x9988889fUL, 0x9a3ec671UL, 0x9be5e106UL, 0x9c897cdaUL, 0x9d525badUL,
                                          0x9ee41543UL, 0x9f3f3234UL, 0x903d2efbUL, 0x91e6098cUL, 0x92504762UL,
                                          0x938b6015UL, 0x94e7fdc9UL, 0x953cdabeUL, 0x968a9450UL, 0x9751b327UL,
                                          0x888eadceUL, 0x89558ab9UL, 0x8ae3c457UL, 0x8b38e320UL, 0x8c547efcUL,
                                          0x8d8f598bUL, 0x8e391765UL, 0x8fe23012UL, 0x80e02cddUL, 0x813b0baaUL,
                                          0x828d4544UL, 0x83566233UL, 0x843affefUL, 0x85e1d898UL, 0x86579676UL,
                                          0x878cb101UL, 0x78af613eUL, 0x79744649UL, 0x7ac208a7UL, 0x7b192fd0UL,
                                          0x7c75b20cUL, 0x7dae957bUL, 0x7e18db95UL, 0x7fc3fce2UL, 0x70c1e02dUL,
                                          0x711ac75aUL, 0x72ac89b4UL, 0x7377aec3UL, 0x741b331fUL, 0x75c01468UL,
                                          0x76765a86UL, 0x77ad7df1UL, 0x68726318UL, 0x69a9446fUL, 0x6a1f0a81UL,
                                          0x6bc42df6UL, 0x6ca8b02aUL, 0x6d73975dUL, 0x6ec5d9b3UL, 0x6f1efec4UL,
                                          0x601ce20bUL, 0x61c7c57cUL, 0x62718b92UL, 0x63aaace5UL, 0x64c63139UL,
                                          0x651d164eUL, 0x66ab58a0UL, 0x67707fd7UL, 0x58ce4205UL, 0x59156572UL,
                                          0x5aa32b9cUL, 0x5b780cebUL, 0x5c149137UL, 0x5dcfb640UL, 0x5e79f8aeUL,
                                          0x5fa2dfd9UL, 0x50a0c316UL, 0x517be461UL, 0x52cdaa8fUL, 0x53168df8UL,
                                          0x547a1024UL, 0x55a13753UL, 0x561779bdUL, 0x57cc5ecaUL, 0x48134023UL,
                                          0x49c86754UL, 0x4a7e29baUL, 0x4ba50ecdUL, 0x4cc99311UL, 0x4d12b466UL,
                                          0x4ea4fa88UL, 0x4f7fddffUL, 0x407dc130UL, 0x41a6e647UL, 0x4210a8a9UL,
                                          0x43cb8fdeUL, 0x44a71202UL, 0x457c3575UL, 0x46ca7b9bUL, 0x47115cecUL,
                                          0x386d2748UL, 0x39b6003fUL, 0x3a004ed1UL, 0x3bdb69a6UL, 0x3cb7f47aUL,
                                          0x3d6cd30dUL, 0x3eda9de3UL, 0x3f01ba94UL, 0x3003a65bUL, 0x31d8812cUL,
                                          0x326ecfc2UL, 0x33b5e8b5UL, 0x34d97569UL, 0x3502521eUL, 0x36b41cf0UL,
                                          0x376f3b87UL, 0x28b0256eUL, 0x296b0219UL, 0x2add4cf7UL, 0x2b066b80UL,
                                          0x2c6af65cUL, 0x2db1d12bUL, 0x2e079fc5UL, 0x2fdcb8b2UL, 0x20dea47dUL,
                                          0x2105830aUL, 0x22b3cde4UL, 0x2368ea93UL, 0x2404774fUL, 0x25df5038UL,
                                          0x26691ed6UL, 0x27b239a1UL, 0x180c0473UL, 0x19d72304UL, 0x1a616deaUL,
                                          0x1bba4a9dUL, 0x1cd6d741UL, 0x1d0df036UL, 0x1ebbbed8UL, 0x1f6099afUL,
                                          0x10628560UL, 0x11b9a217UL, 0x120fecf9UL, 0x13d4cb8eUL, 0x14b85652UL,
                                          0x15637125UL, 0x16d53fcbUL, 0x170e18bcUL, 0x8d10655UL, 0x90a2122UL,
                                          0xabc6fccUL, 0xb6748bbUL, 0xc0bd567UL, 0xdd0f210UL, 0xe66bcfeUL, 0xfbd9b89UL,
                                          0xbf8746UL, 0x164a031UL, 0x2d2eedfUL, 0x309c9a8UL, 0x4655474UL, 0x5be7303UL,
                                          0x6083dedUL, 0x7d31a9aUL, 0x1f84f9d93UL, 0x1f994bae4UL, 0x1fa22f40aUL,
                                          0x1fbf9d37dUL, 0x1fc954ea1UL, 0x1fd4e69d6UL, 0x1fef82738UL, 0x1ff23004fUL,
                                          0x1f0211c80UL, 0x1f1fa3bf7UL, 0x1f24c7519UL, 0x1f397526eUL, 0x1f4fbcfb2UL,
                                          0x1f520e8c5UL, 0x1f696a62bUL, 0x1f74d815cUL, 0x1e8929fb5UL, 0x1e949b8c2UL,
                                          0x1eafff62cUL, 0x1eb24d15bUL, 0x1ec484c87UL, 0x1ed936bf0UL, 0x1ee25251eUL,
                                          0x1effe0269UL, 0x1e0fc1ea6UL, 0x1e12739d1UL, 0x1e291773fUL, 0x1e34a5048UL,
                                          0x1e426cd94UL, 0x1e5fdeae3UL, 0x1e64ba40dUL, 0x1e790837aUL, 0x1d82ebea8UL,
                                          0x1d9f599dfUL, 0x1da43d731UL, 0x1db98f046UL, 0x1dcf46d9aUL, 0x1dd2f4aedUL,
                                          0x1de990403UL, 0x1df422374UL, 0x1d0403fbbUL, 0x1d19b18ccUL, 0x1d22d5622UL,
                                          0x1d3f67155UL, 0x1d49aec89UL, 0x1d541cbfeUL, 0x1d6f78510UL, 0x1d72ca267UL,
                                          0x1c8f3bc8eUL, 0x1c9289bf9UL, 0x1ca9ed517UL, 0x1cb45f260UL, 0x1cc296fbcUL,
                                          0x1cdf248cbUL, 0x1ce440625UL, 0x1cf9f2152UL, 0x1c09d3d9dUL, 0x1c1461aeaUL,
                                          0x1c2f05404UL, 0x1c32b7373UL, 0x1c447eeafUL, 0x1c59cc9d8UL, 0x1c62a8736UL,
                                          0x1c7f1a041UL, 0x1b88ddbe5UL, 0x1b956fc92UL, 0x1bae0b27cUL, 0x1bb3b950bUL,
                                          0x1bc5708d7UL, 0x1bd8c2fa0UL, 0x1be3a614eUL, 0x1bfe14639UL, 0x1b0e35af6UL,
                                          0x1b1387d81UL, 0x1b28e336fUL, 0x1b3551418UL, 0x1b43989c4UL, 0x1b5e2aeb3UL,
                                          0x1b654e05dUL, 0x1b78fc72aUL, 0x1a850d9c3UL, 0x1a98bfeb4UL, 0x1aa3db05aUL,
                                          0x1abe6972dUL, 0x1ac8a0af1UL, 0x1ad512d86UL, 0x1aee76368UL, 0x1af3c441fUL,
                                          0x1a03e58d0UL, 0x1a1e57fa7UL, 0x1a2533149UL, 0x1a388163eUL, 0x1a4e48be2UL,
                                          0x1a53fac95UL, 0x1a689e27bUL, 0x1a752c50cUL, 0x198ecf8deUL, 0x19937dfa9UL,
                                          0x19a819147UL, 0x19b5ab630UL, 0x19c362becUL, 0x19ded0c9bUL, 0x19e5b4275UL,
                                          0x19f806502UL, 0x1908279cdUL, 0x191595ebaUL, 0x192ef1054UL, 0x193343723UL,
                                          0x19458aaffUL, 0x195838d88UL, 0x19635c366UL, 0x197eee411UL, 0x18831faf8UL,
                                          0x189eadd8fUL, 0x18a5c9361UL, 0x18b87b416UL, 0x18ceb29caUL, 0x18d300ebdUL,
                                          0x18e864053UL, 0x18f5d6724UL, 0x1805f7bebUL, 0x181845c9cUL, 0x182321272UL,
                                          0x183e93505UL, 0x18485a8d9UL, 0x1855e8faeUL, 0x186e8c140UL, 0x18733e637UL,
                                          0x178103608UL, 0x179cb117fUL, 0x17a7d5f91UL, 0x17ba678e6UL, 0x17ccae53aUL,
                                          0x17d11c24dUL, 0x17ea78ca3UL, 0x17f7cabd4UL, 0x1707eb71bUL, 0x171a5906cUL,
                                          0x17213de82UL, 0x173c8f9f5UL, 0x174a46429UL, 0x1757f435eUL, 0x176c90db0UL,
                                          0x177122ac7UL, 0x168cd342eUL, 0x169161359UL, 0x16aa05db7UL, 0x16b7b7ac0UL,
                                          0x16c17e71cUL, 0x16dccc06bUL, 0x16e7a8e85UL, 0x16fa1a9f2UL, 0x160a3b53dUL,
                                          0x16178924aUL, 0x162cedca4UL, 0x16315fbd3UL, 0x16479660fUL, 0x165a24178UL,
                                          0x166140f96UL, 0x167cf28e1UL, 0x158711533UL, 0x159aa3244UL, 0x15a1c7caaUL,
                                          0x15bc75bddUL, 0x15cabc601UL, 0x15d70e176UL, 0x15ec6af98UL, 0x15f1d88efUL,
                                          0x1501f9420UL, 0x151c4b357UL, 0x15272fdb9UL, 0x153a9daceUL, 0x154c54712UL,
                                          0x1551e6065UL, 0x156a82e8bUL, 0x1577309fcUL, 0x148ac1715UL, 0x149773062UL,
                                          0x14ac17e8cUL, 0x14b1a59fbUL, 0x14c76c427UL, 0x14dade350UL, 0x14e1badbeUL,
                                          0x14fc08ac9UL, 0x140c29606UL, 0x14119b171UL, 0x142afff9fUL, 0x14374d8e8UL,
                                          0x144184534UL, 0x145c36243UL, 0x146752cadUL, 0x147ae0bdaUL, 0x138d2707eUL,
                                          0x139095709UL, 0x13abf19e7UL, 0x13b643e90UL, 0x13c08a34cUL, 0x13dd3843bUL,
                                          0x13e65cad5UL, 0x13fbeeda2UL, 0x130bcf16dUL, 0x13167d61aUL, 0x132d198f4UL,
                                          0x1330abf83UL, 0x13466225fUL, 0x135bd0528UL, 0x1360b4bc6UL, 0x137d06cb1UL,
                                          0x1280f7258UL, 0x129d4552fUL, 0x12a621bc1UL, 0x12bb93cb6UL, 0x12cd5a16aUL,
                                          0x12d0e861dUL, 0x12eb8c8f3UL, 0x12f63ef84UL, 0x12061f34bUL, 0x121bad43cUL,
                                          0x1220c9ad2UL, 0x123d7bda5UL, 0x124bb2079UL, 0x12560070eUL, 0x126d649e0UL,
                                          0x1270d6e97UL, 0x118b35345UL, 0x119687432UL, 0x11ade3adcUL, 0x11b051dabUL,
                                          0x11c698077UL, 0x11db2a700UL, 0x11e04e9eeUL, 0x11fdfce99UL, 0x110ddd256UL,
                                          0x11106f521UL, 0x112b0bbcfUL, 0x1136b9cb8UL, 0x114070164UL, 0x115dc2613UL,
                                          0x1166a68fdUL, 0x117b14f8aUL, 0x1086e5163UL, 0x109b57614UL, 0x10a0338faUL,
                                          0x10bd81f8dUL, 0x10cb48251UL, 0x10d6fa526UL, 0x10ed9ebc8UL, 0x10f02ccbfUL,
                                          0x10000d070UL, 0x101dbf707UL, 0x1026db9e9UL, 0x103b69e9eUL, 0x104da0342UL,
                                          0x105012435UL, 0x106b76adbUL, 0x1076c4dacUL, 0x2f85543beUL, 0x2f98e64c9UL,
                                          0x2fa382a27UL, 0x2fbe30d50UL, 0x2fc8f908cUL, 0x2fd54b7fbUL, 0x2fee2f915UL,
                                          0x2ff39de62UL, 0x2f03bc2adUL, 0x2f1e0e5daUL, 0x2f256ab34UL, 0x2f38d8c43UL,
                                          0x2f4e1119fUL, 0x2f53a36e8UL, 0x2f68c7806UL, 0x2f7575f71UL, 0x2e8884198UL,
                                          0x2e95366efUL, 0x2eae52801UL, 0x2eb3e0f76UL, 0x2ec5292aaUL, 0x2ed89b5ddUL,
                                          0x2ee3ffb33UL, 0x2efe4dc44UL, 0x2e0e6c08bUL, 0x2e13de7fcUL, 0x2e28ba912UL,
                                          0x2e3508e65UL, 0x2e43c13b9UL, 0x2e5e734ceUL, 0x2e6517a20UL, 0x2e78a5d57UL,
                                          0x2d8346085UL, 0x2d9ef47f2UL, 0x2da59091cUL, 0x2db822e6bUL, 0x2dceeb3b7UL,
                                          0x2dd3594c0UL, 0x2de83da2eUL, 0x2df58fd59UL, 0x2d05ae196UL, 0x2d181c6e1UL,
                                          0x2d237880fUL, 0x2d3ecaf78UL, 0x2d48032a4UL, 0x2d55b15d3UL, 0x2d6ed5b3dUL,
                                          0x2d7367c4aUL, 0x2c8e962a3UL, 0x2c93245d4UL, 0x2ca840b3aUL, 0x2cb5f2c4dUL,
                                          0x2cc33b191UL, 0x2cde896e6UL, 0x2ce5ed808UL, 0x2cf85ff7fUL, 0x2c087e3b0UL,
                                          0x2c15cc4c7UL, 0x2c2ea8a29UL, 0x2c331ad5eUL, 0x2c45d3082UL, 0x2c58617f5UL,
                                          0x2c630591bUL, 0x2c7eb7e6cUL, 0x2b89705c8UL, 0x2b94c22bfUL, 0x2bafa6c51UL,
                                          0x2bb214b26UL, 0x2bc4dd6faUL, 0x2bd96f18dUL, 0x2be20bf63UL, 0x2bffb9814UL,
                                          0x2b0f984dbUL, 0x2b122a3acUL, 0x2b294ed42UL, 0x2b34fca35UL, 0x2b42357e9UL,
                                          0x2b5f8709eUL, 0x2b64e3e70UL, 0x2b7951907UL, 0x2a84a07eeUL, 0x2a9912099UL,
                                          0x2aa276e77UL, 0x2abfc4900UL, 0x2ac90d4dcUL, 0x2ad4bf3abUL, 0x2aefdbd45UL,
                                          0x2af269a32UL, 0x2a02486fdUL, 0x2a1ffa18aUL, 0x2a249ef64UL, 0x2a392c813UL,
                                          0x2a4fe55cfUL, 0x2a52572b8UL, 0x2a6933c56UL, 0x2a7481b21UL, 0x298f626f3UL,
                                          0x2992d0184UL, 0x29a9b4f6aUL, 0x29b40681dUL, 0x29c2cf5c1UL, 0x29df7d2b6UL,
                                          0x29e419c58UL, 0x29f9abb2fUL, 0x29098a7e0UL, 0x291438097UL, 0x292f5ce79UL,
                                          0x2932ee90eUL, 0x2944274d2UL, 0x2959953a5UL, 0x2962f1d4bUL, 0x297f43a3cUL,
                                          0x2882b24d5UL, 0x289f003a2UL, 0x28a464d4cUL, 0x28b9d6a3bUL, 0x28cf1f7e7UL,
                                          0x28d2ad090UL, 0x28e9c9e7eUL, 0x28f47b909UL, 0x28045a5c6UL, 0x2819e82b1UL,
                                          0x28228cc5fUL, 0x283f3eb28UL, 0x2849f76f4UL, 0x285445183UL, 0x286f21f6dUL,
                                          0x28729381aUL, 0x2780ae825UL, 0x279d1cf52UL, 0x27a6781bcUL, 0x27bbca6cbUL,
                                          0x27cd03b17UL, 0x27d0b1c60UL, 0x27ebd528eUL, 0x27f6675f9UL, 0x270646936UL,
                                          0x271bf4e41UL, 0x2720900afUL, 0x273d227d8UL, 0x274beba04UL, 0x275659d73UL,
                                          0x276d3d39dUL, 0x27708f4eaUL, 0x268d7ea03UL, 0x2690ccd74UL, 0x26aba839aUL,
                                          0x26b61a4edUL, 0x26c0d3931UL, 0x26dd61e46UL, 0x26e6050a8UL, 0x26fbb77dfUL,
                                          0x260b96b10UL, 0x261624c67UL, 0x262d40289UL, 0x2630f25feUL, 0x26463b822UL,
                                          0x265b89f55UL, 0x2660ed1bbUL, 0x267d5f6ccUL, 0x2586bcb1eUL, 0x259b0ec69UL,
                                          0x25a06a287UL, 0x25bdd85f0UL, 0x25cb1182cUL, 0x25d6a3f5bUL, 0x25edc71b5UL,
                                          0x25f0756c2UL, 0x250054a0dUL, 0x251de6d7aUL, 0x252682394UL, 0x253b304e3UL,
                                          0x254df993fUL, 0x25504be48UL, 0x256b2f0a6UL, 0x25769d7d1UL, 0x248b6c938UL,
                                          0x2496dee4fUL, 0x24adba0a1UL, 0x24b0087d6UL, 0x24c6c1a0aUL, 0x24db73d7dUL,
                                          0x24e017393UL, 0x24fda54e4UL, 0x240d8482bUL, 0x241036f5cUL, 0x242b521b2UL,
                                          0x2436e06c5UL, 0x244029b19UL, 0x245d9bc6eUL, 0x2466ff280UL, 0x247b4d5f7UL,
                                          0x238c8ae53UL, 0x239138924UL, 0x23aa5c7caUL, 0x23b7ee0bdUL, 0x23c127d61UL,
                                          0x23dc95a16UL, 0x23e7f14f8UL, 0x23fa4338fUL, 0x230a62f40UL, 0x2317d0837UL,
                                          0x232cb46d9UL, 0x2331061aeUL, 0x2347cfc72UL, 0x235a7db05UL, 0x2361195ebUL,
                                          0x237cab29cUL, 0x22815ac75UL, 0x229ce8b02UL, 0x22a78c5ecUL, 0x22ba3e29bUL,
                                          0x22ccf7f47UL, 0x22d145830UL, 0x22ea216deUL, 0x22f7931a9UL, 0x2207b2d66UL,
                                          0x221a00a11UL, 0x2221644ffUL, 0x223cd6388UL, 0x224a1fe54UL, 0x2257ad923UL,
                                          0x226cc97cdUL, 0x22717b0baUL, 0x218a98d68UL, 0x21972aa1fUL, 0x21ac4e4f1UL,
                                          0x21b1fc386UL, 0x21c735e5aUL, 0x21da8792dUL, 0x21e1e37c3UL, 0x21fc510b4UL,
                                          0x210c70c7bUL, 0x2111c2b0cUL, 0x212aa65e2UL, 0x213714295UL, 0x2141ddf49UL,
                                          0x215c6f83eUL, 0x21670b6d0UL, 0x217ab91a7UL, 0x208748f4eUL, 0x209afa839UL,
                                          0x20a19e6d7UL, 0x20bc2c1a0UL, 0x20cae5c7cUL, 0x20d757b0bUL, 0x20ec335e5UL,
                                          0x20f181292UL, 0x2001a0e5dUL, 0x201c1292aUL, 0x2027767c4UL, 0x203ac40b3UL,
                                          0x204c0dd6fUL, 0x2051bfa18UL, 0x206adb4f6UL, 0x207769381UL, 0x3f8ea1488UL,
                                          0x3f93133ffUL, 0x3fa877d11UL, 0x3fb5c5a66UL, 0x3fc30c7baUL, 0x3fdebe0cdUL,
                                          0x3fe5dae23UL, 0x3ff868954UL, 0x3f084959bUL, 0x3f15fb2ecUL, 0x3f2e9fc02UL,
                                          0x3f332db75UL, 0x3f45e46a9UL, 0x3f58561deUL, 0x3f6332f30UL, 0x3f7e80847UL,
                                          0x3e83716aeUL, 0x3e9ec31d9UL, 0x3ea5a7f37UL, 0x3eb815840UL, 0x3ecedc59cUL,
                                          0x3ed36e2ebUL, 0x3ee80ac05UL, 0x3ef5b8b72UL, 0x3e05997bdUL, 0x3e182b0caUL,
                                          0x3e234fe24UL, 0x3e3efd953UL, 0x3e483448fUL, 0x3e55863f8UL, 0x3e6ee2d16UL,
                                          0x3e7350a61UL, 0x3d88b37b3UL, 0x3d95010c4UL, 0x3dae65e2aUL, 0x3db3d795dUL,
                                          0x3dc51e481UL, 0x3dd8ac3f6UL, 0x3de3c8d18UL, 0x3dfe7aa6fUL, 0x3d0e5b6a0UL,
                                          0x3d13e91d7UL, 0x3d288df39UL, 0x3d353f84eUL, 0x3d43f6592UL, 0x3d5e442e5UL,
                                          0x3d6520c0bUL, 0x3d7892b7cUL, 0x3c8563595UL, 0x3c98d12e2UL, 0x3ca3b5c0cUL,
                                          0x3cbe07b7bUL, 0x3cc8ce6a7UL, 0x3cd57c1d0UL, 0x3cee18f3eUL, 0x3cf3aa849UL,
                                          0x3c038b486UL, 0x3c1e393f1UL, 0x3c255dd1fUL, 0x3c38efa68UL, 0x3c4e267b4UL,
                                          0x3c53940c3UL, 0x3c68f0e2dUL, 0x3c754295aUL, 0x3b82852feUL, 0x3b9f37589UL,
                                          0x3ba453b67UL, 0x3bb9e1c10UL, 0x3bcf281ccUL, 0x3bd29a6bbUL, 0x3be9fe855UL,
                                          0x3bf44cf22UL, 0x3b046d3edUL, 0x3b19df49aUL, 0x3b22bba74UL, 0x3b3f09d03UL,
                                          0x3b49c00dfUL, 0x3b54727a8UL, 0x3b6f16946UL, 0x3b72a4e31UL, 0x3a8f550d8UL,
                                          0x3a92e77afUL, 0x3aa983941UL, 0x3ab431e36UL, 0x3ac2f83eaUL, 0x3adf4a49dUL,
                                          0x3ae42ea73UL, 0x3af99cd04UL, 0x3a09bd1cbUL, 0x3a140f6bcUL, 0x3a2f6b852UL,
                                          0x3a32d9f25UL, 0x3a44102f9UL, 0x3a59a258eUL, 0x3a62c6b60UL, 0x3a7f74c17UL,
                                          0x3984971c5UL, 0x3999256b2UL, 0x39a24185cUL, 0x39bff3f2bUL, 0x39c93a2f7UL,
                                          0x39d488580UL, 0x39efecb6eUL, 0x39f25ec19UL, 0x39027f0d6UL, 0x391fcd7a1UL,
                                          0x3924a994fUL, 0x39391be38UL, 0x394fd23e4UL, 0x395260493UL, 0x396904a7dUL,
                                          0x3974b6d0aUL, 0x3889473e3UL, 0x3894f5494UL, 0x38af91a7aUL, 0x38b223d0dUL,
                                          0x38c4ea0d1UL, 0x38d9587a6UL, 0x38e23c948UL, 0x38ff8ee3fUL, 0x380faf2f0UL,
                                          0x38121d587UL, 0x382979b69UL, 0x3834cbc1eUL, 0x3842021c2UL, 0x385fb06b5UL,
                                          0x3864d485bUL, 0x387966f2cUL, 0x378b5bf13UL, 0x3796e9864UL, 0x37ad8d68aUL,
                                          0x37b03f1fdUL, 0x37c6f6c21UL, 0x37db44b56UL, 0x37e0205b8UL, 0x37fd922cfUL,
                                          0x370db3e00UL, 0x371001977UL, 0x372b65799UL, 0x3736d70eeUL, 0x37401ed32UL,
                                          0x375daca45UL, 0x3766c84abUL, 0x377b7a3dcUL, 0x36868bd35UL, 0x369b39a42UL,
                                          0x36a05d4acUL, 0x36bdef3dbUL, 0x36cb26e07UL, 0x36d694970UL, 0x36edf079eUL,
                                          0x36f0420e9UL, 0x360063c26UL, 0x361dd1b51UL, 0x3626b55bfUL, 0x363b072c8UL,
                                          0x364dcef14UL, 0x36507c863UL, 0x366b1868dUL, 0x3676aa1faUL, 0x358d49c28UL,
                                          0x3590fbb5fUL, 0x35ab9f5b1UL, 0x35b62d2c6UL, 0x35c0e4f1aUL, 0x35dd5686dUL,
                                          0x35e632683UL, 0x35fb801f4UL, 0x350ba1d3bUL, 0x351613a4cUL, 0x352d774a2UL,
                                          0x3530c53d5UL, 0x35460ce09UL, 0x355bbe97eUL, 0x3560da790UL, 0x357d680e7UL,
                                          0x348099e0eUL, 0x349d2b979UL, 0x34a64f797UL, 0x34bbfd0e0UL, 0x34cd34d3cUL,
                                          0x34d086a4bUL, 0x34ebe24a5UL, 0x34f6503d2UL, 0x340671f1dUL, 0x341bc386aUL,
                                          0x3420a7684UL, 0x343d151f3UL, 0x344bdcc2fUL, 0x34566eb58UL, 0x346d0a5b6UL,
                                          0x3470b82c1UL, 0x33877f965UL, 0x339acde12UL, 0x33a1a90fcUL, 0x33bc1b78bUL,
                                          0x33cad2a57UL, 0x33d760d20UL, 0x33ec043ceUL, 0x33f1b64b9UL, 0x330197876UL,
                                          0x331c25f01UL, 0x3327411efUL, 0x333af3698UL, 0x334c3ab44UL, 0x335188c33UL,
                                          0x336aec2ddUL, 0x33775e5aaUL, 0x328aafb43UL, 0x32971dc34UL, 0x32ac792daUL,
                                          0x32b1cb5adUL, 0x32c702871UL, 0x32dab0f06UL, 0x32e1d41e8UL, 0x32fc6669fUL,
                                          0x320c47a50UL, 0x3211f5d27UL, 0x322a913c9UL, 0x3237234beUL, 0x3241ea962UL,
                                          0x325c58e15UL, 0x32673c0fbUL, 0x327a8e78cUL, 0x31816da5eUL, 0x319cdfd29UL,
                                          0x31a7bb3c7UL, 0x31ba094b0UL, 0x31ccc096cUL, 0x31d172e1bUL, 0x31ea160f5UL,
                                          0x31f7a4782UL, 0x310785b4dUL, 0x311a37c3aUL, 0x3121532d4UL, 0x313ce15a3UL,
                                          0x314a2887fUL, 0x31579af08UL, 0x316cfe1e6UL, 0x31714c691UL, 0x308cbd878UL,
                                          0x30910ff0fUL, 0x30aa6b1e1UL, 0x30b7d9696UL, 0x30c110b4aUL, 0x30dca2c3dUL,
                                          0x30e7c62d3UL, 0x30fa745a4UL, 0x300a5596bUL, 0x3017e7e1cUL, 0x302c830f2UL,
                                          0x303131785UL, 0x3047f8a59UL, 0x305a4ad2eUL, 0x30612e3c0UL, 0x307c9c4b7UL,
                                          0x4f860ffe4UL, 0x4f9bbd893UL, 0x4fa0d967dUL, 0x4fbd6b10aUL, 0x4fcba2cd6UL,
                                          0x4fd610ba1UL, 0x4fed7454fUL, 0x4ff0c6238UL, 0x4f00e7ef7UL, 0x4f1d55980UL,
                                          0x4f263176eUL, 0x4f3b83019UL, 0x4f4d4adc5UL, 0x4f50f8ab2UL, 0x4f6b9c45cUL,
                                          0x4f762e32bUL, 0x4e8bdfdc2UL, 0x4e966dab5UL, 0x4ead0945bUL, 0x4eb0bb32cUL,
                                          0x4ec672ef0UL, 0x4edbc0987UL, 0x4ee0a4769UL, 0x4efd1601eUL, 0x4e0d37cd1UL,
                                          0x4e1085ba6UL, 0x4e2be1548UL, 0x4e365323fUL, 0x4e409afe3UL, 0x4e5d28894UL,
                                          0x4e664c67aUL, 0x4e7bfe10dUL, 0x4d801dcdfUL, 0x4d9dafba8UL, 0x4da6cb546UL,
                                          0x4dbb79231UL, 0x4dcdb0fedUL, 0x4dd00289aUL, 0x4deb66674UL, 0x4df6d4103UL,
                                          0x4d06f5dccUL, 0x4d1b47abbUL, 0x4d2023455UL, 0x4d3d91322UL, 0x4d4b58efeUL,
                                          0x4d56ea989UL, 0x4d6d8e767UL, 0x4d703c010UL, 0x4c8dcdef9UL, 0x4c907f98eUL,
                                          0x4cab1b760UL, 0x4cb6a9017UL, 0x4cc060dcbUL, 0x4cddd2abcUL, 0x4ce6b6452UL,
                                          0x4cfb04325UL, 0x4c0b25feaUL, 0x4c169789dUL, 0x4c2df3673UL, 0x4c3041104UL,
                                          0x4c4688cd8UL, 0x4c5b3abafUL, 0x4c605e541UL, 0x4c7dec236UL, 0x4b8a2b992UL,
                                          0x4b9799ee5UL, 0x4bacfd00bUL, 0x4bb14f77cUL, 0x4bc786aa0UL, 0x4bda34dd7UL,
                                          0x4be150339UL, 0x4bfce244eUL, 0x4b0cc3881UL, 0x4b1171ff6UL, 0x4b2a15118UL,
                                          0x4b37a766fUL, 0x4b416ebb3UL, 0x4b5cdccc4UL, 0x4b67b822aUL, 0x4b7a0a55dUL,
                                          0x4a87fbbb4UL, 0x4a9a49cc3UL, 0x4aa12d22dUL, 0x4abc9f55aUL, 0x4aca56886UL,
                                          0x4ad7e4ff1UL, 0x4aec8011fUL, 0x4af132668UL, 0x4a0113aa7UL, 0x4a1ca1dd0UL,
                                          0x4a27c533eUL, 0x4a3a77449UL, 0x4a4cbe995UL, 0x4a510cee2UL, 0x4a6a6800cUL,
                                          0x4a77da77bUL, 0x498c39aa9UL, 0x49918bddeUL, 0x49aaef330UL, 0x49b75d447UL,
                                          0x49c19499bUL, 0x49dc26eecUL, 0x49e742002UL, 0x49faf0775UL, 0x490ad1bbaUL,
                                          0x491763ccdUL, 0x492c07223UL, 0x4931b5554UL, 0x49477c888UL, 0x495acefffUL,
                                          0x4961aa111UL, 0x497c18666UL, 0x4881e988fUL, 0x489c5bff8UL, 0x48a73f116UL,
                                          0x48ba8d661UL, 0x48cc44bbdUL, 0x48d1f6ccaUL, 0x48ea92224UL, 0x48f720553UL,
                                          0x48070199cUL, 0x481ab3eebUL, 0x4821d7005UL, 0x483c65772UL, 0x484aacaaeUL,
                                          0x48571edd9UL, 0x486c7a337UL, 0x4871c8440UL, 0x4783f547fUL, 0x479e47308UL,
                                          0x47a523de6UL, 0x47b891a91UL, 0x47ce5874dUL, 0x47d3ea03aUL, 0x47e88eed4UL,
                                          0x47f53c9a3UL, 0x47051d56cUL, 0x4718af21bUL, 0x4723cbcf5UL, 0x473e79b82UL,
                                          0x4748b065eUL, 0x475502129UL, 0x476e66fc7UL, 0x4773d48b0UL, 0x468e25659UL,
                                          0x46939712eUL, 0x46a8f3fc0UL, 0x46b5418b7UL, 0x46c38856bUL, 0x46de3a21cUL,
                                          0x46e55ecf2UL, 0x46f8ecb85UL, 0x4608cd74aUL, 0x46157f03dUL, 0x462e1bed3UL,
                                          0x4633a99a4UL, 0x464560478UL, 0x4658d230fUL, 0x4663b6de1UL, 0x467e04a96UL,
                                          0x4585e7744UL, 0x459855033UL, 0x45a331eddUL, 0x45be839aaUL, 0x45c84a476UL,
                                          0x45d5f8301UL, 0x45ee9cdefUL, 0x45f32ea98UL, 0x45030f657UL, 0x451ebd120UL,
                                          0x4525d9fceUL, 0x45386b8b9UL, 0x454ea2565UL, 0x455310212UL, 0x456874cfcUL,
                                          0x4575c6b8bUL, 0x448837562UL, 0x449585215UL, 0x44aee1cfbUL, 0x44b353b8cUL,
                                          0x44c59a650UL, 0x44d828127UL, 0x44e34cfc9UL, 0x44fefe8beUL, 0x440edf471UL,
                                          0x44136d306UL, 0x442809de8UL, 0x4435bba9fUL, 0x444372743UL, 0x445ec0034UL,
                                          0x4465a4edaUL, 0x4478169adUL, 0x438fd1209UL, 0x43926357eUL, 0x43a907b90UL,
                                          0x43b4b5ce7UL, 0x43c27c13bUL, 0x43dfce64cUL, 0x43e4aa8a2UL, 0x43f918fd5UL,
                                          0x43093931aUL, 0x43148b46dUL, 0x432fefa83UL, 0x43325ddf4UL, 0x434494028UL,
                                          0x43592675fUL, 0x4362429b1UL, 0x437ff0ec6UL, 0x42820102fUL, 0x429fb3758UL,
                                          0x42a4d79b6UL, 0x42b965ec1UL, 0x42cfac31dUL, 0x42d21e46aUL, 0x42e97aa84UL,
                                          0x42f4c8df3UL, 0x4204e913cUL, 0x42195b64bUL, 0x42223f8a5UL, 0x423f8dfd2UL,
                                          0x42494420eUL, 0x4254f6579UL, 0x426f92b97UL, 0x427220ce0UL, 0x4189c3132UL,
                                          0x419471645UL, 0x41af158abUL, 0x41b2a7fdcUL, 0x41c46e200UL, 0x41d9dc577UL,
                                          0x41e2b8b99UL, 0x41ff0aceeUL, 0x410f2b021UL, 0x411299756UL, 0x4129fd9b8UL,
                                          0x41344fecfUL, 0x414286313UL, 0x415f34464UL, 0x416450a8aUL, 0x4179e2dfdUL,
                                          0x408413314UL, 0x4099a1463UL, 0x40a2c5a8dUL, 0x40bf77dfaUL, 0x40c9be026UL,
                                          0x40d40c751UL, 0x40ef689bfUL, 0x40f2daec8UL, 0x4002fb207UL, 0x401f49570UL,
                                          0x40242db9eUL, 0x40399fce9UL, 0x404f56135UL, 0x4052e4642UL, 0x4069808acUL,
                                          0x407432fdbUL, 0x5f8dfa8d2UL, 0x5f9048fa5UL, 0x5fab2c14bUL, 0x5fb69e63cUL,
                                          0x5fc057be0UL, 0x5fdde5c97UL, 0x5fe681279UL, 0x5ffb3350eUL, 0x5f0b129c1UL,
                                          0x5f16a0eb6UL, 0x5f2dc4058UL, 0x5f307672fUL, 0x5f46bfaf3UL, 0x5f5b0dd84UL,
                                          0x5f606936aUL, 0x5f7ddb41dUL, 0x5e802aaf4UL, 0x5e9d98d83UL, 0x5ea6fc36dUL,
                                          0x5ebb4e41aUL, 0x5ecd879c6UL, 0x5ed035eb1UL, 0x5eeb5105fUL, 0x5ef6e3728UL,
                                          0x5e06c2be7UL, 0x5e1b70c90UL, 0x5e201427eUL, 0x5e3da6509UL, 0x5e4b6f8d5UL,
                                          0x5e56ddfa2UL, 0x5e6db914cUL, 0x5e700b63bUL, 0x5d8be8be9UL, 0x5d965ac9eUL,
                                          0x5dad3e270UL, 0x5db08c507UL, 0x5dc6458dbUL, 0x5ddbf7facUL, 0x5de093142UL,
                                          0x5dfd21635UL, 0x5d0d00afaUL, 0x5d10b2d8dUL, 0x5d2bd6363UL, 0x5d3664414UL,
                                          0x5d40ad9c8UL, 0x5d5d1febfUL, 0x5d667b051UL, 0x5d7bc9726UL, 0x5c86389cfUL,
                                          0x5c9b8aeb8UL, 0x5ca0ee056UL, 0x5cbd5c721UL, 0x5ccb95afdUL, 0x5cd627d8aUL,
                                          0x5ced43364UL, 0x5cf0f1413UL, 0x5c00d08dcUL, 0x5c1d62fabUL, 0x5c2606145UL,
                                          0x5c3bb4632UL, 0x5c4d7dbeeUL, 0x5c50cfc99UL, 0x5c6bab277UL, 0x5c7619500UL,
                                          0x5b81deea4UL, 0x5b9c6c9d3UL, 0x5ba70873dUL, 0x5bbaba04aUL, 0x5bcc73d96UL,
                                          0x5bd1c1ae1UL, 0x5beaa540fUL, 0x5bf717378UL, 0x5b0736fb7UL, 0x5b1a848c0UL,
                                          0x5b21e062eUL, 0x5b3c52159UL, 0x5b4a9bc85UL, 0x5b5729bf2UL, 0x5b6c4d51cUL,
                                          0x5b71ff26bUL, 0x5a8c0ec82UL, 0x5a91bcbf5UL, 0x5aaad851bUL, 0x5ab76a26cUL,
                                          0x5ac1a3fb0UL, 0x5adc118c7UL, 0x5ae775629UL, 0x5afac715eUL, 0x5a0ae6d91UL,
                                          0x5a1754ae6UL, 0x5a2c30408UL, 0x5a318237fUL, 0x5a474bea3UL, 0x5a5af99d4UL,
                                          0x5a619d73aUL, 0x5a7c2f04dUL, 0x5987ccd9fUL, 0x599a7eae8UL, 0x59a11a406UL,
                                          0x59bca8371UL, 0x59ca61eadUL, 0x59d7d39daUL, 0x59ecb7734UL, 0x59f105043UL,
                                          0x590124c8cUL, 0x591c96bfbUL, 0x5927f2515UL, 0x593a40262UL, 0x594c89fbeUL,
                                          0x59513b8c9UL, 0x596a5f627UL, 0x5977ed150UL, 0x588a1cfb9UL, 0x5897ae8ceUL,
                                          0x58acca620UL, 0x58b178157UL, 0x58c7b1c8bUL, 0x58da03bfcUL, 0x58e167512UL,
                                          0x58fcd5265UL, 0x580cf4eaaUL, 0x5811469ddUL, 0x582a22733UL, 0x583790044UL,
                                          0x584159d98UL, 0x585cebaefUL, 0x58678f401UL, 0x587a3d376UL, 0x578800349UL,
                                          0x5795b243eUL, 0x57aed6ad0UL, 0x57b364da7UL, 0x57c5ad07bUL, 0x57d81f70cUL,
                                          0x57e37b9e2UL, 0x57fec9e95UL, 0x570ee825aUL, 0x57135a52dUL, 0x57283ebc3UL,
                                          0x57358ccb4UL, 0x574345168UL, 0x575ef761fUL, 0x5765938f1UL, 0x577821f86UL,
                                          0x5685d016fUL, 0x569862618UL, 0x56a3068f6UL, 0x56beb4f81UL, 0x56c87d25dUL,
                                          0x56d5cf52aUL, 0x56eeabbc4UL, 0x56f319cb3UL, 0x56033807cUL, 0x561e8a70bUL,
                                          0x5625ee9e5UL, 0x56385ce92UL, 0x564e9534eUL, 0x565327439UL, 0x566843ad7UL,
                                          0x5675f1da0UL, 0x558e12072UL, 0x5593a0705UL, 0x55a8c49ebUL, 0x55b576e9cUL,
                                          0x55c3bf340UL, 0x55de0d437UL, 0x55e569ad9UL, 0x55f8dbdaeUL, 0x5508fa161UL,
                                          0x551548616UL, 0x552e2c8f8UL, 0x55339ef8fUL, 0x554557253UL, 0x5558e5524UL,
                                          0x556381bcaUL, 0x557e33cbdUL, 0x5483c2254UL, 0x549e70523UL, 0x54a514bcdUL,
                                          0x54b8a6cbaUL, 0x54ce6f166UL, 0x54d3dd611UL, 0x54e8b98ffUL, 0x54f50bf88UL,
                                          0x54052a347UL, 0x541898430UL, 0x5423fcadeUL, 0x543e4eda9UL, 0x544887075UL,
                                          0x545535702UL, 0x546e519ecUL, 0x5473e3e9bUL, 0x53842453fUL, 0x539996248UL,
                                          0x53a2f2ca6UL, 0x53bf40bd1UL, 0x53c98960dUL, 0x53d43b17aUL, 0x53ef5ff94UL,
                                          0x53f2ed8e3UL, 0x5302cc42cUL, 0x531f7e35bUL, 0x53241adb5UL, 0x5339a8ac2UL,
                                          0x534f6171eUL, 0x5352d3069UL, 0x5369b7e87UL, 0x5374059f0UL, 0x5289f4719UL,
                                          0x52944606eUL, 0x52af22e80UL, 0x52b2909f7UL, 0x52c45942bUL, 0x52d9eb35cUL,
                                          0x52e28fdb2UL, 0x52ff3dac5UL, 0x520f1c60aUL, 0x5212ae17dUL, 0x5229caf93UL,
                                          0x5234788e4UL, 0x5242b1538UL, 0x525f0324fUL, 0x526467ca1UL, 0x5279d5bd6UL,
                                          0x518236604UL, 0x519f84173UL, 0x51a4e0f9dUL, 0x51b9528eaUL, 0x51cf9b536UL,
                                          0x51d229241UL, 0x51e94dcafUL, 0x51f4ffbd8UL, 0x5104de717UL, 0x51196c060UL,
                                          0x512208e8eUL, 0x513fba9f9UL, 0x514973425UL, 0x5154c1352UL, 0x516fa5dbcUL,
                                          0x517217acbUL, 0x508fe6422UL, 0x509254355UL, 0x50a930dbbUL, 0x50b482accUL,
                                          0x50c24b710UL, 0x50dff9067UL, 0x50e49de89UL, 0x50f92f9feUL, 0x50090e531UL,
                                          0x5014bc246UL, 0x502fd8ca8UL, 0x50326abdfUL, 0x5044a3603UL, 0x505911174UL,
                                          0x506275f9aUL, 0x507fc78edUL, 0x6f8c576ffUL, 0x6f91e5188UL, 0x6faa81f66UL,
                                          0x6fb733811UL, 0x6fc1fa5cdUL, 0x6fdc482baUL, 0x6fe72cc54UL, 0x6ffa9eb23UL,
                                          0x6f0abf7ecUL, 0x6f170d09bUL, 0x6f2c69e75UL, 0x6f31db902UL, 0x6f47124deUL,
                                          0x6f5aa03a9UL, 0x6f61c4d47UL, 0x6f7c76a30UL, 0x6e81874d9UL, 0x6e9c353aeUL,
                                          0x6ea751d40UL, 0x6ebae3a37UL, 0x6ecc2a7ebUL, 0x6ed19809cUL, 0x6eeafce72UL,
                                          0x6ef74e905UL, 0x6e076f5caUL, 0x6e1add2bdUL, 0x6e21b9c53UL, 0x6e3c0bb24UL,
                                          0x6e4ac26f8UL, 0x6e577018fUL, 0x6e6c14f61UL, 0x6e71a6816UL, 0x6d8a455c4UL,
                                          0x6d97f72b3UL, 0x6dac93c5dUL, 0x6db121b2aUL, 0x6dc7e86f6UL, 0x6dda5a181UL,
                                          0x6de13ef6fUL, 0x6dfc8c818UL, 0x6d0cad4d7UL, 0x6d111f3a0UL, 0x6d2a7bd4eUL,
                                          0x6d37c9a39UL, 0x6d41007e5UL, 0x6d5cb2092UL, 0x6d67d6e7cUL, 0x6d7a6490bUL,
                                          0x6c87957e2UL, 0x6c9a27095UL, 0x6ca143e7bUL, 0x6cbcf190cUL, 0x6cca384d0UL,
                                          0x6cd78a3a7UL, 0x6ceceed49UL, 0x6cf15ca3eUL, 0x6c017d6f1UL, 0x6c1ccf186UL,
                                          0x6c27abf68UL, 0x6c3a1981fUL, 0x6c4cd05c3UL, 0x6c51622b4UL, 0x6c6a06c5aUL,
                                          0x6c77b4b2dUL, 0x6b8073089UL, 0x6b9dc17feUL, 0x6ba6a5910UL, 0x6bbb17e67UL,
                                          0x6bcdde3bbUL, 0x6bd06c4ccUL, 0x6beb08a22UL, 0x6bf6bad55UL, 0x6b069b19aUL,
                                          0x6b1b296edUL, 0x6b204d803UL, 0x6b3dfff74UL, 0x6b4b362a8UL, 0x6b56845dfUL,
                                          0x6b6de0b31UL, 0x6b7052c46UL, 0x6a8da32afUL, 0x6a90115d8UL, 0x6aab75b36UL,
                                          0x6ab6c7c41UL, 0x6ac00e19dUL, 0x6addbc6eaUL, 0x6ae6d8804UL, 0x6afb6af73UL,
                                          0x6a0b4b3bcUL, 0x6a16f94cbUL, 0x6a2d9da25UL, 0x6a302fd52UL, 0x6a46e608eUL,
                                          0x6a5b547f9UL, 0x6a6030917UL, 0x6a7d82e60UL, 0x6986613b2UL, 0x699bd34c5UL,
                                          0x69a0b7a2bUL, 0x69bd05d5cUL, 0x69cbcc080UL, 0x69d67e7f7UL, 0x69ed1a919UL,
                                          0x69f0a8e6eUL, 0x6900892a1UL, 0x691d3b5d6UL, 0x69265fb38UL, 0x693bedc4fUL,
                                          0x694d24193UL, 0x6950966e4UL, 0x696bf280aUL, 0x697640f7dUL, 0x688bb1194UL,
                                          0x6896036e3UL, 0x68ad6780dUL, 0x68b0d5f7aUL, 0x68c61c2a6UL, 0x68dbae5d1UL,
                                          0x68e0cab3fUL, 0x68fd78c48UL, 0x680d59087UL, 0x6810eb7f0UL, 0x682b8f91eUL,
                                          0x68363de69UL, 0x6840f43b5UL, 0x685d464c2UL, 0x686622a2cUL, 0x687b90d5bUL,
                                          0x6789add64UL, 0x67941fa13UL, 0x67af7b4fdUL, 0x67b2c938aUL, 0x67c400e56UL,
                                          0x67d9b2921UL, 0x67e2d67cfUL, 0x67ff640b8UL, 0x670f45c77UL, 0x6712f7b00UL,
                                          0x6729935eeUL, 0x673421299UL, 0x6742e8f45UL, 0x675f5a832UL, 0x67643e6dcUL,
                                          0x67798c1abUL, 0x66847df42UL, 0x6699cf835UL, 0x66a2ab6dbUL, 0x66bf191acUL,
                                          0x66c9d0c70UL, 0x66d462b07UL, 0x66ef065e9UL, 0x66f2b429eUL, 0x660295e51UL,
                                          0x661f27926UL, 0x6624437c8UL, 0x6639f10bfUL, 0x664f38d63UL, 0x66528aa14UL,
                                          0x6669ee4faUL, 0x66745c38dUL, 0x658fbfe5fUL, 0x65920d928UL, 0x65a9697c6UL,
                                          0x65b4db0b1UL, 0x65c212d6dUL, 0x65dfa0a1aUL, 0x65e4c44f4UL, 0x65f976383UL,
                                          0x650957f4cUL, 0x6514e583bUL, 0x652f816d5UL, 0x6532331a2UL, 0x6544fac7eUL,
                                          0x655948b09UL, 0x65622c5e7UL, 0x657f9e290UL, 0x64826fc79UL, 0x649fddb0eUL,
                                          0x64a4b95e0UL, 0x64b90b297UL, 0x64cfc2f4bUL, 0x64d27083cUL, 0x64e9146d2UL,
                                          0x64f4a61a5UL, 0x640487d6aUL, 0x641935a1dUL, 0x6422514f3UL, 0x643fe3384UL,
                                          0x64492ae58UL, 0x64549892fUL, 0x646ffc7c1UL, 0x64724e0b6UL, 0x638589b12UL,
                                          0x63983bc65UL, 0x63a35f28bUL, 0x63beed5fcUL, 0x63c824820UL, 0x63d596f57UL,
                                          0x63eef21b9UL, 0x63f3406ceUL, 0x630361a01UL, 0x631ed3d76UL, 0x6325b7398UL,
                                          0x6338054efUL, 0x634ecc933UL, 0x63537ee44UL, 0x63681a0aaUL, 0x6375a87ddUL,
                                          0x628859934UL, 0x6295ebe43UL, 0x62ae8f0adUL, 0x62b33d7daUL, 0x62c5f4a06UL,
                                          0x62d846d71UL, 0x62e32239fUL, 0x62fe904e8UL, 0x620eb1827UL, 0x621303f50UL,
                                          0x6228671beUL, 0x6235d56c9UL, 0x62431cb15UL, 0x625eaec62UL, 0x6265ca28cUL,
                                          0x6278785fbUL, 0x61839b829UL, 0x619e29f5eUL, 0x61a54d1b0UL, 0x61b8ff6c7UL,
                                          0x61ce36b1bUL, 0x61d384c6cUL, 0x61e8e0282UL, 0x61f5525f5UL, 0x61057393aUL,
                                          0x6118c1e4dUL, 0x6123a50a3UL, 0x613e177d4UL, 0x6148dea08UL, 0x61556cd7fUL,
                                          0x616e08391UL, 0x6173ba4e6UL, 0x608e4ba0fUL, 0x6093f9d78UL, 0x60a89d396UL,
                                          0x60b52f4e1UL, 0x60c3e693dUL, 0x60de54e4aUL, 0x60e5300a4UL, 0x60f8827d3UL,
                                          0x6008a3b1cUL, 0x601511c6bUL, 0x602e75285UL, 0x6033c75f2UL, 0x60450e82eUL,
                                          0x6058bcf59UL, 0x6063d81b7UL, 0x607e6a6c0UL, 0x7f87a21c9UL, 0x7f9a106beUL,
                                          0x7fa174850UL, 0x7fbcc6f27UL, 0x7fca0f2fbUL, 0x7fd7bd58cUL, 0x7fecd9b62UL,
                                          0x7ff16bc15UL, 0x7f014a0daUL, 0x7f1cf87adUL, 0x7f279c943UL, 0x7f3a2ee34UL,
                                          0x7f4ce73e8UL, 0x7f515549fUL, 0x7f6a31a71UL, 0x7f7783d06UL, 0x7e8a723efUL,
                                          0x7e97c0498UL, 0x7eaca4a76UL, 0x7eb116d01UL, 0x7ec7df0ddUL, 0x7eda6d7aaUL,
                                          0x7ee109944UL, 0x7efcbbe33UL, 0x7e0c9a2fcUL, 0x7e112858bUL, 0x7e2a4cb65UL,
                                          0x7e37fec12UL, 0x7e41371ceUL, 0x7e5c856b9UL, 0x7e67e1857UL, 0x7e7a53f20UL,
                                          0x7d81b02f2UL, 0x7d9c02585UL, 0x7da766b6bUL, 0x7dbad4c1cUL, 0x7dcc1d1c0UL,
                                          0x7dd1af6b7UL, 0x7deacb859UL, 0x7df779f2eUL, 0x7d07583e1UL, 0x7d1aea496UL,
                                          0x7d218ea78UL, 0x7d3c3cd0fUL, 0x7d4af50d3UL, 0x7d57477a4UL, 0x7d6c2394aUL,
                                          0x7d7191e3dUL, 0x7c8c600d4UL, 0x7c91d27a3UL, 0x7caab694dUL, 0x7cb704e3aUL,
                                          0x7cc1cd3e6UL, 0x7cdc7f491UL, 0x7ce71ba7fUL, 0x7cfaa9d08UL, 0x7c0a881c7UL,
                                          0x7c173a6b0UL, 0x7c2c5e85eUL, 0x7c31ecf29UL, 0x7c47252f5UL, 0x7c5a97582UL,
                                          0x7c61f3b6cUL, 0x7c7c41c1bUL, 0x7b8b867bfUL, 0x7b96340c8UL, 0x7bad50e26UL,
                                          0x7bb0e2951UL, 0x7bc62b48dUL, 0x7bdb993faUL, 0x7be0fdd14UL, 0x7bfd4fa63UL,
                                          0x7b0d6e6acUL, 0x7b10dc1dbUL, 0x7b2bb8f35UL, 0x7b360a842UL, 0x7b40c359eUL,
                                          0x7b5d712e9UL, 0x7b6615c07UL, 0x7b7ba7b70UL, 0x7a8656599UL, 0x7a9be42eeUL,
                                          0x7aa080c00UL, 0x7abd32b77UL, 0x7acbfb6abUL, 0x7ad6491dcUL, 0x7aed2df32UL,
                                          0x7af09f845UL, 0x7a00be48aUL, 0x7a1d0c3fdUL, 0x7a2668d13UL, 0x7a3bdaa64UL,
                                          0x7a4d137b8UL, 0x7a50a10cfUL, 0x7a6bc5e21UL, 0x7a7677956UL, 0x798d94484UL,
                                          0x7990263f3UL, 0x79ab42d1dUL, 0x79b6f0a6aUL, 0x79c0397b6UL, 0x79dd8b0c1UL,
                                          0x79e6efe2fUL, 0x79fb5d958UL, 0x790b7c597UL, 0x7916ce2e0UL, 0x792daac0eUL,
                                          0x793018b79UL, 0x7946d16a5UL, 0x795b631d2UL, 0x796007f3cUL, 0x797db584bUL,
                                          0x7880446a2UL, 0x789df61d5UL, 0x78a692f3bUL, 0x78bb2084cUL, 0x78cde9590UL,
                                          0x78d05b2e7UL, 0x78eb3fc09UL, 0x78f68db7eUL, 0x7806ac7b1UL, 0x781b1e0c6UL,
                                          0x78207ae28UL, 0x783dc895fUL, 0x784b01483UL, 0x7856b33f4UL, 0x786dd7d1aUL,
                                          0x787065a6dUL, 0x778258a52UL, 0x779fead25UL, 0x77a48e3cbUL, 0x77b93c4bcUL,
                                          0x77cff5960UL, 0x77d247e17UL, 0x77e9230f9UL, 0x77f49178eUL, 0x7704b0b41UL,
                                          0x771902c36UL, 0x7722662d8UL, 0x773fd45afUL, 0x77491d873UL, 0x7754aff04UL,
                                          0x776fcb1eaUL, 0x77727969dUL, 0x768f88874UL, 0x76923af03UL, 0x76a95e1edUL,
                                          0x76b4ec69aUL, 0x76c225b46UL, 0x76df97c31UL, 0x76e4f32dfUL, 0x76f9415a8UL,
                                          0x760960967UL, 0x7614d2e10UL, 0x762fb60feUL, 0x763204789UL, 0x7644cda55UL,
                                          0x76597fd22UL, 0x76621b3ccUL, 0x767fa94bbUL, 0x75844a969UL, 0x7599f8e1eUL,
                                          0x75a29c0f0UL, 0x75bf2e787UL, 0x75c9e7a5bUL, 0x75d455d2cUL, 0x75ef313c2UL,
                                          0x75f2834b5UL, 0x7502a287aUL, 0x751f10f0dUL, 0x7524741e3UL, 0x7539c6694UL,
                                          0x754f0fb48UL, 0x7552bdc3fUL, 0x7569d92d1UL, 0x75746b5a6UL, 0x74899ab4fUL,
                                          0x749428c38UL, 0x74af4c2d6UL, 0x74b2fe5a1UL, 0x74c43787dUL, 0x74d985f0aUL,
                                          0x74e2e11e4UL, 0x74ff53693UL, 0x740f72a5cUL, 0x7412c0d2bUL, 0x7429a43c5UL,
                                          0x7434164b2UL, 0x7442df96eUL, 0x745f6de19UL, 0x7464090f7UL, 0x7479bb780UL,
                                          0x738e7cc24UL, 0x7393ceb53UL, 0x73a8aa5bdUL, 0x73b5182caUL, 0x73c3d1f16UL,
                                          0x73de63861UL, 0x73e50768fUL, 0x73f8b51f8UL, 0x730894d37UL, 0x731526a40UL,
                                          0x732e424aeUL, 0x7333f03d9UL, 0x734539e05UL, 0x73588b972UL, 0x7363ef79cUL,
                                          0x737e5d0ebUL, 0x7283ace02UL, 0x729e1e975UL, 0x72a57a79bUL, 0x72b8c80ecUL,
                                          0x72ce01d30UL, 0x72d3b3a47UL, 0x72e8d74a9UL, 0x72f5653deUL, 0x720544f11UL,
                                          0x7218f6866UL, 0x722392688UL, 0x723e201ffUL, 0x7248e9c23UL, 0x72555bb54UL,
                                          0x726e3f5baUL, 0x72738d2cdUL, 0x71886ef1fUL, 0x7195dc868UL, 0x71aeb8686UL,
                                          0x71b30a1f1UL, 0x71c5c3c2dUL, 0x71d871b5aUL, 0x71e3155b4UL, 0x71fea72c3UL,
                                          0x710e86e0cUL, 0x71133497bUL, 0x712850795UL, 0x7135e20e2UL, 0x71432bd3eUL,
                                          0x715e99a49UL, 0x7165fd4a7UL, 0x71784f3d0UL, 0x7085bed39UL, 0x70980ca4eUL,
                                          0x70a3684a0UL, 0x70beda3d7UL, 0x70c813e0bUL, 0x70d5a197cUL, 0x70eec5792UL,
                                          0x70f3770e5UL, 0x700356c2aUL, 0x701ee4b5dUL, 0x7025805b3UL, 0x7038322c4UL,
                                          0x704efbf18UL, 0x70534986fUL, 0x70682d681UL, 0x70759f1f6UL};
      fromVector(codes, d._code_id);
      d._nbits = 36;
      d._tau = 2; //
      d._type = ARTOOLKITPLUSBCH;
      d._name = "ARTOOLKITPLUSBCH";
    }
      break;
    case TAG36h11:
    {
      std::vector<std::uint64_t> codes = {0xd5d628584UL, 0xd97f18b49UL, 0xdd280910eUL, 0xe479e9c98UL, 0xebcbca822UL,
                                          0xf31dab3acUL, 0x56a5d085UL, 0x10652e1d4UL, 0x22b1dfeadUL, 0x265ad0472UL,
                                          0x34fe91b86UL, 0x3ff962cd5UL, 0x43a25329aUL, 0x474b4385fUL, 0x4e9d243e9UL,
                                          0x5246149aeUL, 0x5997f5538UL, 0x683bb6c4cUL, 0x6be4a7211UL, 0x7e3158eeaUL,
                                          0x81da494afUL, 0x858339a74UL, 0x8cd51a5feUL, 0x9f21cc2d7UL, 0xa2cabc89cUL,
                                          0xadc58d9ebUL, 0xb16e7dfb0UL, 0xb8c05eb3aUL, 0xd25ef139dUL, 0xd607e1962UL,
                                          0xe4aba3076UL, 0x2dde6a3daUL, 0x43d40c678UL, 0x5620be351UL, 0x64c47fa65UL,
                                          0x686d7002aUL, 0x6c16605efUL, 0x6fbf50bb4UL, 0x8d06d39dcUL, 0x9f53856b5UL,
                                          0xadf746dc9UL, 0xbc9b084ddUL, 0xd290aa77bUL, 0xd9e28b305UL, 0xe4dd5c454UL,
                                          0xfad2fe6f2UL, 0x181a8151aUL, 0x26be42c2eUL, 0x2e10237b8UL, 0x405cd5491UL,
                                          0x7742eab1cUL, 0x85e6ac230UL, 0x8d388cdbaUL, 0x9f853ea93UL, 0xc41ea2445UL,
                                          0xcf1973594UL, 0x14a34a333UL, 0x31eacd15bUL, 0x6c79d2dabUL, 0x73cbb3935UL,
                                          0x89c155bd3UL, 0x8d6a46198UL, 0x91133675dUL, 0xa708d89fbUL, 0xae5ab9585UL,
                                          0xb9558a6d4UL, 0xb98743ab2UL, 0xd6cec68daUL, 0x1506bcaefUL, 0x4becd217aUL,
                                          0x4f95c273fUL, 0x658b649ddUL, 0xa76c4b1b7UL, 0xecf621f56UL, 0x1c8a56a57UL,
                                          0x3628e92baUL, 0x53706c0e2UL, 0x5e6b3d231UL, 0x7809cfa94UL, 0xe97eead6fUL,
                                          0x5af40604aUL, 0x7492988adUL, 0xed5994712UL, 0x5eceaf9edUL, 0x7c1632815UL,
                                          0xc1a0095b4UL, 0xe9e25d52bUL, 0x3a6705419UL, 0xa8333012fUL, 0x4ce5704d0UL,
                                          0x508e60a95UL, 0x877476120UL, 0xa864e950dUL, 0xea45cfce7UL, 0x19da047e8UL,
                                          0x24d4d5937UL, 0x6e079cc9bUL, 0x99f2e11d7UL, 0x33aa50429UL, 0x499ff26c7UL,
                                          0x50f1d3251UL, 0x66e7754efUL, 0x96ad633ceUL, 0x9a5653993UL, 0xaca30566cUL,
                                          0xc298a790aUL, 0x8be44b65dUL, 0xdc68f354bUL, 0x16f7f919bUL, 0x4dde0e826UL,
                                          0xd548cbd9fUL, 0xe0439ceeeUL, 0xfd8b1fd16UL, 0x76521bb7bUL, 0xd92375742UL,
                                          0xcab16d40cUL, 0x730c9dd72UL, 0xad9ba39c2UL, 0xb14493f87UL, 0x52b15651fUL,
                                          0x185409cadUL, 0x77ae2c68dUL, 0x94f5af4b5UL, 0xa13bad55UL, 0x61ea437cdUL,
                                          0xa022399e2UL, 0x203b163d1UL, 0x7bba8f40eUL, 0x95bc9442dUL, 0x41c0b5358UL,
                                          0x8e9c6cc81UL, 0xeb549670UL, 0x9da3a0b51UL, 0xd832a67a1UL, 0xdcd4350bcUL,
                                          0x4aa05fdd2UL, 0x60c7bb44eUL, 0x4b358b96cUL, 0x67299b45UL, 0xb9c89b5faUL,
                                          0x6975acaeaUL, 0x62b8f7afaUL, 0x33567c3d7UL, 0xbac139950UL, 0xa5927c62aUL,
                                          0x5c916e6a4UL, 0x260ecb7d5UL, 0x29b7bbd9aUL, 0x903205f26UL, 0xae72270a4UL,
                                          0x3d2ec51a7UL, 0x82ea55324UL, 0x11a6f3427UL, 0x1ca1c4576UL, 0xa40c81aefUL,
                                          0xbddccd730UL, 0xe617561eUL, 0x969317b0fUL, 0x67f781364UL, 0x610912f96UL,
                                          0xb2549fdfcUL, 0x6e5aaa6bUL, 0xb6c475339UL, 0xc56836a4dUL, 0x844e351ebUL,
                                          0x4647f83b4UL, 0x908a04f5UL, 0x7f51034c9UL, 0xaee537fcaUL, 0x5e92494baUL,
                                          0xd445808f4UL, 0x28d68b563UL, 0x4d25374bUL, 0x2bc065f65UL, 0x96dc3ea0cUL,
                                          0x4b2ade817UL, 0x7c3fd502UL, 0xe768b5cafUL, 0x17605cf6cUL, 0x182741ee4UL,
                                          0x62846097cUL, 0x72b5ebf80UL, 0x263da6e13UL, 0xfa841bcb5UL, 0x7e45e8c69UL,
                                          0x653c81fa0UL, 0x7443b5e70UL, 0xa5234afdUL, 0x74756f24eUL, 0x157ebf02aUL,
                                          0x82ef46939UL, 0x80d420264UL, 0x2aeed3e98UL, 0xb0a1dd4f8UL, 0xb5436be13UL,
                                          0x7b7b4b13bUL, 0x1ce80d6d3UL, 0x16c08427dUL, 0xee54462ddUL, 0x1f7644cceUL,
                                          0x9c7b5cc92UL, 0xe369138f8UL, 0x5d5a66e91UL, 0x485d62f49UL, 0xe6e819e94UL,
                                          0xb1f340eb5UL, 0x9d198ce2UL, 0xd60717437UL, 0x196b856cUL, 0xf0a6173a5UL,
                                          0x12c0e1ec6UL, 0x62b82d5cfUL, 0xad154c067UL, 0xce3778832UL, 0x6b0a7b864UL,
                                          0x4c7686694UL, 0x5058ff3ecUL, 0xd5e21ea23UL, 0x9ff4a76eeUL, 0x9dd981019UL,
                                          0x1bad4d30aUL, 0xc601896d1UL, 0x973439b48UL, 0x1ce7431a8UL, 0x57a8021d6UL,
                                          0xf9dba96e6UL, 0x83a2e4e7cUL, 0x8ea585380UL, 0xaf6c0e744UL, 0x875b73babUL,
                                          0xda34ca901UL, 0x2ab9727efUL, 0xd39f21b9aUL, 0x8a10b742fUL, 0x5f8952dbaUL,
                                          0xf8da71ab0UL, 0xc25f9df96UL, 0x6f8a5d94UL, 0xe42e63e1aUL, 0xb78409d1bUL,
                                          0x792229addUL, 0x5acf8c455UL, 0x2fc29a9b0UL, 0xea486237bUL, 0xb0c9685a0UL,
                                          0x1ad748a47UL, 0x3b4712d5UL, 0xf29216d30UL, 0x8dad65e49UL, 0xa2cf09ddUL,
                                          0xb5f174c6UL, 0xe54f57743UL, 0xb9cf54d78UL, 0x4a312a88aUL, 0x27babc962UL,
                                          0xb86897111UL, 0xf2ff6c116UL, 0x82274bd8aUL, 0x97023505eUL, 0x52d46edd1UL,
                                          0x585c1f538UL, 0xbddd00e43UL, 0x5590b74dfUL, 0x729404a1fUL, 0x65320855eUL,
                                          0xd3d4b6956UL, 0x7ae374f14UL, 0x2d7a60e06UL, 0x315cd9b5eUL, 0xfd36b4eacUL,
                                          0xf1df7642bUL, 0x55db27726UL, 0x8f15ebc19UL, 0x992f8c531UL, 0x62dea2a40UL,
                                          0x928275cabUL, 0x69c263cb9UL, 0xa774cca9eUL, 0x266b2110eUL, 0x1b14acbb8UL,
                                          0x624b8a71bUL, 0x1c539406bUL, 0x3086d529bUL, 0x111dd66eUL, 0x98cd630bfUL,
                                          0x8b9d1ffdcUL, 0x72b2f61e7UL, 0x9ed9d672bUL, 0x96cdd15f3UL, 0x6366c2504UL,
                                          0x6ca9df73aUL, 0xa066d60f0UL, 0xe7a4b8addUL, 0x8264647efUL, 0xaa195bf81UL,
                                          0x9a3db8244UL, 0x14d2df6aUL, 0xb63265b7UL, 0x2f010de73UL, 0x97e774986UL,
                                          0x248affc29UL, 0xfb57dcd11UL, 0xb1a7e4d9UL, 0x4bfa2d07dUL, 0x54e5cdf96UL,
                                          0x4c15c1c86UL, 0xcd9c61166UL, 0x499380b2aUL, 0x540308d09UL, 0x8b63fe66fUL,
                                          0xc81aeb35eUL, 0x86fe0bd5cUL, 0xce2480c2aUL, 0x1ab29ee60UL, 0x8048daa15UL,
                                          0xdbfeb2d39UL, 0x567c9858cUL, 0x2b6edc5bcUL, 0x2078fca82UL, 0xadacc22aaUL,
                                          0xb92486f49UL, 0x51fac5964UL, 0x691ee6420UL, 0xf63b3e129UL, 0x39be7e572UL,
                                          0xda2ce6c74UL, 0x20cf17a5cUL, 0xee55f9b6eUL, 0xfb8572726UL, 0xb2c2de548UL,
                                          0xcaa9bce92UL, 0xae9182db3UL, 0x74b6e5bd1UL, 0x137b252afUL, 0x51f686881UL,
                                          0xd672f6c02UL, 0x654146ce4UL, 0xf944bc825UL, 0xe8327f809UL, 0x76a73fd59UL,
                                          0xf79da4cb4UL, 0x956f8099bUL, 0x7b5f2655cUL, 0xd06b114a6UL, 0xd0697ca50UL,
                                          0x27c390797UL, 0xbc61ed9b2UL, 0xcc12dd19bUL, 0xeb7818d2cUL, 0x92fcecdaUL,
                                          0x89ded4ea1UL, 0x256a0ba34UL, 0xb6948e627UL, 0x1ef6b1054UL, 0x8639294a2UL,
                                          0xeda3780a4UL, 0x39ee2af1dUL, 0xcd257edc5UL, 0x2d9d6bc22UL, 0x121d3b47dUL,
                                          0x37e23f8adUL, 0x119f31cf6UL, 0x2c97f4f09UL, 0xd502abfe0UL, 0x10bc3ca77UL,
                                          0x53d7190efUL, 0x90c3e62a6UL, 0x7e9ebf675UL, 0x979ce23d1UL, 0x27f0c98e9UL,
                                          0xeafb4ae59UL, 0x7ca7fe2bdUL, 0x1490ca8f6UL, 0x9123387baUL, 0xb3bc73888UL,
                                          0x3ea87e325UL, 0x4888964aaUL, 0xa0188a6b9UL, 0xcd383c666UL, 0x40029a3fdUL,
                                          0xe1c00ac5cUL, 0x39e6f2b6eUL, 0xde664f622UL, 0xe979a75e8UL, 0x7c6b4c86cUL,
                                          0xfd492e071UL, 0x8fbb35118UL, 0x40b4a09b7UL, 0xaf80bd6daUL, 0x70e0b2521UL,
                                          0x2f5c54d93UL, 0x3f4a118d5UL, 0x9c1897b9UL, 0x79776eacUL, 0x84b00b17UL,
                                          0x3a95ad90eUL, 0x28c544095UL, 0x39d457c05UL, 0x7a3791a78UL, 0xbb770e22eUL,
                                          0x9a822bd6cUL, 0x68a4b1fedUL, 0xa5fd27b3bUL, 0xc3995b79UL, 0xd1519dff1UL,
                                          0x8e7eee359UL, 0xcd3ca50b1UL, 0xb73b8b793UL, 0x57aca1c43UL, 0xec2655277UL,
                                          0x785a2c1b3UL, 0x75a07985aUL, 0xa4b01eb69UL, 0xa18a11347UL, 0xdb1f28ca3UL,
                                          0x877ec3e25UL, 0x31f6341b8UL, 0x1363a3a4cUL, 0x75d8b9baUL, 0x7ae0792a9UL,
                                          0xa83a21651UL, 0x7f08f9fb5UL, 0xd0cf73a9UL, 0xb04dcc98eUL, 0xf65c7b0f8UL,
                                          0x65ddaf69aUL, 0x2cf9b86b3UL, 0x14cb51e25UL, 0xf48027b5bUL, 0xec26ea8bUL,
                                          0x44bafd45cUL, 0xb12c7c0c4UL, 0x959fd9d82UL, 0xc77c9725aUL, 0x48a22d462UL,
                                          0x8398e8072UL, 0xec89b05ceUL, 0xbb682d4c9UL, 0xe5a86d2ffUL, 0x358f01134UL,
                                          0x8556ddcf6UL, 0x67584b6e2UL, 0x11609439fUL, 0x8488816eUL, 0xaaf1a2c46UL,
                                          0xf879898cfUL, 0x8bbe5e2f7UL, 0x101eee363UL, 0x690f69377UL, 0xf5bd93cd9UL,
                                          0xcea4c2bf6UL, 0x9550be706UL, 0x2c5b38a60UL, 0xe72033547UL, 0x4458b0629UL,
                                          0xee8d9ed41UL, 0xd2f918d72UL, 0x78dc39fd3UL, 0x8212636f6UL, 0x7450a72a7UL,
                                          0xc4f0cf4c6UL, 0x367bcddcdUL, 0xc1caf8cc6UL, 0xa7f5b853dUL, 0x9d536818bUL,
                                          0x535e021b0UL, 0xa7eb8729eUL, 0x422a67b49UL, 0x929e928a6UL, 0x48e8aefccUL,
                                          0xa9897393cUL, 0x5eb81d37eUL, 0x1e80287b7UL, 0x34770d903UL, 0x2eef86728UL,
                                          0x59266ccb6UL, 0x110bba61UL, 0x1dfd284efUL, 0x447439d1bUL, 0xfece0e599UL,
                                          0x9309f3703UL, 0x80764d1ddUL, 0x353f1e6a0UL, 0x2c1c12dccUL, 0xc1d21b9d7UL,
                                          0x457ee453eUL, 0xd66faf540UL, 0x44831e652UL, 0xcfd49a848UL, 0x9312d4133UL,
                                          0x3f097d3eeUL, 0x8c9ebef7aUL, 0xa99e29e88UL, 0xe9fab22cUL, 0x4e748f4fbUL,
                                          0xecdee4288UL, 0xabce5f1d0UL, 0xc42f6876cUL, 0x7ed402ea0UL, 0xe5c4242c3UL,
                                          0xd5b2c31aeUL, 0x286863be6UL, 0x160444d94UL, 0x5f0f5808eUL, 0xae3d44b2aUL,
                                          0x9f5c5d109UL, 0x8ad9316d7UL, 0x3422ba064UL, 0x2fed11d56UL, 0xbea6e3e04UL,
                                          0x4b029eecUL, 0x6deed7435UL, 0x3718ce17cUL, 0x55857f5e2UL, 0x2edac7b62UL,
                                          0x85d6c512UL, 0xd6ca88e0fUL, 0x2b7e1fc69UL, 0xa699d5c1bUL, 0xf05ad74deUL,
                                          0x4cf5fb56dUL, 0x5725e07e1UL, 0x72f18a2deUL, 0x1cec52609UL, 0x48534243cUL,
                                          0x2523a4d69UL, 0x35c1b80d1UL, 0xa4d7338a7UL, 0xdb1af012UL, 0xe61a9475dUL,
                                          0x5df03f91UL, 0x97ae260bbUL, 0x32d627fefUL, 0xb640f73c2UL, 0x45a1ac9c6UL,
                                          0x6a2202de1UL, 0x57d3e25f2UL, 0x5aa9f986eUL, 0xcc859d8aUL, 0xe3ec6cca8UL,
                                          0x54e95e1aeUL, 0x446887b06UL, 0x7516732beUL, 0x3817ac8f5UL, 0x3e26d938cUL,
                                          0xaa81bc235UL, 0xdf387ca1bUL, 0xf3a3b3f2UL, 0xb4bf69677UL, 0xae21868edUL,
                                          0x81e1d2d9dUL, 0xa0a9ea14cUL, 0x8eee297a9UL, 0x4740c0559UL, 0xe8b141837UL,
                                          0xac69e0a3dUL, 0x9ed83a1e1UL, 0x5edb55ecbUL, 0x7340fe81UL, 0x50dfbc6bfUL,
                                          0x4f583508aUL, 0xcb1fb78bcUL, 0x4025ced2fUL, 0x39791ebecUL, 0x53ee388f1UL,
                                          0x7d6c0bd23UL, 0x93a995fbeUL, 0x8a41728deUL, 0x2fe70e053UL, 0xab3db443aUL,
                                          0x1364edb05UL, 0x47b6eeed6UL, 0x12e71af01UL, 0x52ff83587UL, 0x3a1575dd8UL,
                                          0x3feaa3564UL, 0xeacf78ba7UL, 0x872b94f8UL, 0xda8ddf9a2UL, 0x9aa920d2bUL,
                                          0x1f350ed36UL, 0x18a5e861fUL, 0x2c35b89c3UL, 0x3347ac48aUL, 0x7f23e022eUL,
                                          0x2459068fbUL, 0xe83be4b73UL};
      fromVector(codes, d._code_id);
      d._nbits = 36;
      d._tau = 11;
      d._type = TAG36h11;
      d._name = "TAG36h11";
    }
      break;
    case TAG36h10:
    {
      std::vector<std::uint64_t> codes = {0x1ca92a687UL, 0x20521ac4cUL, 0x27a3fb7d6UL, 0x2b4cebd9bUL, 0x3647bceeaUL,
                                          0x39f0ad4afUL, 0x3d999da74UL, 0x44eb7e5feUL, 0x538f3fd12UL, 0x5738302d7UL,
                                          0x65dbf19ebUL, 0x70d6c2b3aUL, 0x7f7a8424eUL, 0x832374813UL, 0x86cc64dd8UL,
                                          0x8a755539dUL, 0x9570264ecUL, 0x991916ab1UL, 0xa06af763bUL, 0xab65c878aUL,
                                          0xb2b7a9314UL, 0xb660998d9UL, 0xbdb27a463UL, 0xcc563bb77UL, 0xe24bdde15UL,
                                          0xed46aef64UL, 0xf4988faeeUL, 0x6e5417c7UL, 0x158902edbUL, 0x1cdae3a65UL,
                                          0x242cc45efUL, 0x27d5b4bb4UL, 0x2b7ea5179UL, 0x32d085d03UL, 0x3679762c8UL,
                                          0x3a226688dUL, 0x3dcb56e52UL, 0x48c627fa1UL, 0x5769e96b5UL, 0x6264ba804UL,
                                          0x660daadc9UL, 0x6d5f8b953UL, 0x74b16c4ddUL, 0x7fac3d62cUL, 0x91f8ef305UL,
                                          0x95a1df8caUL, 0x994acfe8fUL, 0xa09cb0a19UL, 0xa445a0fdeUL, 0xa7ee915a3UL,
                                          0xab9781b68UL, 0xaf407212dUL, 0xb69252cb7UL, 0xc8df04990UL, 0xd3d9d5adfUL,
                                          0xd782c60a4UL, 0xf12158907UL, 0x1d0c9ce43UL, 0x20b58d408UL, 0x2f594eb1cUL,
                                          0x3a541fc6bUL, 0x454ef0dbaUL, 0x53f2b24ceUL, 0x629673be2UL, 0x74e3258bbUL,
                                          0x8ad8c7b59UL, 0x9d2579832UL, 0xa8204a981UL, 0xaf722b50bUL, 0xb6c40c095UL,
                                          0xba6cfc65aUL, 0xf15311ce5UL, 0x748b3f83UL, 0xaf1a4548UL, 0xe9a94b0dUL,
                                          0x2be217935UL, 0x3e2ec960eUL, 0x4cd28ad22UL, 0x507b7b2e7UL, 0x54246b8acUL,
                                          0x57cd5be71UL, 0x6a1a0db4aUL, 0x6dc2fe10fUL, 0x876190972UL, 0x99ae4264bUL,
                                          0xabfaf4324UL, 0xc9427714cUL, 0xd09457cd6UL, 0xd43d4829bUL, 0xea32ea539UL,
                                          0xf52dbb688UL, 0x161e2ea75UL, 0x286ae074eUL, 0x66a2d6963UL, 0x8b3c3a315UL,
                                          0x8ee52a8daUL, 0xa131dc5b3UL, 0xe6bbb3352UL, 0xf55f74a66UL, 0x5a45bb5UL,
                                          0x7ac2673fUL, 0x1da1c89ddUL, 0x289c99b2cUL, 0x3ae94b805UL, 0x50deedaa3UL,
                                          0x5830ce62dUL, 0x5bd9bebf2UL, 0x632b9f77cUL, 0x6e26708cbUL, 0x841c12b69UL,
                                          0x92bfd427dUL, 0x9668c4842UL, 0x9dbaa53ccUL, 0xb007570a5UL, 0xb3b04766aUL,
                                          0xc25408d7eUL, 0xea965ccf5UL, 0xf93a1e409UL, 0x434ef558UL, 0x1681a1231UL,
                                          0x1dd381dbbUL, 0x302033a94UL, 0x75aa0a833UL, 0x92f18d65bUL, 0x9a436e1e5UL,
                                          0xa1954ed6fUL, 0xb78af100dUL, 0xbb33e15d2UL, 0xc62eb2721UL, 0x466a8936UL,
                                          0xf6179a85UL, 0x16b35a60fUL, 0x589440de9UL, 0x6738024fdUL, 0x847f85325UL,
                                          0x9e1e17b88UL, 0xacc1d929cUL, 0xb06ac9861UL, 0xd5042d213UL, 0xfd468118aUL,
                                          0xf9332e63UL, 0x342c96815UL, 0x37d586ddaUL, 0x551d09c02UL, 0x5c6eea78cUL,
                                          0x6017dad51UL, 0x9354ffe17UL, 0x9aa6e09a1UL, 0xa94aa20b5UL, 0xacf39267aUL,
                                          0xbb9753d8eUL, 0xbf4044353UL, 0x8730b6b7UL, 0x1716ccdcbUL, 0x22119df1aUL,
                                          0x3f5920d42UL, 0x58f7b35a5UL, 0x6b446527eUL, 0x972fa97baUL, 0x9e818a344UL,
                                          0xa5d36aeceUL, 0xd1beaf40aUL, 0xdcb980559UL, 0xf65812dbcUL, 0x139f95be4UL,
                                          0x6b761e65cUL, 0x6f1f0ec21UL, 0xa605242acUL, 0xe43d1a4c1UL, 0x29c6f1260UL,
                                          0x386ab2974UL, 0x4e6054c12UL, 0x8c984ae27UL, 0x97931bf76UL, 0x9ee4fcb00UL,
                                          0xd22221bc6UL, 0xe46ed389fUL, 0xebc0b4429UL, 0x6f82813ddUL, 0x732b719a2UL,
                                          0x9072f47caUL, 0x941be4d8fUL, 0x97c4d5354UL, 0x9f16b5edeUL, 0xb8b548741UL,
                                          0xd253dafa4UL, 0xd5fccb569UL, 0xd9a5bbb2eUL, 0xe4a08cc7dUL, 0x3c77156f5UL,
                                          0x7aaf0b90aUL, 0xb53e1155aUL, 0xb8e701b1fUL, 0x5c2b9448UL, 0x14667ab5cUL,
                                          0x47a39fc22UL, 0x4ef5807acUL, 0x64eb22a4aUL, 0x982847b10UL, 0xaa74f97e9UL,
                                          0xae1de9daeUL, 0xb56fca938UL, 0xf750b1112UL, 0x1bea14ac4UL, 0x1f9305089UL,
                                          0x233bf564eUL, 0x31dfb6d62UL, 0x3931978ecUL, 0x52d02a14fUL, 0x5a220acd9UL,
                                          0xb1f893751UL, 0xfb2b5aab5UL, 0x40b531854UL, 0x5a53c40b7UL, 0x8d90e917dUL,
                                          0x9139d9742UL, 0x94e2c9d07UL, 0xd6c3b04e1UL, 0xe910621baUL, 0xf40b33309UL,
                                          0x1152b6131UL, 0x32432951eUL, 0x3d3dfa66dUL, 0x65804e5e4UL, 0xab0a25383UL,
                                          0xb604f64d2UL, 0xde474a449UL, 0x11846f50fUL, 0x6d03e854cUL, 0x7455c90d6UL,
                                          0xab3bde761UL, 0xdad013262UL, 0xe973d4976UL, 0x2b54bb150UL, 0x9577f58a1UL,
                                          0x9920e5e66UL, 0xb66868c8eUL, 0xf4a05eea3UL, 0x3dd326207UL, 0x5b1aa902fUL,
                                          0x99529f244UL, 0xb2f131aa7UL, 0xd038b48cfUL, 0xd3e1a4e94UL, 0x24664cd82UL,
                                          0x36b2fea5bUL, 0x95db6805dUL, 0xa0d6391acUL, 0xabd10a2fbUL, 0x15f444a4cUL,
                                          0x2be9e6ceaUL, 0x57d52b226UL, 0x5f270bdb0UL, 0xb6fd94828UL, 0x879b19105UL,
                                          0xd476d0a2eUL, 0xe6c382707UL, 0xdbfa6a996UL, 0x1689705e6UL, 0x3b22d3f98UL,
                                          0x636527f0fUL, 0x7d03ba772UL, 0xee78d5a4dUL, 0xbf165a32aUL, 0xc2bf4a8efUL,
                                          0x517be89f2UL, 0x67718ac90UL, 0x6b1a7b255UL, 0x726c5bddfUL, 0xbb9f23143UL,
                                          0x1375abbbbUL, 0x296b4de59UL, 0x8893b745bUL, 0xa9842a848UL, 0xb827ebf5cUL,
                                          0x3840c894bUL, 0x6b7deda11UL, 0xbc02958ffUL, 0x55ba04b51UL, 0x76aa77f3eUL,
                                          0x9b43db8f0UL, 0x9eeccbeb5UL, 0xa295bc47aUL, 0xb4e26e153UL, 0xe476a2c54UL,
                                          0x6be1601cdUL, 0x6f8a50792UL, 0x97cca4709UL, 0xbc66080bbUL, 0x1093a056eUL,
                                          0x6fbc09b70UL, 0xb8eed0ed4UL, 0xcee473172UL, 0x23120b625UL, 0x5da111275UL,
                                          0xcf162c550UL, 0xec8f68756UL, 0xb5db0c4a9UL, 0x2b2ad1127UL, 0x536d2509eUL,
                                          0x9c9fec402UL, 0xc1394fdb4UL, 0x6c326b53UL, 0x5e99af5cbUL, 0xaf1e574b9UL,
                                          0xe6046cb44UL, 0x661d49533UL, 0x8e5f9d4aaUL, 0xdb3b54dd3UL, 0xe63625f22UL,
                                          0xe9df164e7UL, 0x455e8f524UL, 0x5b54317c2UL, 0xbe258b389UL, 0x54340a016UL,
                                          0x5b85eaba0UL, 0x1284dcc1aUL, 0x24d18e8f3UL, 0x4d13e286aUL, 0x8b4bd8a7fUL,
                                          0x215a5770cUL, 0x46572d87aUL, 0xc2c719ca4UL, 0x4ddac77e2UL, 0xd19c94796UL,
                                          0x2d1c0d7d3UL, 0x9ae8384e9UL, 0x9e9128aaeUL, 0xca7c6cfeaUL, 0x16282675UL,
                                          0xad985c97eUL, 0x4af8bc195UL, 0x9f580da26UL, 0xa6a9ee5b0UL, 0xbc9f9084eUL,
                                          0xd63e230b1UL, 0xc4232a7b6UL, 0xd66fdc48fUL, 0xec657e72dUL, 0xa364707a7UL,
                                          0xf79208c5aUL, 0xde88a1f91UL, 0x574f9ddf6UL, 0x65f35f50aUL, 0x69ce08eadUL,
                                          0x490f4ee9eUL, 0xc9282b88dUL, 0x752c4c7b8UL, 0xb364429cdUL, 0x8b53a7e34UL,
                                          0xbe90ccefaUL, 0xb0507dfa2UL, 0xd525e90UL, 0x5c549eecdUL, 0xe3bf5c446UL,
                                          0x936c6d936UL, 0x9747172d9UL, 0xca843c39fUL, 0xd57f0d4eeUL, 0x2d5595f66UL,
                                          0xbfbb2462eUL, 0x266727b98UL, 0x7ac679429UL, 0x26fc53732UL, 0x656602d25UL,
                                          0x2eb1a6a78UL, 0x4850392dbUL, 0xe5b098af2UL, 0xab534c280UL, 0x9ce143f4aUL,
                                          0xf4b7cc9c2UL, 0x35b8e0d6UL, 0x871d5b08aUL, 0x5b958930aUL, 0xb429a7faUL,
                                          0x54d8d431aUL, 0x7d1b28291UL, 0xa1e645021UL, 0xb80da069dUL, 0xeef3b5d28UL,
                                          0x263d3db6fUL, 0x9592d503UL, 0x4b9d86499UL, 0x6c8df9886UL, 0xa3740ef11UL,
                                          0xc4963b6dcUL, 0x6da94672UL, 0x53b64bf9bUL, 0xb2deb559dUL, 0xf116ab7b2UL,
                                          0x8ace1aa04UL, 0x8ea8c43a7UL, 0x6a4119dd3UL, 0x99d54e8d4UL, 0xc969833d5UL,
                                          0xf554c7911UL, 0x3ade9e6b0UL, 0x6e1bc3776UL, 0x7916948c5UL, 0xdbe7ee48cUL,
                                          0x79484dca3UL, 0xf992e3a70UL, 0x884f81b73UL, 0xc68777d88UL, 0x603ee6fdaUL,
                                          0x728b98cb3UL, 0xb12701684UL, 0xd5f21e414UL, 0x58652f15UL, 0x2dc8a6e8cUL,
                                          0x4767396efUL, 0xb8dc549caUL, 0xf36b5a61aUL, 0xd09ece7dUL, 0xdda77175aUL,
                                          0x5e9c56d1UL, 0x73e7a97c5UL, 0xb21f9f9daUL, 0xde3c9d2f4UL, 0x69504ae32UL,
                                          0x77f40c546UL, 0xed1217de6UL, 0x3a1f88aedUL, 0xe623a9a18UL, 0xaeec67a8UL,
                                          0xbea83aa19UL, 0x92eeaf8bbUL, 0xa5d08d12eUL, 0x819a9bf38UL, 0x473d4f6c6UL,
                                          0xb192431f5UL, 0xa6c92b484UL, 0x7046885b5UL, 0xb9ab08cf7UL, 0x782d94cd9UL,
                                          0xf158032faUL, 0x77f5e976UL, 0x12dda2281UL, 0xe72417123UL, 0x3056de487UL,
                                          0xe3de9931aUL, 0xeb3079ea4UL, 0xe4420bad6UL, 0x439c2e4b6UL, 0x47da4a615UL,
                                          0xd7cfdda3UL, 0x56afc5107UL, 0xe978c5f8bUL, 0x5aede1266UL, 0xaf1b79719UL,
                                          0xf8b1b3239UL, 0x75e8845dbUL, 0xbf1b4b93fUL, 0xfd5341b54UL, 0xa2373b2d3UL,
                                          0x5967e672bUL, 0xa2cc66e6dUL, 0xb17028581UL, 0xb54ad1f24UL, 0xe91d22b84UL,
                                          0xde85c41f1UL, 0x53d588e6fUL, 0xe9e407afcUL, 0xfc6272bb3UL, 0xa8ca0629aUL,
                                          0xb86665d04UL, 0x5a58fde9UL, 0x1855b427eUL, 0xaabb42946UL, 0xe204ca78dUL,
                                          0x32897267bUL, 0xa78d7ae2UL, 0x96536a598UL, 0xbf2aea0a9UL, 0xc9bcd56cUL,
                                          0x81eb921eaUL, 0x2732fe125UL, 0x2eb69808dUL, 0x61f3bd153UL, 0x8ddf0168fUL,
                                          0x921d1d7eeUL, 0x2bd48ca40UL, 0x83ab154b8UL, 0x5f436aee4UL, 0x93dca0abcUL,
                                          0x26d75ad1eUL, 0x872a1ba54UL, 0x373a9f700UL, 0x50d931f63UL, 0x12d2f512cUL,
                                          0xc6efdbb59UL, 0x88e99ed22UL, 0x903b7f8acUL, 0xa9da1210fUL, 0xa2eba3d41UL,
                                          0xfe9cd615cUL, 0xfb8911731UL, 0x1cab3defcUL, 0xe6289b02dUL, 0x66d6a35b6UL,
                                          0xb0096a91aUL, 0xc9afcc532UL, 0x80aebe5acUL, 0xd2f2e9768UL, 0xcc67edb56UL,
                                          0xa51e37f35UL, 0x6ac0eb6c3UL, 0x6af2a4aa1UL, 0xe76290ecbUL, 0x37e738db9UL,
                                          0x72d9b11c5UL, 0x76e613f46UL, 0x4f073278bUL, 0xe1eea307UL, 0xe9e8f9111UL,
                                          0x793ee6f5UL, 0x17304e15fUL, 0x7a3361104UL, 0x731339958UL, 0x8daa6a511UL,
                                          0xa4037ef6bUL, 0x210896f2fUL, 0xafc535032UL, 0xe3025a0f8UL, 0x63e21ba5fUL,
                                          0x3ebb5b8c8UL, 0xf9c6b06c3UL, 0xca95ee37eUL, 0x81f852bb4UL, 0xd6895d823UL,
                                          0x7040cca75UL, 0x4d66ec391UL, 0x4a216e588UL, 0x51d6c18ceUL, 0x47711c319UL,
                                          0x6ae7f794cUL, 0x4abe694d7UL, 0xbc96f6f6eUL, 0x57aa76cd2UL, 0xf948f2648UL,
                                          0x31bcd1bc3UL, 0x94c7b3f1dUL, 0x32eef86acUL, 0x668f8ff2eUL, 0x6de170ab8UL,
                                          0x9341b93e2UL, 0x974e1c163UL, 0x73182af6dUL, 0x5d85fb48bUL, 0x78b257bdeUL,
                                          0x173d0eb29UL, 0x5add785d1UL, 0xaf6e83240UL, 0xdce79166cUL, 0x22716840bUL,
                                          0x7b408f1d9UL, 0xde43a217eUL, 0x36e10fb6eUL, 0xea9a83ddfUL, 0x4dcf50162UL,
                                          0x9aab07a8bUL, 0x281364431UL, 0x8392dd46eUL, 0x945e6aceUL, 0x8d07b3a82UL,
                                          0xd012f1990UL, 0xb7098acc7UL, 0xc2fcfa16cUL, 0x1e7c731a9UL, 0xf16ea68eeUL,
                                          0x2fa69cb03UL, 0x4cee1f92bUL, 0x1e20cfda2UL, 0x9e9d1ef4dUL, 0xe83358a6dUL,
                                          0x59a873d48UL, 0xa6842b671UL, 0x6885bdbefUL, 0x73e4014faUL, 0x7b9954840UL,
                                          0x548157ffdUL, 0x8853a8c5dUL, 0xeb8874fe0UL, 0xb25d4f257UL, 0x36b447da5UL,
                                          0x71d87958fUL, 0xeedd91553UL, 0x4af23612aUL, 0x278329eacUL, 0x191121b76UL,
                                          0x6e691175dUL, 0xbd140329UL, 0x6ed4532ceUL, 0xd5e3c8ff4UL, 0xe26c64033UL,
                                          0x494a2097bUL, 0xf5e36d440UL, 0xb7818d202UL, 0xa63548c34UL, 0x682f0bdfdUL,
                                          0x3d3c65c17UL, 0x4c4399ae7UL, 0x6b18e67ffUL, 0x4778211a3UL, 0x4089b2dd5UL,
                                          0x21edee850UL, 0x739cede72UL, 0x858dfc744UL, 0x4bc5dba6cUL, 0x21cbd3bdcUL,
                                          0xac83de313UL, 0x6135f08daUL, 0xd3a3a9f0bUL, 0xeb2716099UL, 0x40b88e413UL,
                                          0x992442a25UL, 0xc639de695UL, 0x683bcc7c7UL, 0x6245fc74fUL, 0x543766bd5UL,
                                          0x375356569UL, 0xfa45b7a88UL, 0x9f29b1207UL, 0xba245457cUL, 0x5b98e5ec9UL,
                                          0x204aca6b6UL, 0xd52e9605bUL, 0x504a40d28UL, 0xab6e1695eUL, 0xa26481ebbUL,
                                          0x9455ec341UL, 0x2f05f98e9UL, 0xead83365cUL, 0xb928d8486UL, 0x7b860de0bUL,
                                          0x964ef7da2UL, 0x2422962b9UL, 0x28f5ddfb2UL, 0x8c5c63713UL, 0x9068c6494UL,
                                          0x50aad5744UL, 0x93e7cca30UL, 0x9825e8b8fUL, 0x3a8b4947dUL, 0xfa06737b5UL,
                                          0xcb9c963e8UL, 0x91a2bc332UL, 0x284666b59UL, 0xceb82a1c8UL, 0xa2fe9f06aUL,
                                          0x6fa50365UL, 0x19aa747faUL, 0xa6e117dc2UL, 0x2b3810910UL, 0xff7e857b2UL,
                                          0x48b55c3eUL, 0x975456ac2UL, 0xad200ed37UL, 0xe4d4d86efUL, 0xb3ec62491UL,
                                          0x8cd465c4eUL, 0x8a2be2d94UL, 0x5a9f7d648UL, 0x59e067a85UL, 0xb5c35327eUL,
                                          0xc4ca8714eUL, 0xe927a04cUL, 0xf71eac628UL, 0x109354e62UL, 0x1037b1a5bUL,
                                          0x26c27f893UL, 0x3597fa385UL, 0xee24b62efUL, 0x4b31f921cUL, 0x650244e5dUL,
                                          0xcfec64526UL, 0xcd4bb0a21UL, 0x648c56197UL, 0xbd95056f8UL, 0x983c8c183UL,
                                          0xddc662f22UL, 0x5631bb980UL, 0x1203f56f3UL, 0x6a0c37549UL, 0xa59baa8a4UL,
                                          0x6a23a5068UL, 0xad609c354UL, 0xf6f6d5e74UL, 0x36bc95f79UL, 0x424c92c62UL,
                                          0x3692abf50UL, 0xa4bc460dUL, 0x1b4434b89UL, 0x4eb31302dUL, 0x9a3a891f9UL,
                                          0x93af8d5e7UL, 0xef60bfa02UL, 0x607a378d6UL, 0x5a5a7d835UL, 0x536c0f467UL,
                                          0x11f66a7feUL, 0x74c7c43c5UL, 0x66eae7c29UL, 0xf5a785d2cUL, 0xd3948a5c0UL,
                                          0xec1094aa4UL, 0xfcad61c19UL, 0x49ca7108aUL, 0x77437f4b6UL, 0x553083d4aUL,
                                          0x5c26c14cdUL, 0x6eb4caceeUL, 0xe8aded63cUL, 0x132aa4b5cUL, 0x57603a19eUL,
                                          0xee359dda3UL, 0xd0f5ea330UL, 0x46b0f0b1fUL, 0xd47cbfc81UL, 0x3ed1b37b0UL,
                                          0x4c8c752d8UL, 0xac202044bUL, 0x207f16128UL, 0x53f5c3981UL, 0x70e1a33a2UL,
                                          0xed8348baaUL, 0x798f94a3eUL, 0x8896c890eUL, 0x521425a3fUL, 0xc8329e9eaUL,
                                          0x41f238ba5UL, 0x1093d3c81UL, 0xcab628e90UL, 0xa1b4bf356UL, 0x92eee2fceUL,
                                          0x59d35b9afUL, 0x2176e9f53UL, 0x7c9abfb89UL, 0xb06d107e9UL, 0xe94c318d5UL,
                                          0x2f397ae30UL, 0xd7e5a1a48UL, 0x98c4abc47UL, 0x574737c29UL, 0xd7f5401b2UL,
                                          0x852b87bc6UL, 0x1180fa957UL, 0x501c63328UL, 0xfd28c0d13UL, 0xf764aa079UL,
                                          0xc8a6f8c5aUL, 0x975b10240UL, 0x6bd33e4c0UL, 0xa1113e39cUL, 0xabcfa8fb8UL,
                                          0x149ea1facUL, 0xf6443556fUL, 0x959da07e7UL, 0x4721a2ac4UL, 0x30bde0f15UL,
                                          0xc7c4fdef8UL, 0xb7feb4465UL, 0x56675073cUL, 0x96f3f57b9UL, 0x876f0386eUL,
                                          0x393f0a7e8UL, 0x32b40ebd6UL, 0x10a11346aUL, 0xfb81f48aeUL, 0xa892877eUL,
                                          0x86f636e08UL, 0x4fbc4d72bUL, 0x561d5f314UL, 0xca18e2835UL, 0x7e6f519f5UL,
                                          0x8b94e7983UL, 0x619adfaf3UL, 0x4c9ddbbabUL, 0xcb6a461f2UL, 0xf6e22456UL,
                                          0x858c9b401UL, 0x92dc1b3b8UL, 0x3a783615bUL, 0xc74b66f67UL, 0xea90891bcUL,
                                          0x31a829e4bUL, 0x7bd38f505UL, 0xb4476ea80UL, 0x1af371feaUL, 0x84894ff56UL,
                                          0x537584dfUL, 0xd4ebdd1d0UL, 0xb43cc192bUL, 0x76700d287UL, 0xaaad9fa58UL,
                                          0xfa511710fUL, 0x6e54699e5UL, 0x6d73391aeUL, 0x3c2f1fb49UL, 0x4fbd96a75UL,
                                          0x252e6304bUL, 0xe72826214UL, 0x8fe6c9336UL, 0x326397743UL, 0xe03bc524UL,
                                          0xdedac9594UL, 0x4ff19096UL, 0x409b4cdbbUL, 0xb15921888UL, 0xa259bcd6dUL,
                                          0x43c67f305UL, 0x1dc65dcecUL, 0x1730b4f85UL, 0xf8a48f16aUL, 0x57a30e743UL,
                                          0x5afd9839UL, 0x682d5f3aeUL, 0xb694337eUL, 0x758c7dacfUL, 0x18fa1ae7dUL,
                                          0x5ba9b5984UL, 0x32e1d45ddUL, 0x672f05518UL, 0x382ffc5b1UL, 0x8f0b08f33UL,
                                          0xb4a4d9ff0UL, 0x53ba0f980UL, 0x2ac0751fbUL, 0xab005de73UL, 0x61ab7be9bUL,
                                          0x63078c9adUL, 0x27659d148UL, 0xc653c684fUL, 0xecee05017UL, 0x24378ce5eUL,
                                          0xd0f7e5bacUL, 0x8b4bf4199UL, 0xb7aa495fbUL, 0xf3d6b78a5UL, 0x98bab1024UL,
                                          0x6b4971fadUL, 0x723d6c89UL, 0xa17071a75UL, 0xd55a301f4UL, 0x988de925bUL,
                                          0x7ca276f45UL, 0x321b6e484UL, 0x316149ed6UL, 0xb8dba5bb9UL, 0xaad4df3f4UL,
                                          0x178e204f5UL, 0x95cd2e357UL, 0x73fb8a733UL, 0x84ca10c86UL, 0x89498492dUL,
                                          0x5b9bdf383UL, 0x66c58bb10UL, 0xf153ac21eUL, 0x7ca5d3b04UL, 0x637a521c7UL,
                                          0xb5ed589c1UL, 0xd0a3c644eUL, 0x3d5d0754fUL, 0xc6b901174UL, 0x3e96fd3edUL,
                                          0x79c2fdf8cUL, 0x6594ae371UL, 0x3504fd77aUL, 0x32b81dcc7UL, 0x57b4f3e35UL,
                                          0x4c9808072UL, 0xa06c10993UL, 0xb059666afUL, 0x72b69c034UL, 0xe33ae836eUL,
                                          0xad6cadf0dUL, 0x19573ace1UL, 0xd562fd1e7UL, 0x847804d9dUL, 0x9e32f6734UL,
                                          0x2355c580fUL, 0x2f177b8d6UL, 0xd689ac650UL, 0x71b70abcUL, 0x254915730UL,
                                          0xc5934f949UL, 0x134d59d09UL, 0x2c731fb06UL, 0xd90c6c5cbUL, 0xcc3f9bca4UL,
                                          0xbf078980cUL, 0x80838e95aUL, 0x5ccd6f054UL, 0x378bae56UL, 0x8d1ddb978UL,
                                          0x93b875cf4UL, 0xe2ce90bc6UL, 0xec291b91bUL, 0xd0a11bdc1UL, 0x751be7244UL,
                                          0x579fcd29eUL, 0xf76e5ccb1UL, 0xeb33da184UL, 0xc0ac75b0fUL, 0x15454fb33UL,
                                          0x8b92a411cUL, 0x7da34b476UL, 0x4f413d45eUL, 0x10ec1dbeaUL, 0x650739b93UL,
                                          0x315e08a31UL, 0xd4fd5f1bdUL, 0xb5058a126UL, 0x20d5cb63bUL, 0xc1598dfe7UL,
                                          0xa0a2a338dUL, 0xe29af7686UL, 0x83fd0cac9UL, 0x14a8094d8UL, 0x816e0afa3UL,
                                          0x499ef5d2cUL, 0xbddbd0d95UL, 0xa30839ca9UL, 0x4fd33fb4cUL, 0xef63557b7UL,
                                          0x535f06ab2UL, 0xd47d352eUL, 0xa371e6dc4UL, 0x8ac914b17UL, 0x6ba9d69d7UL,
                                          0x96da89aaUL, 0x65bbd5d14UL, 0xd41d2c5c4UL, 0x52a283583UL, 0x4c1f56d26UL,
                                          0x86cd9984aUL, 0x26cbcedc6UL, 0x1aa0eaa03UL, 0xb95d5ad2cUL, 0xe2eb56b20UL,
                                          0xff49d9d5cUL, 0xcce338378UL, 0x330e9fac7UL, 0xe2f53974aUL, 0x668d1c6d5UL,
                                          0xeca0ba751UL, 0x8d48ab5e6UL, 0xd205e18cdUL, 0x1c391633cUL, 0xef5d02e5fUL,
                                          0xd12bb5f20UL, 0x323215199UL, 0x88f5b3ffcUL, 0x931445f29UL, 0xb893cb727UL,
                                          0x32851ecc0UL, 0x80b44d81bUL, 0x5aa48da98UL, 0x46d1e1284UL, 0x4c837ba14UL,
                                          0xeb22c26deUL, 0xe51e9d246UL, 0x8d03deee6UL, 0x5af8e0909UL, 0xbde9773a4UL,
                                          0xbf611cabfUL, 0xd24ac96e7UL, 0x9fe919318UL, 0x50d0206a6UL, 0xb43b9741cUL,
                                          0xba48d4fb3UL, 0x6bccd7290UL, 0x8bc6bfb9cUL, 0xe5a036c9fUL, 0xa80a2cfeeUL,
                                          0xc193655a7UL, 0x7c8e5170dUL, 0x6141edbbbUL, 0x4d6b990dcUL, 0xcc49b5702UL,
                                          0x2343fef58UL, 0xd50cb593cUL, 0x4248a60cdUL, 0x901cfbd4cUL, 0x64a4c8736UL,
                                          0x1b2dcbaeaUL, 0xd691e5f4cUL, 0xdf352a493UL, 0x1991ac7daUL, 0x4c4879f45UL,
                                          0x9b34aadeeUL, 0x52bb3db0dUL, 0x7b9a8c9d3UL, 0xd7ce6e47eUL, 0xec0b922d8UL,
                                          0x8079cab6bUL, 0xabadc8899UL, 0xf57b93b7UL, 0x5c4ef219UL, 0xd7a438d49UL,
                                          0xf55ecca97UL, 0xd07899f1dUL, 0x260947d6cUL, 0xffbd21ab6UL, 0xd04ff923eUL,
                                          0x964b72033UL, 0x31ac3fd7eUL, 0xd2c52e2c4UL, 0x799a640efUL, 0x98dd061edUL,
                                          0x5cb2ab7b8UL, 0x72f3881c8UL, 0xe65ed1164UL, 0x34fa0bd5bUL, 0x64f9823cdUL,
                                          0x3797e1ac0UL, 0x2fb8a4751UL, 0x6f347342eUL, 0x22dd7ea0aUL, 0xb19b65e57UL,
                                          0x44fe83e8aUL, 0x7732732eUL, 0x64de20ed7UL, 0x6c9ea834UL, 0x8ce066650UL,
                                          0xc2a685ff0UL, 0x64f19b01fUL, 0x491ab8a88UL, 0x41212fe5aUL, 0x6f9916f3bUL,
                                          0x694f72e71UL, 0xad7a5b35eUL, 0xf62795292UL, 0xc8cdc3d3aUL, 0xfbc6b3518UL,
                                          0x67b631901UL, 0x5b5ba79d5UL, 0xf4fadebddUL, 0xac7c802e7UL, 0x385712d9dUL,
                                          0x64bd375b4UL, 0xc9a11df70UL, 0x88355bf31UL, 0x606ffbb0aUL, 0xbda93c2d5UL,
                                          0x7c5f94f0aUL, 0x76fe26501UL, 0x5d8b9153cUL, 0x886bbb218UL, 0xacee2fecaUL,
                                          0x2ad19a925UL, 0x83b97855cUL, 0xd36608312UL, 0x8ac60dbc7UL, 0x885c8f58UL,
                                          0x8abbdf891UL, 0xea1602271UL, 0xad654fee1UL, 0x6c461195eUL, 0x5eeb1a327UL,
                                          0x18d743962UL, 0x1fc7c55a5UL, 0xaba749670UL, 0x9c9a59c60UL, 0x6e5bafc06UL,
                                          0x96977db12UL, 0xa97b6ebfaUL, 0x63d2d9da6UL, 0xfab00cd60UL, 0xd7bdf4632UL,
                                          0xf83878d59UL, 0xb1c2c462eUL, 0x14e5144a7UL, 0xf4a909b28UL, 0xe979a185bUL,
                                          0x908090a64UL, 0x99eccd798UL, 0x348780a96UL, 0xfdc7ad169UL, 0xa600c2e5bUL,
                                          0xb0968cd98UL, 0x1a45ec098UL, 0x99118c1b4UL, 0x8afa5cd5aUL, 0x1db7e655eUL,
                                          0x9f637e452UL, 0x9568504e3UL, 0x45b2a662UL, 0xf2a1455a2UL, 0x6c1ca9e75UL,
                                          0x30a4a4639UL, 0xc6c2c1a30UL, 0x87500b452UL, 0x5e338bb2eUL, 0xd9dd11dffUL,
                                          0x8c4b5d012UL, 0x8191194e0UL, 0xdd11db867UL, 0xc67c151ceUL, 0x5cb1a00e4UL,
                                          0x98b7a1c6UL, 0x369f35cd4UL, 0xca2190bdbUL, 0x6e14bb3b9UL, 0x8d5692f8cUL,
                                          0xca4b2f4f8UL, 0x787f06877UL, 0x8acbb8550UL, 0x535f4b56aUL, 0xf4caf7ecbUL,
                                          0xd4615b258UL, 0x347ca7070UL, 0x3c798c85dUL, 0x460506465UL, 0x870d0a5dcUL,
                                          0x6510b2464UL, 0xd1dba5544UL, 0xd57789a33UL, 0xe2417c5baUL, 0xb5ff8628cUL,
                                          0xa3bb22787UL, 0xa16b64f34UL, 0x421e81d3dUL, 0x35b4596a7UL, 0x8d7a2dd7eUL,
                                          0x50b2d83faUL, 0x9ea87e7c2UL, 0xd5055e752UL, 0xf96aa9da5UL, 0xb096e2a07UL,
                                          0x49970b44bUL, 0x867fb1518UL, 0x5d0f5dba2UL, 0x1b191d11eUL, 0x8e839bb8fUL,
                                          0x1cd4aca15UL, 0x971ec5615UL, 0x7d72a7ebdUL, 0x8b1253bfbUL, 0xe11de1d25UL,
                                          0xa7566839UL, 0xf4f3542e0UL, 0x1ea791e32UL, 0x32a84f759UL, 0x646f1844eUL,
                                          0x42af26809UL, 0x1f4b464ffUL, 0xda684d2d9UL, 0xd854f5fb9UL, 0x4d4d3e91aUL,
                                          0x5af3ef4e2UL, 0x8a1ef5ce7UL, 0x2354febf3UL, 0xb3c5a8944UL, 0x98b62a144UL,
                                          0x9bdba0b4eUL, 0x4aa99b42UL, 0x8099ea151UL, 0x2185463a3UL, 0xb0a1ae997UL,
                                          0xe628d5770UL, 0xb40b5ac89UL, 0x27213b17dUL, 0x4d21db5b5UL, 0x10d0748f7UL,
                                          0x2276c7876UL, 0xb98bee56dUL, 0xbd1ca6ae8UL, 0x824ab48faUL, 0xc6f35ae62UL,
                                          0x3547a563cUL, 0xf1fc0d824UL, 0x58f55ed75UL, 0xaa9d0de01UL, 0x4719dde60UL,
                                          0xd5386b3ddUL, 0x4d8d9f666UL, 0xaee36013bUL, 0xba4ee322fUL, 0x898d2db4eUL,
                                          0x9fe364808UL, 0xbb13e8045UL, 0xbe346d43aUL, 0xb4c9f886fUL, 0xc9a6f53b8UL,
                                          0xed5a7b6fUL, 0x2a1fac740UL, 0xb8c134a59UL, 0xb1f773993UL, 0xc4d9d0025UL,
                                          0xca905bdcaUL, 0x3150a39a7UL, 0xe8329fad5UL, 0xbd4f98059UL, 0x3bc5cf6cdUL,
                                          0xc982fdd03UL, 0xa372de28UL, 0x73fe2e35aUL, 0xb9f684ecUL, 0xc543ff680UL,
                                          0x1bcf5f09aUL, 0x51b2a8099UL, 0xee53277c2UL, 0xb3835a6cUL, 0xaed6765c1UL,
                                          0x92cfd64c8UL, 0xd20c60ed2UL, 0x59dbd9f51UL, 0xb6acb694bUL, 0x427dcd5fdUL,
                                          0x646336a75UL, 0x8008dea4dUL, 0xaf2bdc7cUL, 0xb8a46478aUL, 0xb02c535b6UL,
                                          0xc645d8631UL, 0x44b4af3dUL, 0xc9edfe6cbUL, 0x32ac8ea2aUL, 0x79266a23fUL,
                                          0xc2d902e93UL, 0x6ae5cfbdbUL, 0x2c66c633eUL, 0xeb7a8a4e3UL, 0xcb17281cfUL,
                                          0x7ca378680UL, 0x7ac81509dUL, 0xa59a05073UL, 0xc9cb9f18dUL, 0xb78100d29UL,
                                          0xfab49420aUL, 0xd0a4e69c4UL, 0xd6c33f722UL, 0x68d21bff8UL, 0x1fdad8ca3UL,
                                          0x2884d6968UL, 0xb091ff264UL, 0xeb5fb236fUL, 0xa3d2a1839UL, 0x527db0bc8UL,
                                          0x2dc68cd9fUL, 0xe3f4ea98aUL, 0xa629fe44fUL, 0xb73bd7d66UL, 0x2abfd7b6bUL,
                                          0x1b4056054UL, 0xd6efaac28UL, 0xd13cc950UL, 0xef84ead94UL, 0x5b6ee0d50UL,
                                          0xf4bec692UL, 0xde1b98881UL, 0x55ccccd31UL, 0x86d9b84dUL, 0x5ab736e3dUL,
                                          0x167d2f005UL, 0x118ed1522UL, 0x38bbdc903UL, 0x39cd31ac2UL, 0x31091bc51UL,
                                          0xd66a87d3fUL, 0xafdade6d3UL, 0x2bd1fe097UL, 0x5cf545dd2UL, 0x5e0af578eUL,
                                          0x6fe6dd4c9UL, 0x862bc8fcaUL, 0xcbce0b4c6UL, 0x8b7fa8ddUL, 0x3d108ae9fUL,
                                          0xfed2d914aUL, 0xbab304bd8UL, 0xdebe74f8dUL, 0x1e857e3dcUL, 0x570340581UL,
                                          0x114bbf4f5UL, 0xa3cfc0566UL, 0x4026cd686UL, 0x266fb76cdUL, 0xb715773bbUL,
                                          0x2fd2785fdUL, 0x481b34cadUL, 0x11c58d2baUL, 0x3a5186f4dUL, 0xda55ab71cUL,
                                          0xac887db92UL, 0x9bd6d5592UL, 0x45857d12aUL, 0x8c862f0b9UL, 0x870c88666UL,
                                          0x4a4f4901fUL, 0x774a993d0UL, 0xc9f16c81dUL, 0xeb415e9efUL, 0x307aa6302UL,
                                          0xa246f21eeUL, 0x1a4f8a9c2UL, 0xcf09f9b4UL, 0xdb30dbb49UL, 0x3581be36fUL,
                                          0x6919a4318UL, 0x8ee677afdUL, 0x5944b9d59UL, 0x8d5fe61aaUL, 0x77c174b1dUL,
                                          0x5cff8fa10UL, 0xc1ce82f48UL, 0x7fbb18e65UL, 0xb6737103UL, 0xe2d30a9b6UL,
                                          0x6481ff469UL, 0x5834b4d26UL, 0x3bba517d5UL, 0xeee6e8080UL, 0x5fe4fea5eUL,
                                          0xe84e94c8cUL, 0xba2ad0a2aUL, 0xa7f2aead0UL, 0x63cecb46dUL, 0x8943d7229UL,
                                          0x1d3878b2bUL, 0xf2b4efe94UL, 0xd9af1949dUL, 0xbb5824d39UL, 0xb8d8f5090UL,
                                          0xed5e19d08UL, 0x60287437eUL, 0x8fe6ae5c2UL, 0x6c85ac058UL, 0xb906be1b8UL,
                                          0xf9d423f65UL, 0x6efed81d6UL, 0x781b67fa2UL, 0xe1dd437acUL, 0x7a9201a8cUL,
                                          0xfb444c819UL, 0xce75af959UL, 0x86df6e72bUL, 0x756695aa7UL, 0xb7b2bddf2UL,
                                          0xf19a1b99eUL, 0x9a5790e90UL, 0x1d3b3eac0UL, 0xa5c5d9d2bUL, 0x152850218UL,
                                          0x25c4ba6eUL, 0xd4a5f4bebUL, 0x709cec10eUL, 0x94ddbdb6cUL, 0x9d1218277UL,
                                          0x6190ca34aUL, 0x468ed6a3fUL, 0x801bda52eUL, 0x261b3f1a9UL, 0xb3494d9bUL,
                                          0x583e2d7e5UL, 0x9407a80f2UL, 0x58e902456UL, 0x9108c2273UL, 0x59778ff8cUL,
                                          0xd6ce05028UL, 0x286adc62UL, 0x7ed3060dcUL, 0x57b7e03edUL, 0x3e3dce5c1UL,
                                          0x1bebc2295UL, 0x14a17c9aUL, 0xc7d90fbdaUL, 0x8158ae35aUL, 0x69d70a335UL,
                                          0xd3ef97931UL, 0x5793efb7aUL, 0xe6989ef43UL, 0xcd15f0116UL, 0xf9dbc6e25UL,
                                          0xda4a91117UL, 0x54d0917aUL, 0x60f2c3f15UL, 0x7393b0a66UL, 0x6630ed79bUL,
                                          0xed8589c60UL, 0x7db37ab26UL, 0xc4631e80aUL, 0x1badaf501UL, 0x9bdef764dUL,
                                          0xdd0949b4bUL, 0x86f116771UL, 0xacd7ea109UL, 0x7cc9d2f6bUL, 0x3f5598822UL,
                                          0x4ba5a8d0cUL, 0x66e7f9c42UL, 0x33127fb36UL, 0xc85ff976UL, 0x9dbb32ddfUL,
                                          0x3d06c7a56UL, 0xac07601ddUL, 0x5fda3d7e9UL, 0x40a47aef0UL, 0x139928cd0UL,
                                          0x183ab75ebUL, 0x9dd6d1f4bUL, 0x954afec44UL, 0x29953fe22UL, 0xf947e49b1UL,
                                          0xa74266cb0UL, 0x3bbb7fdabUL, 0x8a72b63d1UL, 0x8763e2fbbUL, 0x8c9b4f9a2UL,
                                          0xa35f5a861UL, 0x99e54752cUL, 0x2fdb8e16fUL, 0x2d083ed68UL, 0xa05d36c5eUL,
                                          0x5460842feUL, 0x173ae0ee6UL, 0x38b3c62e5UL, 0x476c1ae99UL, 0x9a8cb898aUL,
                                          0x19d4032acUL, 0xa9c01d80bUL, 0xca7d5e4deUL, 0x295d53115UL, 0xb26740e51UL,
                                          0xbf21b0988UL, 0x167391c15UL, 0xd10af35c6UL, 0xd94750799UL, 0xcb986d117UL,
                                          0x1dddf588UL, 0x71ed85f46UL, 0xa5437d58fUL, 0x4029d1e25UL, 0xc580ec972UL,
                                          0x6847df8baUL, 0xe294d997bUL, 0xe2e8b10eeUL, 0x1593103ddUL, 0x222103857UL,
                                          0x1e035591dUL, 0xb5c9ef2e9UL, 0x9f815ec3eUL, 0xd1da2a021UL, 0x54f171191UL,
                                          0xe51f4a05eUL, 0xc15e7d603UL, 0xba7f16b87UL, 0x80b7a83e1UL, 0x720e2b18dUL,
                                          0x5ec0c069dUL, 0xa4f9f689cUL, 0x5871cafdaUL, 0xc913140a2UL, 0x7a8f2efd1UL,
                                          0x77064952cUL, 0x4ea2d857fUL, 0x484523555UL, 0x54971a9e3UL, 0xeb0694eb2UL,
                                          0xb513c8e63UL, 0x5c910db58UL, 0xca87a4dd7UL, 0xb8ca63158UL, 0xb4b09431dUL,
                                          0x3dc9d50b7UL, 0x7d57f02acUL, 0x5c595b1b2UL, 0x9e0caf698UL, 0x136b48555UL,
                                          0x687dbcc2bUL, 0x54bae2294UL, 0x6899bbd7bUL, 0x8108f46deUL, 0x1dbe8cf08UL,
                                          0xa02e1ae1dUL, 0xf5f26d59UL, 0x805cf202bUL, 0xafede5687UL, 0x1583d5b30UL,
                                          0xda9ed0620UL, 0xcf1237338UL, 0x3a5a77bc4UL, 0xa17ffa0c6UL, 0x29de4c387UL,
                                          0x7825d431UL, 0x2d7b9b38UL, 0x8ed0f26aaUL, 0x56e54e30dUL, 0x9620ab0e7UL,
                                          0xc7e3ea94cUL, 0xd288a41e2UL, 0xf68884f1eUL, 0x5ee02df09UL, 0xc02dbf645UL,
                                          0xeac4c2424UL, 0xcab2d51e1UL, 0x37439577UL, 0x5618ada43UL, 0x2683b5859UL,
                                          0x8a607c1ceUL, 0x795fd9198UL, 0xb3edb11b8UL, 0x846939c5cUL, 0x8b1f6fa23UL,
                                          0xb1a2f2bfeUL, 0xb63a07ad7UL, 0x5f8ea7b00UL, 0x4ee9c6d0cUL, 0x990f2889bUL,
                                          0xb7f7251d0UL, 0xac3291369UL, 0x9d8f36a7bUL, 0xd57342897UL, 0xefca98365UL,
                                          0xdacc69f0eUL, 0x3a70e4b3cUL, 0x1e95c34c2UL, 0x4caab6c06UL, 0x7231f6ee1UL,
                                          0x37909aa04UL, 0x48c9a9ccUL, 0x59cd081bcUL, 0x4dd78c2e4UL, 0x4979da10fUL,
                                          0x4749d0c5UL, 0xa17a4283bUL, 0xde7e1d52dUL, 0xe47cedf1UL, 0x4fa48cbffUL,
                                          0x545a932a0UL, 0x6c2bd9eb8UL, 0xdd9bd3b8cUL, 0x43332c1baUL, 0x501fa761dUL,
                                          0x7ec40adbbUL, 0x4049f2b33UL, 0xcde28f57bUL, 0xf68c804b9UL, 0x8f50fbd3eUL,
                                          0x54e1bc344UL, 0x36b26e3a2UL, 0x2e5ac9b1UL, 0x10837858dUL, 0x6ccac9e0bUL,
                                          0x625ba8a52UL, 0xac4c8b45cUL, 0x868678237UL, 0x4187235feUL, 0xbd62663ceUL,
                                          0xea832dfb2UL, 0xd5a72f0a7UL, 0x659c855eUL, 0xbea7f5e48UL, 0xff9566715UL,
                                          0x1bd06d99aUL, 0x9666c578cUL, 0xc6527d3ecUL, 0xb541f3c61UL, 0x678a9ad70UL,
                                          0x36eaadfa3UL, 0xaf74b01deUL, 0x54cc3cdc3UL, 0xd2e587ce6UL, 0x8694b9349UL,
                                          0xd309898feUL, 0x5c3250e09UL, 0x84dcac28eUL, 0xf72add2dfUL, 0x1901681a3UL,
                                          0x9e6a8fd4UL, 0x12f614cd1UL, 0x6d7801ac4UL, 0x14cf1ca54UL, 0x12a7eb608UL,
                                          0x5e7a3bf62UL, 0xba5056a2UL, 0x5bee44c9bUL, 0x819d7dc86UL, 0x62adc8fdUL,
                                          0xbd3155d41UL, 0xcd8c6b38aUL, 0xe320fd50eUL, 0xe189d6655UL, 0x6863c2831UL,
                                          0xd2b9058fUL, 0x23bfad8faUL, 0x199bd1216UL, 0x56138afd7UL, 0xface83a93UL,
                                          0x9554da725UL, 0x9b614dd91UL, 0x98acbca3fUL, 0xd5f0d5f21UL, 0xeb59039e1UL,
                                          0x51d1ec82aUL, 0xa366ef3baUL, 0x1ad0e01f0UL, 0x7f038ad0bUL, 0x3ee055321UL,
                                          0x3bf2dcbb7UL, 0x210e9856cUL, 0xe4fea8231UL, 0xb89444937UL, 0x58852cc34UL,
                                          0x1ee29eea9UL, 0xb919c79f2UL, 0xddc44d3adUL, 0xddcbd4777UL, 0x3c3982ba1UL,
                                          0xdc8ebc45dUL, 0x8b97712b1UL, 0x9702ea21eUL, 0x1f457e726UL, 0x27c6f6e26UL,
                                          0xa9797770UL, 0xd7615f53bUL, 0x74f1cb6e1UL, 0xa32e4d7dcUL, 0x2e89afd1dUL,
                                          0xb03704d5UL, 0xcca58aab0UL, 0x1e5749225UL, 0x6e63a36baUL, 0x562992099UL,
                                          0x64701b950UL, 0xf94ed6196UL, 0xb3441b5f1UL, 0xc64fac247UL, 0xd72ebd98bUL,
                                          0xfa1985b23UL, 0x2df788358UL, 0x88838b488UL, 0x6091032b4UL, 0x25ff2d736UL,
                                          0xdce63d3d5UL, 0xbb5970414UL, 0x44d8b5ffeUL, 0xe1a5666d8UL, 0xe34129125UL,
                                          0xe23854b1UL, 0x1b2a6dbeUL, 0xd11507bcdUL, 0x844531e6bUL, 0xd864a8611UL,
                                          0xe2a5a7700UL, 0x2d178962aUL, 0x156b07f01UL, 0x48b59fec3UL, 0x3d3d9d79cUL,
                                          0x1846fb339UL, 0xddf1d03caUL, 0x998abaf9UL, 0xc9d76190bUL, 0x67354a1a8UL,
                                          0xcc89e2b09UL, 0x353356834UL, 0x7ad97470eUL, 0xf4d560524UL, 0x534b7804eUL,
                                          0x14290c632UL, 0xb67d39d60UL, 0x35b166febUL, 0x88e6fb681UL, 0xa0f82ae1aUL,
                                          0x8460ce52UL, 0x8b06a9012UL, 0xdaf1299dcUL, 0x629ab696cUL, 0x3113b448aUL,
                                          0xdb5ca215UL, 0x3e00b1e2dUL, 0x85a87f5abUL, 0xb3995ff20UL, 0x85661554dUL,
                                          0xe709c5384UL, 0x111ca99bUL, 0x49e614279UL, 0xf14677ec4UL, 0x8f6439bfbUL,
                                          0x749faa461UL, 0x1c4f9189aUL, 0xe8e9015caUL, 0xf6e68d510UL, 0xb3819319fUL,
                                          0xda9f7119fUL, 0x7787f40f8UL, 0xbc57f5716UL, 0x60ff2897eUL, 0xb3a28a934UL,
                                          0x10b34c97cUL, 0xc14f53aedUL, 0xd3c4eaf5dUL, 0xb3148d39eUL, 0x7874ea02UL,
                                          0xf86692b4aUL, 0x5b03a0e8dUL, 0xce6db8cc6UL, 0x8233d5908UL, 0xf163e3c06UL,
                                          0xdff854cceUL, 0x26706f1bcUL, 0x94c358653UL, 0x7384c9821UL, 0xe51b8e5d5UL,
                                          0xeda32963bUL, 0xa073f392fUL, 0xc3ccfa213UL, 0x34adf5216UL, 0xcb8da286bUL,
                                          0x3b5fbbf08UL, 0x12812d1f8UL, 0xc97c54c39UL, 0xe1c3e36b9UL, 0xabb8dc0edUL,
                                          0x19dcbbf6UL, 0x25b0d7c4dUL, 0x45e6b5ceUL, 0x17dc086caUL, 0xc3f425e6bUL,
                                          0x6fdee14f8UL, 0x39155e6b4UL, 0xa191ec15UL, 0x398fcd7f4UL, 0xa6e2b0594UL,
                                          0xfe5678d82UL, 0xe317eba1fUL, 0x2c4f10ca1UL, 0xae239c19eUL, 0x18e663ed2UL,
                                          0x4a040b7e7UL, 0xbbca0849cUL, 0xce05b3a74UL, 0x7cee982fdUL, 0x78ee54fa7UL,
                                          0x7b47bb0bdUL, 0x7e8f19216UL, 0xd67d91cedUL, 0xef5effe94UL, 0xec1d1938dUL,
                                          0x4c05ef70eUL, 0x324442d9UL, 0xfb0183bb4UL, 0xfb7a0bd50UL, 0x89aa17d87UL,
                                          0xe4e6aed89UL, 0xdbecf68b4UL, 0x683770de4UL, 0xb9f41a136UL, 0xc7614caceUL,
                                          0x89c298386UL, 0x959cf09deUL, 0xab30b19e3UL, 0xdb2e4b614UL, 0x26d30d39bUL,
                                          0x6ccefe452UL, 0x587c5035cUL, 0xea73bbbe0UL, 0xdd9d91a11UL, 0xdd8c5e851UL,
                                          0xe8b4aa077UL, 0x8ccf8faddUL, 0x47ddd3c0bUL, 0x635a92f19UL, 0xf0edfd1a3UL,
                                          0x1f760bf5eUL, 0xa83feb68aUL, 0x4f74da9ddUL, 0x52f759252UL, 0x98bee689eUL,
                                          0xc5fc8c3d5UL, 0x8373d1286UL, 0xf5f1cdabdUL, 0xada68d3e5UL, 0x3bbb9eb5eUL,
                                          0x50cde8478UL, 0xf01f956e0UL, 0xa922f2842UL, 0x233a8b25aUL, 0x71118b754UL,
                                          0xb7f874552UL, 0x44d757121UL, 0xb873b14ccUL, 0x5bcc1db5cUL, 0xbf9b895ceUL,
                                          0x5e65bb620UL, 0xbbd1ed35cUL, 0x358e79973UL, 0x62aa5a4a5UL, 0x81715fc0fUL,
                                          0x8df03a76eUL, 0x376b7c6c7UL, 0xa07a49f2eUL, 0x45e159b63UL, 0xdae5706b0UL,
                                          0xb5e52c7ccUL, 0x206935e8eUL, 0x39f0c5119UL, 0x3cd58c574UL, 0x571986d35UL,
                                          0xad66da60fUL, 0x2b1a6315UL, 0xd0131b533UL, 0x741a195c5UL, 0xb8663437UL,
                                          0x1cde52798UL, 0x6b8e658b1UL, 0xb43c0d44dUL, 0x45481d697UL, 0x29de93df5UL,
                                          0x10549b874UL, 0xc056b5828UL, 0x3fa830adUL, 0x9496d14faUL, 0xf540592a0UL,
                                          0xf31c8b855UL, 0x64f2ba36bUL, 0xfe7c6e4f5UL, 0x5e42a78b0UL, 0x9c2b8b096UL,
                                          0xdcb4a6e71UL, 0xd63b0e7edUL, 0xde1bcbcdaUL, 0x68e7161f2UL, 0x3e5ddf88dUL,
                                          0x419a37501UL, 0xfad63e7abUL, 0xc6e81b4baUL, 0x8329315d3UL, 0xc88d267e6UL,
                                          0x73a0ac25fUL, 0xe7b75690fUL, 0xdcbb95be2UL, 0x7a1d2a059UL, 0xd8fac361eUL,
                                          0x6312ff5c9UL, 0xd2cf50d54UL, 0x8c65fd00fUL, 0xaa1636532UL, 0x870c7285dUL,
                                          0x1894f0b84UL, 0x4260cc5c3UL, 0xe9997b9ecUL, 0x87a052144UL, 0x8706babf6UL,
                                          0xbd5f62ad3UL, 0x1a7895439UL, 0xf7e294bbcUL, 0xbcc27ca26UL, 0x3186a63d4UL,
                                          0x7f3ede4a4UL, 0xb64e32468UL, 0x71f250d53UL, 0x7c6513783UL, 0xb1778714aUL,
                                          0x94bf2c57fUL, 0x64a9f893aUL, 0x1305be654UL, 0x493e0c9f6UL, 0x5ba6fed8UL,
                                          0xc4a0c7a06UL, 0xcc2ec0ddUL, 0xd9a6769afUL, 0x724c78a49UL, 0xc85c981a4UL,
                                          0x12553c4cdUL, 0x83cb892b1UL, 0xbc324ccc7UL, 0xef43f6c1dUL, 0x2d6748bb7UL,
                                          0x5efdce2d7UL, 0x94af64f28UL, 0xf9d58feb3UL, 0xcf547ac63UL, 0xceb309febUL,
                                          0x30beba8caUL, 0x8ab2e486aUL, 0x4a95d58adUL, 0x25ce07c46UL, 0x712b93fd7UL,
                                          0x7f46acc81UL, 0x64049d4beUL, 0x65303aa09UL, 0xf3aad21b3UL, 0x2903a6cd0UL,
                                          0x5a0e0467dUL, 0x3c4fa64e4UL, 0x5c6655126UL, 0xb40a2a67fUL, 0xb0c22c6e5UL,
                                          0x1507e039bUL, 0xb282b16b8UL, 0xc0e14a3d3UL, 0x93d381427UL, 0x6bb55bb87UL,
                                          0xb675af72fUL, 0xfceb4f95eUL, 0x66af6ebbdUL, 0x20a44d1f2UL, 0x6bc873916UL,
                                          0xb8947bee8UL, 0x4b6bed8a6UL, 0x7012f7867UL, 0x7eda3c150UL, 0xab3ef1b8eUL,
                                          0x6d71466eeUL, 0x408c4e225UL, 0xe117838b1UL, 0xaef3a075UL, 0x5a0779d4fUL,
                                          0x70a3b1d69UL, 0x26ccd31fdUL, 0xed64dd1b2UL, 0x981d4f60cUL, 0x6a6e4fb61UL,
                                          0x52f15fc93UL, 0x32b3a64dUL, 0xecb17d667UL, 0xa983fb935UL, 0x37d23c88dUL,
                                          0xb8590fbcbUL, 0xec2f1a277UL, 0x90d3053e6UL, 0xa36fa8ccdUL, 0x44bd08eccUL,
                                          0x61dd197d9UL, 0xa307cfd82UL, 0x1d09c2de4UL, 0x5f6d74368UL, 0x1327d1b2dUL,
                                          0x594cc36b9UL, 0xfea1cba7cUL, 0x50c31262dUL, 0xd99b1a6baUL, 0x1bf789cd2UL,
                                          0xe2f6f66f9UL, 0x13d5edfc6UL, 0xbc3a9ab0cUL, 0x1da5b2734UL, 0x25ef4f2deUL,
                                          0xdcb55a50aUL, 0x9c6dbc6acUL, 0x89a838853UL, 0x168f099eeUL, 0xd51601760UL,
                                          0x89f324f1aUL, 0x2cb1ec1eaUL, 0x6306de366UL, 0x12a2f11eUL, 0xb5c0bf797UL,
                                          0x5c5f02be4UL, 0x5019f54beUL, 0x6ae4a096aUL, 0x4bce78778UL, 0x94b65b97fUL,
                                          0xd3f6e7bd2UL, 0x1fbd2a84cUL, 0x6d0127ab1UL, 0x3e82799aaUL, 0x4c1264dfeUL,
                                          0xcf69c9360UL, 0x4b43e5342UL, 0x35d1f0372UL, 0xd78c18eb4UL, 0x262574101UL,
                                          0xc2c5c7335UL, 0xbad04051aUL, 0x1c481f94eUL, 0x3285aa0deUL, 0x8973e1f69UL,
                                          0x5d238c694UL, 0x7b71847b9UL, 0x242f5675cUL, 0xcc5751c2dUL, 0xe09bc620bUL,
                                          0xe4e904ddUL, 0x7ca4f1a7UL, 0x2ac79ae43UL, 0xe213d4250UL, 0xd4137c2b5UL,
                                          0xddfce11bcUL, 0xd1d658566UL, 0x213f5b1bbUL, 0xcd35be0a8UL, 0xcc67d7f91UL,
                                          0x509bde098UL, 0x74d3d8f46UL, 0x51309c970UL, 0x53e2bdf66UL, 0xa5dd3fed3UL,
                                          0xa4e69b212UL, 0xb1d39936dUL, 0x6b6c8926bUL, 0x46540a7b0UL, 0x2eebc599fUL,
                                          0x2e54a283eUL, 0xf9a328a9cUL, 0x7ea9cfc53UL, 0x5cffa2bdbUL, 0x464d16f8eUL,
                                          0xeb09444bcUL, 0x3f341b259UL, 0x4d112b108UL, 0x70cb94242UL, 0x974ed4ffdUL,
                                          0x1084da291UL, 0x85673ca39UL, 0xd4d74766fUL, 0x64a68e1deUL, 0xe35630caeUL,
                                          0x2073229dbUL, 0x63d3a3902UL, 0x31598ee06UL, 0x808d61126UL, 0x29957984UL,
                                          0xd4f5f2649UL, 0x9ec8a706bUL, 0x349981760UL, 0xc93ab23a6UL, 0x2c7aa80daUL,
                                          0x866f102baUL, 0xb15cff7bcUL, 0x66a13a4caUL, 0x54a755048UL, 0xd13fdb8d9UL,
                                          0x16ad5edf3UL, 0xe043bb154UL, 0xcc8755671UL, 0xcf9b2bfd5UL, 0x3608890b4UL,
                                          0x330fef315UL, 0xe3299ca65UL, 0xb60765e1UL, 0xe9bb17dcUL, 0x95f474d8bUL,
                                          0xe721d3d00UL, 0xd4679e565UL, 0xc80da6113UL, 0x98deeff30UL, 0xc293bb871UL,
                                          0xe79132f48UL, 0xb152dafbbUL, 0x55f6a4386UL, 0xa1b8a4044UL, 0x4f4187b05UL,
                                          0xb17c2ed3UL, 0x95d75ba04UL, 0xbbf12e96dUL, 0x6abd1a52fUL, 0xf300bc991UL,
                                          0xf0a7385d4UL, 0x52964f82aUL, 0xa9962925fUL, 0x613b2eef1UL, 0x5fd2c92a8UL,
                                          0x9ebecd05UL, 0x36002b87aUL, 0x902c79eefUL, 0x394e63c7eUL, 0x133285064UL,
                                          0xf7cfe2d4bUL, 0x4f068522cUL, 0x96fea1a0fUL, 0xc5a927b13UL, 0xe9a2c1994UL,
                                          0x5c53b3803UL, 0xf636b6188UL, 0x7c656e3UL, 0x26af1fc5fUL, 0xec2f40b78UL,
                                          0xfaa1921e5UL, 0x6137a8b30UL, 0x28674f7bUL, 0x3de184e35UL, 0xeeef093e6UL,
                                          0xd44b3dae0UL, 0xbb7ab7d93UL, 0x2ae18c956UL, 0xcde492bd6UL, 0x1cee0216eUL,
                                          0xf1e5830adUL, 0x76f6c3299UL, 0xdea24af84UL, 0x277e75586UL, 0xa17318024UL,
                                          0x5c4739486UL, 0x5e3de4725UL, 0x6f67c9f6dUL, 0x25f42791dUL, 0x3c54d15b3UL,
                                          0xef98d9c32UL, 0x42f64819dUL, 0x16d5fd070UL, 0x63cb98d4fUL, 0x45a3ad27cUL,
                                          0x1b496b0acUL, 0xaa471c42dUL, 0x599346a2UL, 0xdc8d1c2dUL, 0x7498928c1UL,
                                          0xea06e90ffUL, 0xb683baa32UL, 0xf93014e16UL, 0x20575d56eUL, 0x794325589UL,
                                          0x1533e9935UL, 0x86b8bcb70UL, 0xce11faf5dUL, 0x36c0bd318UL, 0xe5e8c1167UL,
                                          0xe1831ba64UL, 0xe088dbfa4UL, 0x984479674UL, 0xafef02b29UL, 0x48518c716UL,
                                          0x4301564ceUL, 0x21cc88710UL, 0xd5c995278UL, 0xd8367de1cUL, 0x4a51125e8UL,
                                          0x113e1c226UL, 0xef141e076UL, 0x44097011dUL, 0x4ca9d707cUL, 0x40d8831f1UL,
                                          0xbd9c3b1d8UL, 0x978364177UL, 0x10f7606a9UL, 0x46a64270aUL, 0x42df1b22bUL,
                                          0xe906cf2a0UL, 0x997da6fa5UL, 0xa5722c26fUL, 0xb14f58aaaUL, 0xafc167ad8UL,
                                          0x37be56e60UL, 0xde7f80d62UL, 0xc3fb0a64UL, 0xce8ca802cUL, 0x35032ed9dUL,
                                          0xaa8ba3ee6UL, 0x94b2e707cUL, 0x2debbdae1UL, 0xf53e25fcfUL, 0xe935543ebUL,
                                          0x1462f0e90UL, 0x54ce7d18cUL, 0x2ddafdc5fUL, 0x700565deeUL, 0xfd408e0afUL,
                                          0x17d089decUL, 0x833ea2459UL, 0x3c8d3776aUL, 0x2e5eebac8UL, 0x20cbf49b0UL,
                                          0xc44675eb7UL, 0x3a4b6beb1UL, 0xce6f37c1eUL, 0x63fba2e7cUL, 0x5a05b553dUL,
                                          0x1286445b0UL, 0x5e07a9b61UL, 0x7d8397ea4UL, 0x8084b7bbbUL, 0xb05b38097UL,
                                          0x29c3019eeUL, 0xed1d2708bUL, 0x9df8a4d47UL, 0xe4891e436UL, 0x2a762ab72UL,
                                          0x92f70600fUL, 0x92329a2cdUL, 0x3e200c6edUL, 0x8c0a7233eUL, 0x60866806aUL,
                                          0xf4fddd24aUL, 0xf78464c71UL, 0x9c3d22242UL, 0x3877ea6d1UL, 0xe2a6d54acUL,
                                          0x497d2a5e7UL, 0xca82f781eUL, 0x481524f4cUL, 0xdee088814UL, 0xb2a82d3a4UL,
                                          0x8e6afe6e5UL, 0xd6279a5daUL, 0x4567cbc1aUL, 0x5bec2b2fdUL, 0x4ef452505UL,
                                          0x61d992cbaUL, 0xab96be0cbUL, 0x708ef35d9UL, 0xb3f6f3623UL, 0x36eb1801dUL,
                                          0xbadfee917UL, 0xa3db13cd0UL, 0x1d1a12828UL, 0x2500816ceUL, 0xcf7612148UL,
                                          0xbe6a3f4bUL, 0x74142f3daUL, 0xce5deed92UL, 0xf9530a786UL, 0x47c8bb38UL,
                                          0xfcabfe88fUL, 0xbc83accb1UL, 0x20cd9fb1fUL, 0x23dcceb3UL, 0x9e969b8c4UL,
                                          0x6e28de934UL, 0x80a399667UL, 0x76a0b85adUL, 0x21a84be3cUL, 0xa28d028b5UL,
                                          0xc4e7690dfUL, 0xbfd9621e8UL, 0x6f4bc0c24UL, 0xaa8e76bd7UL, 0xdeb55dac9UL,
                                          0xbb344fa8bUL, 0xfcaab4decUL, 0x146aba6cbUL, 0xf49ed6eb8UL, 0xdd57e9deaUL,
                                          0x225d5d090UL, 0xd6e86c1c5UL, 0x639be5f39UL, 0xf5e7a6132UL, 0xd2968b09fUL,
                                          0x82b30ba1eUL, 0x803fa46ccUL, 0xc290fab00UL, 0x10df59de5UL, 0x51ae9dcfbUL,
                                          0x49af8516dUL, 0x2b564ce6UL, 0xc615a1de0UL, 0xfef9864a4UL, 0xc16e27341UL,
                                          0x39e846736UL, 0x1ecbb6746UL, 0x588d03a7cUL, 0x10a0eaf9cUL, 0x671ccea6bUL,
                                          0x33a154603UL, 0xa7b003bc1UL, 0xc5fc3848dUL, 0x78e50a9c7UL, 0x17dbfb88eUL,
                                          0x4fd0ed541UL, 0x84221debaUL, 0x3132cf7e6UL, 0xb67e7ac53UL, 0xdf6b28024UL,
                                          0x785b9f7edUL, 0xe3d35320dUL, 0x159c06583UL, 0x5c54a80a3UL, 0xed4d4533bUL,
                                          0xcf16c601aUL, 0x5e94efbd1UL, 0x5d587126eUL, 0xeef2f2807UL, 0x9f3c558eUL,
                                          0x736cfd539UL, 0xf5a922ae1UL, 0x4e2ab9959UL, 0x6a2dd34e7UL, 0x8c9d30d23UL,
                                          0xeba20b791UL, 0xd5c5095e3UL, 0x423d75a82UL, 0x40cebaafeUL, 0x65e08d288UL,
                                          0x2e4f6d767UL, 0xfe10d2f21UL, 0x110347bdaUL, 0xe43a9bfb3UL, 0xcdea483ccUL,
                                          0xfb1e2d8c6UL, 0xd8a0af7a7UL, 0x37d05b182UL, 0x8d1241d83UL, 0xda1ea7b6eUL,
                                          0x65bea93dbUL, 0x2a02f8753UL, 0x454243289UL, 0x4150bc5a2UL, 0xbbabe5911UL,
                                          0x4cbcdbc59UL, 0xf0e61340bUL, 0x30a2cdea8UL, 0x5daecb091UL, 0x5dc93d891UL,
                                          0xc501b4051UL, 0x782cfba78UL, 0x4c191b61eUL, 0xb7e27ef35UL, 0x5a476838UL,
                                          0x9b0209574UL, 0xa775164cfUL, 0xd33d21701UL, 0x3afcb7d45UL, 0x4df2035cdUL,
                                          0x498819a21UL, 0x293f9e506UL, 0x9a35ff1c8UL, 0xc090ebe6bUL, 0xa4f0551d4UL,
                                          0x5dc0dc194UL, 0x1388aeb31UL, 0x340b27bf4UL, 0x3a0f320abUL, 0x996be75dUL,
                                          0xb257ecf39UL, 0x78d86f2f1UL, 0x673f5ff91UL, 0x4538d7e3eUL, 0xde5bc4369UL};
      fromVector(codes, d._code_id);
      d._nbits = 36;
      d._tau = 10;
      d._type = TAG36h10;
      d._name = "TAG36h10";
    }
      break;
    case TAG16h5:
    {
      std::vector<std::uint64_t> codes = {0x231bUL, 0x2ea5UL, 0x346aUL, 0x45b9UL, 0x79a6UL, 0x7f6bUL, 0xb358UL,
                                          0xe745UL, 0xfe59UL, 0x156dUL, 0x380bUL, 0xf0abUL, 0xd84UL, 0x4736UL, 0x8c72UL,
                                          0xaf10UL, 0x93cUL, 0x93b4UL, 0xa503UL, 0x468fUL, 0xe137UL, 0x5795UL, 0xdf42UL,
                                          0x1c1dUL, 0xe9dcUL, 0x73adUL, 0xad5fUL, 0xd530UL, 0x7caUL, 0xaf2eUL};
      fromVector(codes, d._code_id);
      d._nbits = 16;
      d._tau = 5;
      d._type = TAG16h5;
      d._name = "TAG16h5";
    }
      break;
    case TAG25h7:
    {
      std::vector<std::uint64_t> codes = {0x4b770dUL, 0x11693e6UL, 0x1a599abUL, 0xc3a535UL, 0x152aafaUL, 0xaccd98UL,
                                          0x1cad922UL, 0x2c2fadUL, 0xbb3572UL, 0x14a3b37UL, 0x186524bUL, 0xc99d4cUL,
                                          0x23bfeaUL, 0x141cb74UL, 0x1d0d139UL, 0x1670aebUL, 0x851675UL, 0x150334eUL,
                                          0x6e3ed8UL, 0xfd449dUL, 0xaa55ecUL, 0x1c86176UL, 0x15e9b28UL, 0x7ca6b2UL,
                                          0x147c38bUL, 0x1d6c950UL, 0x8b0e8cUL, 0x11a1451UL, 0x1562b65UL, 0x13f53c8UL,
                                          0xd58d7aUL, 0x829ec9UL, 0xfaccf1UL, 0x136e405UL, 0x7a2f06UL, 0x10934cbUL,
                                          0x16a8b56UL, 0x1a6a26aUL, 0xf85545UL, 0x195c2e4UL, 0x24c8a9UL, 0x12bfc96UL,
                                          0x16813aaUL, 0x1a42abeUL, 0x1573424UL, 0x1044573UL, 0xb156c2UL, 0x5e6811UL,
                                          0x1659bfeUL, 0x1d55a63UL, 0x5bf065UL, 0xe28667UL, 0x1e9ba54UL, 0x17d7c5aUL,
                                          0x1f5aa82UL, 0x1a2bbd1UL, 0x1ae9f9UL, 0x1259e51UL, 0x134062bUL, 0xe1177aUL,
                                          0xed07a8UL, 0x162be24UL, 0x59128bUL, 0x1663e8fUL, 0x1a83cbUL, 0x45bb59UL,
                                          0x189065aUL, 0x4bb370UL, 0x16fb711UL, 0x122c077UL, 0xeca17aUL, 0xdbc1f4UL,
                                          0x88d343UL, 0x58ac5dUL, 0xba02e8UL, 0x1a1d9dUL, 0x1c72eecUL, 0x924bc5UL,
                                          0xdccab3UL, 0x886d15UL, 0x178c965UL, 0x5bc69aUL, 0x1716261UL, 0x174e2ccUL,
                                          0x1ed10f4UL, 0x156aa8UL, 0x3e2a8aUL, 0x2752edUL, 0x153c651UL, 0x1741670UL,
                                          0x765b05UL, 0x119c0bbUL, 0x172a783UL, 0x4faca1UL, 0xf31257UL, 0x12441fcUL,
                                          0xd3748UL, 0xc21f15UL, 0xac5037UL, 0x180e592UL, 0x7d3210UL, 0xa27187UL,
                                          0x2beeafUL, 0x26ff57UL, 0x690e82UL, 0x77765cUL, 0x1a9e1d7UL, 0x140be1aUL,
                                          0x1aa1e3aUL, 0x1944f5cUL, 0x19b5032UL, 0x169897UL, 0x1068eb9UL, 0xf30dbcUL,
                                          0x106a151UL, 0x1d53e95UL, 0x1348ceeUL, 0xcf4fcaUL, 0x1728bb5UL, 0xdc1eecUL,
                                          0x69e8dbUL, 0x16e1523UL, 0x105fa25UL, 0x18abb0cUL, 0xc4275dUL, 0x6d8e76UL,
                                          0xe8d6dbUL, 0xe16fd7UL, 0x1ac2682UL, 0x77435bUL, 0xa359ddUL, 0x3a9c4eUL,
                                          0x123919aUL, 0x1e25817UL, 0x2a836UL, 0x1545a4UL, 0x1209c8dUL, 0xbb5f69UL,
                                          0x1dc1f02UL, 0x5d5f7eUL, 0x12d0581UL, 0x13786c2UL, 0xe15409UL, 0x1aa3599UL,
                                          0x139aad8UL, 0xb09d2aUL, 0x54488fUL, 0x13c351cUL, 0x976079UL, 0xb25b12UL,
                                          0x1addb34UL, 0x1cb23aeUL, 0x1175738UL, 0x1303bb8UL, 0xd47716UL, 0x188ceeaUL,
                                          0xbaf967UL, 0x1226d39UL, 0x135e99bUL, 0x34adc5UL, 0x2e384dUL, 0x90d3faUL,
                                          0x232713UL, 0x17d49b1UL, 0xaa84d6UL, 0xc2ddf8UL, 0x1665646UL, 0x4f345fUL,
                                          0x2276b1UL, 0x1255dd7UL, 0x16f4cccUL, 0x4aaffcUL, 0xc46da6UL, 0x85c7b3UL,
                                          0x1311fcbUL, 0x9c6c4fUL, 0x187d947UL, 0x8578e4UL, 0xe2bf0bUL, 0xa01b4cUL,
                                          0xa1493bUL, 0x7ad766UL, 0xccfe82UL, 0x1981b5bUL, 0x1cacc85UL, 0x562cdbUL,
                                          0x15b0e78UL, 0x8f66c5UL, 0x3332bfUL, 0x12ce754UL, 0x96a76UL, 0x1d5e3baUL,
                                          0x27ea41UL, 0x14412dfUL, 0x67b9b4UL, 0xdaa51aUL, 0x1dcb17UL, 0x4d4afdUL,
                                          0x6335d5UL, 0xee2334UL, 0x17d4e55UL, 0x1b8b0f0UL, 0x14999e3UL, 0x1513dfaUL,
                                          0x765cf2UL, 0x56af90UL, 0x12e16acUL, 0x1d3d86cUL, 0xff279bUL, 0x18822ddUL,
                                          0x99d478UL, 0x8dc0d2UL, 0x34b666UL, 0xcf9526UL, 0x186443dUL, 0x7a8e29UL,
                                          0x19c6aa5UL, 0x1f2a27dUL, 0x12b2136UL, 0xd0cd0dUL, 0x12cb320UL, 0x17ddb0bUL,
                                          0x5353bUL, 0x15b2cafUL, 0x1e5a507UL, 0x120f1e5UL, 0x114605aUL, 0x14efe4cUL,
                                          0x568134UL, 0x11b9f92UL, 0x174d2a7UL, 0x692b1dUL, 0x39e4feUL, 0xaaff3dUL,
                                          0x96224cUL, 0x13c9f77UL, 0x110ee8fUL, 0xf17beaUL, 0x99fb5dUL, 0x337141UL,
                                          0x2b54dUL, 0x1233a70UL};
      fromVector(codes, d._code_id);
      d._nbits = 25;
      d._tau = 7;
      d._type = TAG25h7;
      d._name = "TAG25h7";
    }
      break;
    case TAG25h9:
    {
      std::vector<std::uint64_t> codes = {0x155cbf1UL, 0x1e4d1b6UL, 0x17b0b68UL, 0x1eac9cdUL, 0x12e14ceUL, 0x3548bbUL,
                                          0x7757e6UL, 0x1065dabUL, 0x1baa2e7UL, 0xdea688UL, 0x81d927UL, 0x51b241UL,
                                          0xdbc8aeUL, 0x1e50e19UL, 0x15819d2UL, 0x16d8282UL, 0x163e035UL, 0x9d9b81UL,
                                          0x173eec4UL, 0xae3a09UL, 0x5f7c51UL, 0x1a137fcUL, 0xdc9562UL, 0x1802e45UL,
                                          0x1c3542cUL, 0x870fa4UL, 0x914709UL, 0x16684f0UL, 0xc8f2a5UL, 0x833ebbUL,
                                          0x59717fUL, 0x13cd050UL, 0xfa0ad1UL, 0x1b763b0UL, 0xb991ceUL};
      fromVector(codes, d._code_id);
      d._nbits = 25;
      d._tau = 9;
      d._type = TAG25h9;
      d._name = "TAG25h9";
    }
      break;
    case ARUCO_MIP_16h3:
    {
      std::vector<std::uint64_t> codes = {0x5867UL, 0x8b03UL, 0x2537UL, 0xb6c7UL, 0xe45UL, 0x161UL, 0x219UL, 0x859bUL,
                                          0x87UL, 0xc93fUL, 0x905fUL, 0x3e73UL, 0x6ab7UL, 0x1bafUL, 0x6f0fUL, 0x23d3UL,
                                          0x47a5UL, 0x8cf7UL, 0x83cfUL, 0x9205UL, 0x29a5UL, 0x8033UL, 0x857dUL,
                                          0xa4afUL, 0x422fUL, 0x1d07UL, 0x4ee3UL, 0x64c5UL, 0xaa7fUL, 0x4b75UL,
                                          0x34dbUL, 0x926UL, 0x262UL, 0x501dUL, 0x415UL, 0x6201UL, 0x2064UL, 0x2d5UL,
                                          0x10bUL, 0x9427UL, 0xc16bUL, 0xa603UL, 0x911UL, 0x1043UL, 0x87bUL, 0xccfUL,
                                          0x162bUL, 0x9ab3UL, 0x30b7UL, 0xad0bUL, 0x60a6UL, 0x3845UL, 0xce2bUL,
                                          0xadc7UL, 0x612UL, 0x4253UL, 0x9cc3UL, 0xc23UL, 0x409bUL, 0x8e87UL, 0x98e5UL,
                                          0x20f1UL, 0xa807UL, 0x1bfUL, 0x7023UL, 0xdf1UL, 0x2957UL, 0x26b3UL, 0xd80fUL,
                                          0x4076UL, 0x233bUL, 0x32e3UL, 0x7a85UL, 0x4349UL, 0xc857UL, 0x41c6UL,
                                          0x3813UL, 0x6d97UL, 0x324fUL, 0xe3fUL, 0x47ffUL, 0x2217UL, 0xdd6fUL, 0x48b1UL,
                                          0x8b95UL, 0xd9c7UL, 0x1a7dUL, 0x867bUL, 0xb7ffUL, 0x7fa7UL, 0x478bUL,
                                          0x6d43UL, 0x6167UL, 0x8f67UL, 0xcda7UL, 0x5cb7UL, 0xf3afUL, 0x889fUL,
                                          0x2fdfUL, 0xd2e7UL, 0x553UL, 0xbc1fUL, 0x7607UL, 0x2a61UL, 0x11ebUL, 0xa457UL,
                                          0x789fUL, 0x2e95UL, 0xb46bUL, 0x39ffUL, 0x4fb6UL, 0x647fUL, 0x250dUL,
                                          0x5883UL, 0xc5d7UL, 0xe73fUL, 0x129fUL, 0x548fUL, 0xb253UL, 0x635fUL,
                                          0x1ed7UL, 0xc647UL, 0xa1f7UL, 0x565fUL, 0x7e6fUL, 0xe10fUL, 0xf56UL, 0xf2dfUL,
                                          0x87a3UL, 0x4b87UL, 0x24f6UL, 0xafb7UL, 0x3bc7UL, 0x51a7UL, 0x6a47UL,
                                          0x4c46UL, 0x6723UL, 0x2787UL, 0x2667UL, 0x2f13UL, 0x50d7UL, 0x680dUL,
                                          0x7ad3UL, 0x454fUL, 0x6c35UL, 0xf17fUL, 0x6b91UL, 0x4011UL, 0x16UL, 0x2022UL,
                                          0xa1UL, 0x142UL, 0x40fUL, 0x1025UL, 0xcbUL, 0x45UL, 0x4427UL, 0x8491UL,
                                          0x2292UL, 0x482bUL, 0x8221UL, 0x8293UL, 0xf7UL, 0x7dUL, 0x601UL, 0x84a5UL,
                                          0x2443UL, 0x806fUL, 0x21c1UL, 0x4825UL, 0x466UL, 0x742dUL, 0xca07UL, 0x20bbUL,
                                          0x88a3UL, 0x2251UL, 0xa81UL, 0x202dUL, 0x205bUL, 0x1a07UL, 0x8119UL, 0x6413UL,
                                          0x6105UL, 0x820bUL, 0x8715UL, 0x1c0bUL, 0xb003UL, 0x307UL, 0x147bUL, 0x135UL,
                                          0x62a3UL, 0x6c0bUL, 0x14b3UL, 0x5c05UL, 0xd81UL, 0x36dUL, 0x8d3UL, 0x8ba7UL,
                                          0xb25UL, 0x4c5bUL, 0x2867UL, 0x242bUL, 0x1237UL, 0x4206UL, 0xa0c3UL, 0xa4f3UL,
                                          0x984bUL, 0x8507UL, 0x4529UL, 0xf0dUL, 0xb2bUL, 0x2891UL, 0x1a3bUL, 0x234bUL,
                                          0x8d43UL, 0x22e6UL, 0x7245UL, 0x1c77UL, 0x825dUL, 0x3487UL, 0x60d6UL,
                                          0x5403UL, 0x90e3UL, 0xa43UL, 0x6519UL, 0x6169UL, 0xc397UL, 0x2285UL, 0xc127UL,
                                          0x2c05UL, 0x8871UL, 0x5a0bUL, 0x4e26UL, 0x15afUL, 0xf97UL, 0x258bUL, 0x463bUL,
                                          0x86c3UL, 0x292fUL, 0x9e3UL, 0x2571UL, 0x4ce5UL, 0x81d5UL, 0x459UL, 0x40e2UL,
                                          0x5b6UL, 0x4a33UL, 0x837UL, 0xa2a7UL, 0xf4bUL};
      fromVector(codes, d._code_id);
      d._nbits = 16;
      d._tau = 3;
      d._type = ARUCO_MIP_16h3;
      d._name = "ARUCO_MIP_16h3";
    }
      break;

    case ARUCO_MIP_25h7:
    {
      std::vector<std::uint64_t> codes = {0x1ad2487UL, 0x141b6e9UL, 0x1780e66UL, 0x1a86ad8UL, 0x1146916UL, 0x14ebcb6UL,
                                          0x1a1893cUL, 0xd79525UL, 0xf8db53UL, 0x1b5928UL, 0x127bf52UL, 0x1cb3a02UL,
                                          0x145ed8cUL, 0x3970efUL, 0xc8934cUL, 0x5badedUL, 0x53c9daUL, 0x1991f79UL,
                                          0x1d40a4dUL, 0x1631993UL, 0x74d97cUL, 0x62c8a5UL, 0x1a6ce96UL, 0x1c71456UL,
                                          0x18d8df2UL, 0xe290faUL, 0x494061UL, 0xedc6cdUL, 0x12bd8f0UL, 0x1a5d254UL,
                                          0x172eb3aUL, 0x9d6e92UL, 0x10a2172UL, 0x1bd085aUL, 0x186e528UL, 0xb4af45UL,
                                          0x55a5b2UL, 0x5e5136UL, 0x88508bUL, 0xea2c6fUL, 0x1a9274cUL, 0xbbb977UL,
                                          0x5ede62UL, 0x105ba85UL, 0x18a79bcUL, 0x18d4b9dUL, 0x1a144deUL, 0x1ba5390UL,
                                          0x1b8a652UL, 0x1c4948dUL, 0x3ce285UL, 0xb5cce7UL, 0x16d2cb9UL, 0xb7d338UL,
                                          0xb650e9UL, 0x130e4b1UL, 0xd3badcUL, 0xf2161dUL, 0x17a45a7UL, 0x14c5c50UL,
                                          0x14be8abUL, 0x1230de9UL, 0xf7c242UL, 0x508697UL, 0x14b1cc5UL, 0x2c1e94UL,
                                          0xf89469UL, 0xdc52d8UL, 0x6eb5c4UL, 0xd8ea08UL, 0x16c779dUL, 0x1770dbeUL,
                                          0x90a4ecUL, 0x1bf212eUL, 0x1a4b99aUL, 0x1d2d3a9UL, 0xf4f5aeUL, 0x842e89UL,
                                          0x1c0e8d4UL, 0x1af4a4cUL, 0x189044bUL, 0xdd3108UL, 0x1ab639eUL, 0x18836c5UL,
                                          0x5f5ad3UL, 0x1410f5dUL, 0xeb532fUL, 0xbda0baUL, 0x9edf2dUL, 0xd5ae4eUL,
                                          0x165d1b8UL, 0x187d07aUL, 0x1dd616dUL, 0x912527UL, 0x940c3cUL, 0x516b8dUL,
                                          0xc1da19UL, 0xc7db85UL, 0x144cb34UL, 0xcd7a34UL};
      fromVector(codes, d._code_id);
      d._nbits = 25;
      d._tau = 7;
      d._type = ARUCO_MIP_25h7;
      d._name = "ARUCO_MIP_25h7";
    }
      break;

    case ARUCO_MIP_36h12:
    {
      std::vector<std::uint64_t> codes = {0xd2b63a09dUL, 0x6001134e5UL, 0x1206fbe72UL, 0xff8ad6cb4UL, 0x85da9bc49UL,
                                          0xb461afe9cUL, 0x6db51fe13UL, 0x5248c541fUL, 0x8f34503UL, 0x8ea462eceUL,
                                          0xeac2be76dUL, 0x1af615c44UL, 0xb48a49f27UL, 0x2e4e1283bUL, 0x78b1f2fa8UL,
                                          0x27d34f57eUL, 0x89222fff1UL, 0x4c1669406UL, 0xbf49b3511UL, 0xdc191cd5dUL,
                                          0x11d7c3f85UL, 0x16a130e35UL, 0xe29f27effUL, 0x428d8ae0cUL, 0x90d548477UL,
                                          0x2319cbc93UL, 0xc3b0c3dfcUL, 0x424bccc9UL, 0x2a081d630UL, 0x762743d96UL,
                                          0xd0645bf19UL, 0xf38d7fd60UL, 0xc6cbf9a10UL, 0x3c1be7c65UL, 0x276f75e63UL,
                                          0x4490a3f63UL, 0xda60acd52UL, 0x3cc68df59UL, 0xab46f9daeUL, 0x88d533d78UL,
                                          0xb6d62ec21UL, 0xb3c02b646UL, 0x22e56d408UL, 0xac5f5770aUL, 0xaaa993f66UL,
                                          0x4caa07c8dUL, 0x5c9b4f7b0UL, 0xaa9ef0e05UL, 0x705c5750UL, 0xac81f545eUL,
                                          0x735b91e74UL, 0x8cc35cee4UL, 0xe44694d04UL, 0xb5e121de0UL, 0x261017d0fUL,
                                          0xf1d439eb5UL, 0xa1a33ac96UL, 0x174c62c02UL, 0x1ee27f716UL, 0x8b1c5ece9UL,
                                          0x6a05b0c6aUL, 0xd0568dfcUL, 0x192d25e5fUL, 0x1adbeccc8UL, 0xcfec87f00UL,
                                          0xd0b9dde7aUL, 0x88dcef81eUL, 0x445681cb9UL, 0xdbb2ffc83UL, 0xa48d96df1UL,
                                          0xb72cc2e7dUL, 0xc295b53fUL, 0xf49832704UL, 0x9968edc29UL, 0x9e4e1af85UL,
                                          0x8683e2d1bUL, 0x810b45c04UL, 0x6ac44bfe2UL, 0x645346615UL, 0x3990bd598UL,
                                          0x1c9ed0f6aUL, 0xc26729d65UL, 0x83993f795UL, 0x3ac05ac5dUL, 0x357adff3bUL,
                                          0xd5c05565UL, 0x2f547ef44UL, 0x86c115041UL, 0x640fd9e5fUL, 0xce08bbcf7UL,
                                          0x109bb343eUL, 0xc21435c92UL, 0x35b4dfce4UL, 0x459752cf2UL, 0xec915b82cUL,
                                          0x51881eed0UL, 0x2dda7dc97UL, 0x2e0142144UL, 0x42e890f99UL, 0x9a8856527UL,
                                          0x8e80d9d80UL, 0x891cbcf34UL, 0x25dd82410UL, 0x239551d34UL, 0x8fe8f0c70UL,
                                          0x94106a970UL, 0x82609b40cUL, 0xfc9caf36UL, 0x688181d11UL, 0x718613c08UL,
                                          0xf1ab7629UL, 0xa357bfc18UL, 0x4c03b7a46UL, 0x204dedce6UL, 0xad6300d37UL,
                                          0x84cc4cd09UL, 0x42160e5c4UL, 0x87d2adfa8UL, 0x7850e7749UL, 0x4e750fc7cUL,
                                          0xbf2e5dfdaUL, 0xd88324da5UL, 0x234b52f80UL, 0x378204514UL, 0xabdf2ad53UL,
                                          0x365e78ef9UL, 0x49caa6ca2UL, 0x3c39ddf3UL, 0xc68c5385dUL, 0x5bfcbbf67UL,
                                          0x623241e21UL, 0xabc90d5ccUL, 0x388c6fe85UL, 0xda0e2d62dUL, 0x10855dfe9UL,
                                          0x4d46efd6bUL, 0x76ea12d61UL, 0x9db377d3dUL, 0xeed0efa71UL, 0xe6ec3ae2fUL,
                                          0x441faee83UL, 0xba19c8ff5UL, 0x313035eabUL, 0x6ce8f7625UL, 0x880dab58dUL,
                                          0x8d3409e0dUL, 0x2be92ee21UL, 0xd60302c6cUL, 0x469ffc724UL, 0x87eebeed3UL,
                                          0x42587ef7aUL, 0x7a8cc4e52UL, 0x76a437650UL, 0x999e41ef4UL, 0x7d0969e42UL,
                                          0xc02baf46bUL, 0x9259f3e47UL, 0x2116a1dc0UL, 0x9f2de4d84UL, 0xeffac29UL,
                                          0x7b371ff8cUL, 0x668339da9UL, 0xd010aee3fUL, 0x1cd00b4c0UL, 0x95070fc3bUL,
                                          0xf84c9a770UL, 0x38f863d76UL, 0x3646ff045UL, 0xce1b96412UL, 0x7a5d45da8UL,
                                          0x14e00ef6cUL, 0x5e95abfd8UL, 0xb2e9cb729UL, 0x36c47dd7UL, 0xb8ee97c6bUL,
                                          0xe9e8f657UL, 0xd4ad2ef1aUL, 0x8811c7f32UL, 0x47bde7c31UL, 0x3adadfb64UL,
                                          0x6e5b28574UL, 0x33e67cd91UL, 0x2ab9fdd2dUL, 0x8afa67f2bUL, 0xe6a28fc5eUL,
                                          0x72049cdbdUL, 0xae65dac12UL, 0x1251a4526UL, 0x1089ab841UL, 0xe2f096ee0UL,
                                          0xb0caee573UL, 0xfd6677e86UL, 0x444b3f518UL, 0xbe8b3a56aUL, 0x680a75cfcUL,
                                          0xac02baea8UL, 0x97d815e1cUL, 0x1d4386e08UL, 0x1a14f5b0eUL, 0xe658a8d81UL,
                                          0xa3868efa7UL, 0x3668a9673UL, 0xe8fc53d85UL, 0x2e2b7edd5UL, 0x8b2470f13UL,
                                          0xf69795f32UL, 0x4589ffc8eUL, 0x2e2080c9cUL, 0x64265f7dUL, 0x3d714dd10UL,
                                          0x1692c6ef1UL, 0x3e67f2f49UL, 0x5041dad63UL, 0x1a1503415UL, 0x64c18c742UL,
                                          0xa72eec35UL, 0x1f0f9dc60UL, 0xa9559bc67UL, 0xf32911d0dUL, 0x21c0d4ffcUL,
                                          0xe01cef5b0UL, 0x4e23a3520UL, 0xaa4f04e49UL, 0xe1c4fcc43UL, 0x208e8f6e8UL,
                                          0x8486774a5UL, 0x9e98c7558UL, 0x2c59fb7dcUL, 0x9446a4613UL, 0x8292dcc2eUL,
                                          0x4d61631UL, 0xd05527809UL, 0xa0163852dUL, 0x8f657f639UL, 0xcca6c3e37UL,
                                          0xcb136bc7aUL, 0xfc5a83e53UL, 0x9aa44fc30UL, 0xbdec1bd3cUL, 0xe020b9f7cUL,
                                          0x4b8f35fb0UL, 0xb8165f637UL, 0x33dc88d69UL, 0x10a2f7e4dUL, 0xc8cb5ff53UL,
                                          0xde259ff6bUL, 0x46d070dd4UL, 0x32d3b9741UL, 0x7075f1c04UL, 0x4d58dbea0UL};
      fromVector(codes, d._code_id);
      d._nbits = 36;
      d._tau = 12;
      d._type = ARUCO_MIP_36h12;
      d._name = "ARUCO_MIP_36h12";
    }
      break;
    case CHILITAGS:
    {
      std::vector<std::uint64_t> codes =
          {0x7c765c6e0c3e00UL, 0x7c765c14121e00UL, 0x7c765a06763e00UL, 0x7c765a7c681e00UL, 0x7c764248643e00UL,
           0x7c7642327a1e00UL, 0x7c7644201e3e00UL, 0x7c76445a001e00UL, 0x7c7626702c3800UL, 0x7c76260a321800UL,
           0x7c762018563800UL, 0x7c762062481800UL, 0x7c763856443800UL, 0x7c76382c5a1800UL, 0x7c763e3e3e3800UL,
           0x7c763e44201800UL, 0x7c70346e120000UL, 0x7c7034140c2000UL, 0x7c703206680000UL, 0x7c70327c762000UL,
           0x7c702a487a0000UL, 0x7c702a32642000UL, 0x7c702c20000000UL, 0x7c702c5a1e2000UL, 0x7c704e70320600UL,
           0x7c704e0a2c2600UL, 0x7c704818480600UL, 0x7c704862562600UL, 0x7c7050565a0600UL, 0x7c70502c442600UL,
           0x7c70563e200600UL, 0x7c7056443e2600UL, 0x7c687c6e764400UL, 0x7c687c14686400UL, 0x7c687a060c4400UL,
           0x7c687a7c126400UL, 0x7c6862481e4400UL, 0x7c686232006400UL, 0x7c686420644400UL, 0x7c68645a7a6400UL,
           0x7c680670564200UL, 0x7c68060a486200UL, 0x7c6800182c4200UL, 0x7c680062326200UL, 0x7c6818563e4200UL,
           0x7c68182c206200UL, 0x7c681e3e444200UL, 0x7c681e445a6200UL, 0x7c6e146e687a00UL, 0x7c6e1414765a00UL,
           0x7c6e1206127a00UL, 0x7c6e127c0c5a00UL, 0x7c6e0a48007a00UL, 0x7c6e0a321e5a00UL, 0x7c6e0c207a7a00UL,
           0x7c6e0c5a645a00UL, 0x7c6e6e70487c00UL, 0x7c6e6e0a565c00UL, 0x7c6e6818327c00UL, 0x7c6e68622c5c00UL,
           0x7c6e7056207c00UL, 0x7c6e702c3e5c00UL, 0x7c6e763e5a7c00UL, 0x7c6e7644445c00UL, 0x7c0c5c68625600UL,
           0x7c0c5c127c7600UL, 0x7c0c5a00185600UL, 0x7c0c5a7a067600UL, 0x7c0c424e0a5600UL, 0x7c0c4234147600UL,
           0x7c0c4426705600UL, 0x7c0c445c6e7600UL, 0x7c0c2676425000UL, 0x7c0c260c5c7000UL, 0x7c0c201e385000UL,
           0x7c0c2064267000UL, 0x7c0c38502a5000UL, 0x7c0c382a347000UL, 0x7c0c3e38505000UL, 0x7c0c3e424e7000UL,
           0x7c0a34687c6800UL, 0x7c0a3412624800UL, 0x7c0a3200066800UL, 0x7c0a327a184800UL, 0x7c0a2a4e146800UL,
           0x7c0a2a340a4800UL, 0x7c0a2c266e6800UL, 0x7c0a2c5c704800UL, 0x7c0a4e765c6e00UL, 0x7c0a4e0c424e00UL,
           0x7c0a481e266e00UL, 0x7c0a4864384e00UL, 0x7c0a5050346e00UL, 0x7c0a502a2a4e00UL, 0x7c0a56384e6e00UL,
           0x7c0a5642504e00UL, 0x7c127c68182c00UL, 0x7c127c12060c00UL, 0x7c127a00622c00UL, 0x7c127a7a7c0c00UL,
           0x7c12624e702c00UL, 0x7c1262346e0c00UL, 0x7c1264260a2c00UL, 0x7c12645c140c00UL, 0x7c120676382a00UL,
           0x7c12060c260a00UL, 0x7c12001e422a00UL, 0x7c1200645c0a00UL, 0x7c121850502a00UL, 0x7c12182a4e0a00UL,
           0x7c121e382a2a00UL, 0x7c121e42340a00UL, 0x7c141468061200UL, 0x7c141412183200UL, 0x7c1412007c1200UL,
           0x7c14127a623200UL, 0x7c140a4e6e1200UL, 0x7c140a34703200UL, 0x7c140c26141200UL, 0x7c140c5c0a3200UL,
           0x7c146e76261400UL, 0x7c146e0c383400UL, 0x7c14681e5c1400UL, 0x7c146864423400UL, 0x7c1470504e1400UL,
           0x7c14702a503400UL, 0x7c147638341400UL, 0x7c1476422a3400UL, 0x7a1e5c70321800UL, 0x7a1e5c0a2c3800UL,
           0x7a1e5a18481800UL, 0x7a1e5a62563800UL, 0x7a1e42565a1800UL, 0x7a1e422c443800UL, 0x7a1e443e201800UL,
           0x7a1e44443e3800UL, 0x7a1e266e121e00UL, 0x7a1e26140c3e00UL, 0x7a1e2006681e00UL, 0x7a1e207c763e00UL,
           0x7a1e38487a1e00UL, 0x7a1e3832643e00UL, 0x7a1e3e20001e00UL, 0x7a1e3e5a1e3e00UL, 0x7a1834702c2600UL,
           0x7a18340a320600UL, 0x7a183218562600UL, 0x7a183262480600UL, 0x7a182a56442600UL, 0x7a182a2c5a0600UL,
           0x7a182c3e3e2600UL, 0x7a182c44200600UL, 0x7a184e6e0c2000UL, 0x7a184e14120000UL, 0x7a184806762000UL,
           0x7a18487c680000UL, 0x7a185048642000UL, 0x7a1850327a0000UL, 0x7a1856201e2000UL, 0x7a18565a000000UL,
           0x7a007c70486200UL, 0x7a007c0a564200UL, 0x7a007a18326200UL, 0x7a007a622c4200UL, 0x7a006256206200UL,
           0x7a00622c3e4200UL, 0x7a00643e5a6200UL, 0x7a006444444200UL, 0x7a00066e686400UL, 0x7a000614764400UL,
           0x7a000006126400UL, 0x7a00007c0c4400UL, 0x7a001848006400UL, 0x7a0018321e4400UL, 0x7a001e207a6400UL,
           0x7a001e5a644400UL, 0x7a061470565c00UL, 0x7a06140a487c00UL, 0x7a0612182c5c00UL, 0x7a061262327c00UL,
           0x7a060a563e5c00UL, 0x7a060a2c207c00UL, 0x7a060c3e445c00UL, 0x7a060c445a7c00UL, 0x7a066e6e765a00UL,
           0x7a066e14687a00UL, 0x7a0668060c5a00UL, 0x7a06687c127a00UL, 0x7a0670481e5a00UL, 0x7a067032007a00UL,
           0x7a067620645a00UL, 0x7a06765a7a7a00UL, 0x7a645c765c7000UL, 0x7a645c0c425000UL, 0x7a645a1e267000UL,
           0x7a645a64385000UL, 0x7a644250347000UL, 0x7a64422a2a5000UL, 0x7a6444384e7000UL, 0x7a644442505000UL,
           0x7a6426687c7600UL, 0x7a642612625600UL, 0x7a642000067600UL, 0x7a64207a185600UL, 0x7a64384e147600UL,
           0x7a6438340a5600UL, 0x7a643e266e7600UL, 0x7a643e5c705600UL, 0x7a623476424e00UL, 0x7a62340c5c6e00UL,
           0x7a62321e384e00UL, 0x7a623264266e00UL, 0x7a622a502a4e00UL, 0x7a622a2a346e00UL, 0x7a622c38504e00UL,
           0x7a622c424e6e00UL, 0x7a624e68624800UL, 0x7a624e127c6800UL, 0x7a624800184800UL, 0x7a62487a066800UL,
           0x7a62504e0a4800UL, 0x7a625034146800UL, 0x7a625626704800UL, 0x7a62565c6e6800UL, 0x7a7a7c76260a00UL,
           0x7a7a7c0c382a00UL, 0x7a7a7a1e5c0a00UL, 0x7a7a7a64422a00UL, 0x7a7a62504e0a00UL, 0x7a7a622a502a00UL,
           0x7a7a6438340a00UL, 0x7a7a64422a2a00UL, 0x7a7a0668060c00UL, 0x7a7a0612182c00UL, 0x7a7a00007c0c00UL,
           0x7a7a007a622c00UL, 0x7a7a184e6e0c00UL, 0x7a7a1834702c00UL, 0x7a7a1e26140c00UL, 0x7a7a1e5c0a2c00UL,
           0x7a7c1476383400UL, 0x7a7c140c261400UL, 0x7a7c121e423400UL, 0x7a7c12645c1400UL, 0x7a7c0a50503400UL,
           0x7a7c0a2a4e1400UL, 0x7a7c0c382a3400UL, 0x7a7c0c42341400UL, 0x7a7c6e68183200UL, 0x7a7c6e12061200UL,
           0x7a7c6800623200UL, 0x7a7c687a7c1200UL, 0x7a7c704e703200UL, 0x7a7c70346e1200UL, 0x7a7c76260a3200UL,
           0x7a7c765c141200UL, 0x62565c6e680600UL, 0x62565c14762600UL, 0x62565a06120600UL, 0x62565a7c0c2600UL,
           0x62564248000600UL, 0x625642321e2600UL, 0x625644207a0600UL, 0x6256445a642600UL, 0x62562670480000UL,
           0x6256260a562000UL, 0x62562018320000UL, 0x625620622c2000UL, 0x62563856200000UL, 0x6256382c3e2000UL,
           0x62563e3e5a0000UL, 0x62563e44442000UL, 0x6250346e763800UL, 0x62503414681800UL, 0x625032060c3800UL,
           0x6250327c121800UL, 0x62502a481e3800UL, 0x62502a32001800UL, 0x62502c20643800UL, 0x62502c5a7a1800UL,
           0x62504e70563e00UL, 0x62504e0a481e00UL, 0x625048182c3e00UL, 0x62504862321e00UL, 0x625050563e3e00UL,
           0x6250502c201e00UL, 0x6250563e443e00UL, 0x625056445a1e00UL, 0x62487c6e127c00UL, 0x62487c140c5c00UL,
           0x62487a06687c00UL, 0x62487a7c765c00UL, 0x624862487a7c00UL, 0x62486232645c00UL, 0x62486420007c00UL,
           0x6248645a1e5c00UL, 0x62480670327a00UL, 0x6248060a2c5a00UL, 0x62480018487a00UL, 0x62480062565a00UL,
           0x624818565a7a00UL, 0x6248182c445a00UL, 0x62481e3e207a00UL, 0x62481e443e5a00UL, 0x624e146e0c4200UL,
           0x624e1414126200UL, 0x624e1206764200UL, 0x624e127c686200UL, 0x624e0a48644200UL, 0x624e0a327a6200UL,
           0x624e0c201e4200UL, 0x624e0c5a006200UL, 0x624e6e702c4400UL, 0x624e6e0a326400UL, 0x624e6818564400UL,
           0x624e6862486400UL, 0x624e7056444400UL, 0x624e702c5a6400UL, 0x624e763e3e4400UL, 0x624e7644206400UL,
           0x622c5c68066e00UL, 0x622c5c12184e00UL, 0x622c5a007c6e00UL, 0x622c5a7a624e00UL, 0x622c424e6e6e00UL,
           0x622c4234704e00UL, 0x622c4426146e00UL, 0x622c445c0a4e00UL, 0x622c2676266800UL, 0x622c260c384800UL,
           0x622c201e5c6800UL, 0x622c2064424800UL, 0x622c38504e6800UL, 0x622c382a504800UL, 0x622c3e38346800UL,
           0x622c3e422a4800UL, 0x622a3468185000UL, 0x622a3412067000UL, 0x622a3200625000UL, 0x622a327a7c7000UL,
           0x622a2a4e705000UL, 0x622a2a346e7000UL, 0x622a2c260a5000UL, 0x622a2c5c147000UL, 0x622a4e76385600UL,
           0x622a4e0c267600UL, 0x622a481e425600UL, 0x622a48645c7600UL, 0x622a5050505600UL, 0x622a502a4e7600UL,
           0x622a56382a5600UL, 0x622a5642347600UL, 0x62327c687c1400UL, 0x62327c12623400UL, 0x62327a00061400UL,
           0x62327a7a183400UL, 0x6232624e141400UL, 0x623262340a3400UL, 0x623264266e1400UL, 0x6232645c703400UL,
           0x623206765c1200UL, 0x6232060c423200UL, 0x6232001e261200UL, 0x62320064383200UL, 0x62321850341200UL,
           0x6232182a2a3200UL, 0x62321e384e1200UL, 0x62321e42503200UL, 0x62341468622a00UL, 0x623414127c0a00UL,
           0x62341200182a00UL, 0x6234127a060a00UL, 0x62340a4e0a2a00UL, 0x62340a34140a00UL, 0x62340c26702a00UL,
           0x62340c5c6e0a00UL, 0x62346e76422c00UL, 0x62346e0c5c0c00UL, 0x6234681e382c00UL, 0x62346864260c00UL,
           0x623470502a2c00UL, 0x6234702a340c00UL, 0x62347638502c00UL, 0x623476424e0c00UL, 0x643e5c70562000UL,
           0x643e5c0a480000UL, 0x643e5a182c2000UL, 0x643e5a62320000UL, 0x643e42563e2000UL, 0x643e422c200000UL,
           0x643e443e442000UL, 0x643e44445a0000UL, 0x643e266e762600UL, 0x643e2614680600UL, 0x643e20060c2600UL,
           0x643e207c120600UL, 0x643e38481e2600UL, 0x643e3832000600UL, 0x643e3e20642600UL, 0x643e3e5a7a0600UL,
           0x64383470481e00UL, 0x6438340a563e00UL, 0x64383218321e00UL, 0x643832622c3e00UL, 0x64382a56201e00UL,
           0x64382a2c3e3e00UL, 0x64382c3e5a1e00UL, 0x64382c44443e00UL, 0x64384e6e681800UL, 0x64384e14763800UL,
           0x64384806121800UL, 0x6438487c0c3800UL, 0x64385048001800UL, 0x643850321e3800UL, 0x643856207a1800UL,
           0x6438565a643800UL, 0x64207c702c5a00UL, 0x64207c0a327a00UL, 0x64207a18565a00UL, 0x64207a62487a00UL,
           0x64206256445a00UL, 0x6420622c5a7a00UL, 0x6420643e3e5a00UL, 0x64206444207a00UL, 0x6420066e0c5c00UL,
           0x64200614127c00UL, 0x64200006765c00UL, 0x6420007c687c00UL, 0x64201848645c00UL, 0x642018327a7c00UL,
           0x64201e201e5c00UL, 0x64201e5a007c00UL, 0x64261470326400UL, 0x6426140a2c4400UL, 0x64261218486400UL,
           0x64261262564400UL, 0x64260a565a6400UL, 0x64260a2c444400UL, 0x64260c3e206400UL, 0x64260c443e4400UL,
           0x64266e6e126200UL, 0x64266e140c4200UL, 0x64266806686200UL, 0x6426687c764200UL, 0x642670487a6200UL,
           0x64267032644200UL, 0x64267620006200UL, 0x6426765a1e4200UL, 0x64445c76384800UL, 0x64445c0c266800UL,
           0x64445a1e424800UL, 0x64445a645c6800UL, 0x64444250504800UL, 0x6444422a4e6800UL, 0x644444382a4800UL,
           0x64444442346800UL, 0x64442668184e00UL, 0x64442612066e00UL, 0x64442000624e00UL, 0x6444207a7c6e00UL,
           0x6444384e704e00UL, 0x644438346e6e00UL, 0x64443e260a4e00UL, 0x64443e5c146e00UL, 0x64423476267600UL,
           0x6442340c385600UL, 0x6442321e5c7600UL, 0x64423264425600UL, 0x64422a504e7600UL, 0x64422a2a505600UL,
           0x64422c38347600UL, 0x64422c422a5600UL, 0x64424e68067000UL, 0x64424e12185000UL, 0x644248007c7000UL,
           0x6442487a625000UL, 0x6442504e6e7000UL, 0x64425034705000UL, 0x64425626147000UL, 0x6442565c0a5000UL,
           0x645a7c76423200UL, 0x645a7c0c5c1200UL, 0x645a7a1e383200UL, 0x645a7a64261200UL, 0x645a62502a3200UL,
           0x645a622a341200UL, 0x645a6438503200UL, 0x645a64424e1200UL, 0x645a0668623400UL, 0x645a06127c1400UL,
           0x645a0000183400UL, 0x645a007a061400UL, 0x645a184e0a3400UL, 0x645a1834141400UL, 0x645a1e26703400UL,
           0x645a1e5c6e1400UL, 0x645c14765c0c00UL, 0x645c140c422c00UL, 0x645c121e260c00UL, 0x645c1264382c00UL,
           0x645c0a50340c00UL, 0x645c0a2a2a2c00UL, 0x645c0c384e0c00UL, 0x645c0c42502c00UL, 0x645c6e687c0a00UL,
           0x645c6e12622a00UL, 0x645c6800060a00UL, 0x645c687a182a00UL, 0x645c704e140a00UL, 0x645c70340a2a00UL,
           0x645c76266e0a00UL, 0x645c765c702a00UL, 0x6765c681e5a00UL, 0x6765c12007a00UL, 0x6765a00645a00UL,
           0x6765a7a7a7a00UL, 0x676424e765a00UL, 0x6764234687a00UL, 0x67644260c5a00UL, 0x676445c127a00UL,
           0x67626763e5c00UL, 0x676260c207c00UL, 0x676201e445c00UL, 0x67620645a7c00UL, 0x6763850565c00UL,
           0x676382a487c00UL, 0x6763e382c5c00UL, 0x6763e42327c00UL, 0x6703468006400UL, 0x67034121e4400UL,
           0x67032007a6400UL, 0x670327a644400UL, 0x6702a4e686400UL, 0x6702a34764400UL, 0x6702c26126400UL,
           0x6702c5c0c4400UL, 0x6704e76206200UL, 0x6704e0c3e4200UL, 0x670481e5a6200UL, 0x6704864444200UL,
           0x6705050486200UL, 0x670502a564200UL, 0x6705638326200UL, 0x67056422c4200UL, 0x6687c68642000UL,
           0x6687c127a0000UL, 0x6687a001e2000UL, 0x6687a7a000000UL, 0x668624e0c2000UL, 0x6686234120000UL,
           0x6686426762000UL, 0x668645c680000UL, 0x6680676442600UL, 0x668060c5a0600UL, 0x668001e3e2600UL,
           0x6680064200600UL, 0x66818502c2600UL, 0x668182a320600UL, 0x6681e38562600UL, 0x6681e42480600UL,
           0x66e14687a1e00UL, 0x66e1412643e00UL, 0x66e1200001e00UL, 0x66e127a1e3e00UL, 0x66e0a4e121e00UL,
           0x66e0a340c3e00UL, 0x66e0c26681e00UL, 0x66e0c5c763e00UL, 0x66e6e765a1800UL, 0x66e6e0c443800UL,
           0x66e681e201800UL, 0x66e68643e3800UL, 0x66e7050321800UL, 0x66e702a2c3800UL, 0x66e7638481800UL,
           0x66e7642563800UL, 0x60c5c6e703200UL, 0x60c5c146e1200UL, 0x60c5a060a3200UL, 0x60c5a7c141200UL,
           0x60c4248183200UL, 0x60c4232061200UL, 0x60c4420623200UL, 0x60c445a7c1200UL, 0x60c2670503400UL,
           0x60c260a4e1400UL, 0x60c20182a3400UL, 0x60c2062341400UL, 0x60c3856383400UL, 0x60c382c261400UL,
           0x60c3e3e423400UL, 0x60c3e445c1400UL, 0x60a346e6e0c00UL, 0x60a3414702c00UL, 0x60a3206140c00UL,
           0x60a327c0a2c00UL, 0x60a2a48060c00UL, 0x60a2a32182c00UL, 0x60a2c207c0c00UL, 0x60a2c5a622c00UL,
           0x60a4e704e0a00UL, 0x60a4e0a502a00UL, 0x60a4818340a00UL, 0x60a48622a2a00UL, 0x60a5056260a00UL,
           0x60a502c382a00UL, 0x60a563e5c0a00UL, 0x60a5644422a00UL, 0x6127c6e0a4800UL, 0x6127c14146800UL,
           0x6127a06704800UL, 0x6127a7c6e6800UL, 0x6126248624800UL, 0x61262327c6800UL, 0x6126420184800UL,
           0x612645a066800UL, 0x61206702a4e00UL, 0x612060a346e00UL, 0x6120018504e00UL, 0x61200624e6e00UL,
           0x6121856424e00UL, 0x612182c5c6e00UL, 0x6121e3e384e00UL, 0x6121e44266e00UL, 0x614146e147600UL,
           0x61414140a5600UL, 0x61412066e7600UL, 0x614127c705600UL, 0x6140a487c7600UL, 0x6140a32625600UL,
           0x6140c20067600UL, 0x6140c5a185600UL, 0x6146e70347000UL, 0x6146e0a2a5000UL, 0x61468184e7000UL,
           0x6146862505000UL, 0x61470565c7000UL, 0x614702c425000UL, 0x614763e267000UL, 0x6147644385000UL,
           0x1e5c76207c00UL, 0x1e5c0c3e5c00UL, 0x1e5a1e5a7c00UL, 0x1e5a64445c00UL, 0x1e4250487c00UL, 0x1e422a565c00UL,
           0x1e4438327c00UL, 0x1e44422c5c00UL, 0x1e2668007a00UL, 0x1e26121e5a00UL, 0x1e20007a7a00UL, 0x1e207a645a00UL,
           0x1e384e687a00UL, 0x1e3834765a00UL, 0x1e3e26127a00UL, 0x1e3e5c0c5a00UL, 0x1834763e4200UL, 0x18340c206200UL,
           0x18321e444200UL, 0x1832645a6200UL, 0x182a50564200UL, 0x182a2a486200UL, 0x182c382c4200UL, 0x182c42326200UL,
           0x184e681e4400UL, 0x184e12006400UL, 0x184800644400UL, 0x18487a7a6400UL, 0x18504e764400UL, 0x185034686400UL,
           0x1856260c4400UL, 0x18565c126400UL, 0x7c765a0600UL, 0x7c0c442600UL, 0x7a1e200600UL, 0x7a643e2600UL,
           0x6250320600UL, 0x622a2c2600UL, 0x6438480600UL, 0x6442562600UL, 0x6687a0000UL, 0x612642000UL, 0x0UL,
           0x7a1e2000UL, 0x184e120000UL, 0x18340c2000UL, 0x1e26680000UL, 0x1e5c762000UL, 0x61476443800UL,
           0x6140c5a1800UL, 0x6121e3e3800UL, 0x61264201800UL, 0x60a502c3800UL, 0x60a2a321800UL, 0x60c38563800UL,
           0x60c42481800UL, 0x66e68643e00UL, 0x66e127a1e00UL, 0x668001e3e00UL, 0x6687a001e00UL, 0x6704e0c3e00UL,
           0x67034121e00UL, 0x67626763e00UL, 0x6765c681e00UL, 0x645c704e1400UL, 0x645c0a503400UL, 0x645a18341400UL,
           0x645a622a3400UL, 0x644256261400UL, 0x64422c383400UL, 0x64443e5c1400UL, 0x644444423400UL, 0x64266e6e1200UL,
           0x642614703200UL, 0x642006141200UL, 0x64207c0a3200UL, 0x643848061200UL, 0x643832183200UL, 0x643e207c1200UL,
           0x643e5a623200UL, 0x623470502a00UL, 0x62340a4e0a00UL, 0x6232182a2a00UL, 0x623262340a00UL, 0x622a56382a00UL,
           0x622a2c260a00UL, 0x622c3e422a00UL, 0x622c445c0a00UL, 0x624e6e702c00UL, 0x624e146e0c00UL, 0x6248060a2c00UL,
           0x62487c140c00UL, 0x625048182c00UL, 0x625032060c00UL, 0x625620622c00UL, 0x62565a7c0c00UL, 0x7a7c70346e00UL,
           0x7a7c0a2a4e00UL, 0x7a7a184e6e00UL, 0x7a7a62504e00UL, 0x7a62565c6e00UL, 0x7a622c424e00UL, 0x7a643e266e00UL,
           0x7a6444384e00UL, 0x7a066e146800UL, 0x7a06140a4800UL, 0x7a00066e6800UL, 0x7a007c704800UL, 0x7a18487c6800UL,
           0x7a1832624800UL, 0x7a1e20066800UL, 0x7a1e5a184800UL, 0x7c14702a5000UL, 0x7c140a347000UL, 0x7c1218505000UL,
           0x7c12624e7000UL, 0x7c0a56425000UL, 0x7c0a2c5c7000UL, 0x7c0c3e385000UL, 0x7c0c44267000UL, 0x7c6e6e0a5600UL,
           0x7c6e14147600UL, 0x7c6806705600UL, 0x7c687c6e7600UL, 0x7c7048625600UL, 0x7c70327c7600UL, 0x7c7620185600UL,
           0x7c765a067600UL, 0x18565c687a6200UL, 0x18565c12644200UL, 0x18565a00006200UL, 0x18565a7a1e4200UL,
           0x1856424e126200UL, 0x185642340c4200UL, 0x18564426686200UL, 0x1856445c764200UL, 0x185626765a6400UL,
           0x1856260c444400UL, 0x1856201e206400UL, 0x185620643e4400UL, 0x18563850326400UL, 0x1856382a2c4400UL,
           0x18563e38486400UL, 0x18563e42564400UL, 0x18503468645c00UL, 0x185034127a7c00UL, 0x185032001e5c00UL,
           0x1850327a007c00UL, 0x18502a4e0c5c00UL, 0x18502a34127c00UL, 0x18502c26765c00UL, 0x18502c5c687c00UL,
           0x18504e76445a00UL, 0x18504e0c5a7a00UL, 0x1850481e3e5a00UL, 0x18504864207a00UL, 0x185050502c5a00UL,
           0x1850502a327a00UL, 0x18505638565a00UL, 0x18505642487a00UL, 0x18487c68001800UL, 0x18487c121e3800UL,
           0x18487a007a1800UL, 0x18487a7a643800UL, 0x1848624e681800UL, 0x18486234763800UL, 0x18486426121800UL,
           0x1848645c0c3800UL, 0x18480676201e00UL, 0x1848060c3e3e00UL, 0x1848001e5a1e00UL, 0x18480064443e00UL,
           0x18481850481e00UL, 0x1848182a563e00UL, 0x18481e38321e00UL, 0x18481e422c3e00UL, 0x184e14681e2600UL,
           0x184e1412000600UL, 0x184e1200642600UL, 0x184e127a7a0600UL, 0x184e0a4e762600UL, 0x184e0a34680600UL,
           0x184e0c260c2600UL, 0x184e0c5c120600UL, 0x184e6e763e2000UL, 0x184e6e0c200000UL, 0x184e681e442000UL,
           0x184e68645a0000UL, 0x184e7050562000UL, 0x184e702a480000UL, 0x184e76382c2000UL, 0x184e7642320000UL,
           0x182c5c6e140a00UL, 0x182c5c140a2a00UL, 0x182c5a066e0a00UL, 0x182c5a7c702a00UL, 0x182c42487c0a00UL,
           0x182c4232622a00UL, 0x182c4420060a00UL, 0x182c445a182a00UL, 0x182c2670340c00UL, 0x182c260a2a2c00UL,
           0x182c20184e0c00UL, 0x182c2062502c00UL, 0x182c38565c0c00UL, 0x182c382c422c00UL, 0x182c3e3e260c00UL,
           0x182c3e44382c00UL, 0x182a346e0a3400UL, 0x182a3414141400UL, 0x182a3206703400UL, 0x182a327c6e1400UL,
           0x182a2a48623400UL, 0x182a2a327c1400UL, 0x182a2c20183400UL, 0x182a2c5a061400UL, 0x182a4e702a3200UL,
           0x182a4e0a341200UL, 0x182a4818503200UL, 0x182a48624e1200UL, 0x182a5056423200UL, 0x182a502c5c1200UL,
           0x182a563e383200UL, 0x182a5644261200UL, 0x18327c6e6e7000UL, 0x18327c14705000UL, 0x18327a06147000UL,
           0x18327a7c0a5000UL, 0x18326248067000UL, 0x18326232185000UL, 0x183264207c7000UL, 0x1832645a625000UL,
           0x183206704e7600UL, 0x1832060a505600UL, 0x18320018347600UL, 0x183200622a5600UL, 0x18321856267600UL,
           0x1832182c385600UL, 0x18321e3e5c7600UL, 0x18321e44425600UL, 0x1834146e704e00UL, 0x183414146e6e00UL,
           0x183412060a4e00UL, 0x1834127c146e00UL, 0x18340a48184e00UL, 0x18340a32066e00UL, 0x18340c20624e00UL,
           0x18340c5a7c6e00UL, 0x18346e70504800UL, 0x18346e0a4e6800UL, 0x183468182a4800UL, 0x18346862346800UL,
           0x18347056384800UL, 0x1834702c266800UL, 0x1834763e424800UL, 0x183476445c6800UL, 0x1e3e5c76444400UL,
           0x1e3e5c0c5a6400UL, 0x1e3e5a1e3e4400UL, 0x1e3e5a64206400UL, 0x1e3e42502c4400UL, 0x1e3e422a326400UL,
           0x1e3e4438564400UL, 0x1e3e4442486400UL, 0x1e3e2668644200UL, 0x1e3e26127a6200UL, 0x1e3e20001e4200UL,
           0x1e3e207a006200UL, 0x1e3e384e0c4200UL, 0x1e3e3834126200UL, 0x1e3e3e26764200UL, 0x1e3e3e5c686200UL,
           0x1e3834765a7a00UL, 0x1e38340c445a00UL, 0x1e38321e207a00UL, 0x1e3832643e5a00UL, 0x1e382a50327a00UL,
           0x1e382a2a2c5a00UL, 0x1e382c38487a00UL, 0x1e382c42565a00UL, 0x1e384e687a7c00UL, 0x1e384e12645c00UL,
           0x1e384800007c00UL, 0x1e38487a1e5c00UL, 0x1e38504e127c00UL, 0x1e3850340c5c00UL, 0x1e385626687c00UL,
           0x1e38565c765c00UL, 0x1e207c763e3e00UL, 0x1e207c0c201e00UL, 0x1e207a1e443e00UL, 0x1e207a645a1e00UL,
           0x1e206250563e00UL, 0x1e20622a481e00UL, 0x1e2064382c3e00UL, 0x1e206442321e00UL, 0x1e2006681e3800UL,
           0x1e200612001800UL, 0x1e200000643800UL, 0x1e20007a7a1800UL, 0x1e20184e763800UL, 0x1e201834681800UL,
           0x1e201e260c3800UL, 0x1e201e5c121800UL, 0x1e261476200000UL, 0x1e26140c3e2000UL, 0x1e26121e5a0000UL,
           0x1e261264442000UL, 0x1e260a50480000UL, 0x1e260a2a562000UL, 0x1e260c38320000UL, 0x1e260c422c2000UL,
           0x1e266e68000600UL, 0x1e266e121e2600UL, 0x1e2668007a0600UL, 0x1e26687a642600UL, 0x1e26704e680600UL,
           0x1e267034762600UL, 0x1e267626120600UL, 0x1e26765c0c2600UL, 0x1e445c702a2c00UL, 0x1e445c0a340c00UL,
           0x1e445a18502c00UL, 0x1e445a624e0c00UL, 0x1e444256422c00UL, 0x1e44422c5c0c00UL, 0x1e44443e382c00UL,
           0x1e444444260c00UL, 0x1e44266e0a2a00UL, 0x1e442614140a00UL, 0x1e442006702a00UL, 0x1e44207c6e0a00UL,
           0x1e443848622a00UL, 0x1e4438327c0a00UL, 0x1e443e20182a00UL, 0x1e443e5a060a00UL, 0x1e423470341200UL,
           0x1e42340a2a3200UL, 0x1e4232184e1200UL, 0x1e423262503200UL, 0x1e422a565c1200UL, 0x1e422a2c423200UL,
           0x1e422c3e261200UL, 0x1e422c44383200UL, 0x1e424e6e141400UL, 0x1e424e140a3400UL, 0x1e4248066e1400UL,
           0x1e42487c703400UL, 0x1e4250487c1400UL, 0x1e425032623400UL, 0x1e425620061400UL, 0x1e42565a183400UL,
           0x1e5a7c70505600UL, 0x1e5a7c0a4e7600UL, 0x1e5a7a182a5600UL, 0x1e5a7a62347600UL, 0x1e5a6256385600UL,
           0x1e5a622c267600UL, 0x1e5a643e425600UL, 0x1e5a64445c7600UL, 0x1e5a066e705000UL, 0x1e5a06146e7000UL,
           0x1e5a00060a5000UL, 0x1e5a007c147000UL, 0x1e5a1848185000UL, 0x1e5a1832067000UL, 0x1e5a1e20625000UL,
           0x1e5a1e5a7c7000UL, 0x1e5c14704e6800UL, 0x1e5c140a504800UL, 0x1e5c1218346800UL, 0x1e5c12622a4800UL,
           0x1e5c0a56266800UL, 0x1e5c0a2c384800UL, 0x1e5c0c3e5c6800UL, 0x1e5c0c44424800UL, 0x1e5c6e6e6e6e00UL,
           0x1e5c6e14704e00UL, 0x1e5c6806146e00UL, 0x1e5c687c0a4e00UL, 0x1e5c7048066e00UL, 0x1e5c7032184e00UL,
           0x1e5c76207c6e00UL, 0x1e5c765a624e00UL};
      fromVector(codes, d._code_id);
      d._nbits = 64;
      d._tau = 5;
      d._type = CHILITAGS;
      d._name = "CHILITAGS";
    }
      break;
    case CUSTOM:
      throw cv::Exception(-1, "CUSTOM type is only set by loading from file", "Dictionary::loadPredefined",
                          "dictionary.h", -1);
      break;
    case ALL_DICTS:
    {
      d._nbits = 64;
      d._tau = 1; //
      d._type = ALL_DICTS;
      d._name = "ALL_DICTS";
    }
      break;

    default:
      throw cv::Exception(9001, "Invalid Dictionary type requested", "Dictionary::loadPredefined", __FILE__, __LINE__);
  };

  return d;
}

/**
 * @brief Dictionary::getMarkerImage_id
 * @param id
 * @return
 */
cv::Mat Dictionary::getMarkerImage_id(int id, int bit_size, bool addWaterMark, bool enclosed_corners,
                                      bool externalWhiteBorder)
{
  const int nBitsSquared = static_cast<int>(std::sqrt(nbits()));
  const int A = bit_size * (2 + nBitsSquared);

  cv::Mat img = cv::Mat::zeros(A, A, CV_8UC1);

  //find the code in the map
  std::uint64_t code;
  bool found = false;
  for (auto c_id : _code_id)
    if (c_id.second == id)
    {
      found = true;
      code = c_id.first;
      break;
    }
  if (!found)
  {
    std::cerr << "marker " << id << " not found" << std::endl;
    return cv::Mat();
  }
  // get the code as a bitset
  std::bitset<64> bset = code;

  int bidx = 0;
  for (int y = nBitsSquared - 1; y >= 0; y--)
    for (int x = nBitsSquared - 1; x >= 0; x--)
    {
      if (bset[bidx++])
      {
        cv::Mat bit_pix = img(cv::Range((1 + y) * bit_size, (2 + y) * bit_size),
                              cv::Range((1 + x) * bit_size, (2 + x) * bit_size));
        bit_pix.setTo(cv::Scalar::all(255));
      }
    }

  if (addWaterMark)
  {
    char idcad[30];
    sprintf(idcad, "#%d", id);
    float ax = static_cast<float>(A) / 100.f;
    int linew = 1 + (img.rows / 500);
    cv::putText(img, idcad, cv::Point(0, img.rows - img.rows / 40), cv::FONT_HERSHEY_COMPLEX, ax * 0.15f,
                cv::Scalar::all(30), linew, cv::LINE_AA);
  }

  if (enclosed_corners)
  {
    cv::Mat biggerImage(img.rows + img.rows / 2, img.cols + img.cols / 2, img.type());
    biggerImage.setTo(cv::Scalar::all(255));

    // set the image in the center
    int sy = img.rows / 4;
    int sx = img.cols / 4;
    cv::Mat center = biggerImage(cv::Range(sy, sy + img.rows), cv::Range(sx, sx + img.cols));
    img.copyTo(center);
    biggerImage(cv::Range(0, sy), cv::Range(0, sx)).setTo(cv::Scalar::all(0));
    biggerImage(cv::Range(biggerImage.rows - sy, biggerImage.rows), cv::Range(0, sx)).setTo(cv::Scalar::all(0));
    biggerImage(cv::Range(biggerImage.rows - sy, biggerImage.rows), cv::Range(biggerImage.cols - sx, biggerImage.cols)).setTo(
        cv::Scalar::all(0));
    biggerImage(cv::Range(0, sy), cv::Range(biggerImage.cols - sx, biggerImage.cols)).setTo(cv::Scalar::all(0));

    img = biggerImage;
  }

  if (externalWhiteBorder && !enclosed_corners)
  {
    int borderSize = bit_size;

    // create another image in white and put the marker in the middle
    cv::Mat biggerImage(img.rows + 2 * borderSize, img.cols + 2 * borderSize, img.type());
    biggerImage.setTo(cv::Scalar::all(255));

    // set the image in the center
    cv::Mat center = biggerImage(cv::Range(borderSize, borderSize + img.rows),
                                 cv::Range(borderSize, borderSize + img.cols));
    img.copyTo(center);
    img = biggerImage;
  }
  return img;
}

std::string Dictionary::getTypeString(DICT_TYPES t)
{
  switch (t)
  {
    case ARUCO:
      return "ARUCO";
    case ARUCO_MIP_16h3:
      return "ARUCO_MIP_16h3";
    case ARUCO_MIP_25h7:
      return "ARUCO_MIP_25h7";
    case ARTAG:
      return "ARTAG";

    case ARUCO_MIP_36h12:
      return "ARUCO_MIP_36h12";
    case ARTOOLKITPLUS:
      return "ARTOOLKITPLUS";
    case ARTOOLKITPLUSBCH:
      return "ARTOOLKITPLUSBCH";
    case TAG16h5:
      return "TAG16h5";
    case TAG25h7:
      return "TAG25h7";
    case TAG25h9:
      return "TAG25h9";
    case TAG36h11:
      return "TAG36h11";
    case TAG36h10:
      return "TAG36h10";
    case CHILITAGS:
      return "CHILITAGS";
    case CUSTOM:
      return "CUSTOM";
    case ALL_DICTS:
      return "ALL_DICTS";
  };
  return "Non valid DICT_TYPE";
}

Dictionary::DICT_TYPES Dictionary::getTypeFromString(std::string str)
{
  if (str == "ARUCO")
    return ARUCO;
  if (str == "ARUCO_MIP_16h3")
    return ARUCO_MIP_16h3;
  if (str == "ARUCO_MIP_25h7")
    return ARUCO_MIP_25h7;
  if (str == "ARUCO_MIP_36h12")
    return ARUCO_MIP_36h12;
  if (str == "ARTOOLKITPLUS")
    return ARTOOLKITPLUS;
  if (str == "ARTOOLKITPLUSBCH")
    return ARTOOLKITPLUSBCH;
  if (str == "ARTAG")
    return ARTAG;
  if (str == "TAG16h5")
    return TAG16h5;
  if (str == "TAG25h7")
    return TAG25h7;
  if (str == "TAG25h9")
    return TAG25h9;
  if (str == "TAG36h11")
    return TAG36h11;
  if (str == "TAG36h10")
    return TAG36h10;
  if (str == "CHILITAGS")
    return CHILITAGS;
  if (str == "CUSTOM")
    return CUSTOM;
  if (str == "ALL_DICTS")
    return ALL_DICTS;

  throw cv::Exception(9001, "Invalid string <" + str + "> to convert to Dictionary type",
                      "Dictionary::getTypeFromString", __FILE__, __LINE__);
}

bool Dictionary::isPredefinedDictinaryString(std::string str)
{
  try
  {
    getTypeFromString(str);
    return true;
  }
  catch (std::exception &)
  {
    return false;
  }
}

std::vector<std::string> Dictionary::getDicTypes()
{
  return
  { "ARUCO","ARUCO_MIP_16h3","ARUCO_MIP_25h7","ARUCO_MIP_36h12", "ARTOOLKITPLUS","ARTOOLKITPLUSBCH","TAG16h5","TAG25h7",
    "TAG25h9","TAG36h11","TAG36h10","CHILITAGS","ALL_DICTS"};
}

MarkerMap Dictionary::createMarkerMap(cv::Size gridSize, int MarkerSize, int MarkerDistance,
                                      const std::vector<int> &ids, bool chess_board)
{
  if (gridSize.height * gridSize.width != int(ids.size()))
    throw cv::Exception(9001, "gridSize != ids.size()Invalid ", "Dictionary::createMarkerMap", __FILE__, __LINE__);
  MarkerMap TInfo;

  TInfo.mInfoType = MarkerMap::PIX;
  TInfo.setDictionary(getTypeString(_type));
  const float markerSizeFloat = static_cast<float>(MarkerSize);

  if (!chess_board)
  {
    TInfo.resize(ids.size());
    for (std::size_t i = 0; i < ids.size(); i++)
      TInfo[i].id = ids[i];
    int idp = 0;
    float mtotal = static_cast<float>(MarkerDistance) + markerSizeFloat;
    for (int y = 0; y < gridSize.height; y++)
      for (int x = 0; x < gridSize.width; x++, idp++)
      {
        /// \todo use const auto &
        for (auto p : aruco::Marker::get3DPoints(markerSizeFloat))
          TInfo[idp].points.push_back(p + cv::Point3f(x * mtotal, y * mtotal, 0));
      }
  }
  else
  {
    // find the center so that the ref system is in it
    int CurMarkerIdx = 0;
    for (int y = 0; y < gridSize.height; y++)
    {

      bool toWrite;
      if (y % 2 == 0)
        toWrite = false;
      else
        toWrite = true;
      for (int x = 0; x < gridSize.width; x++)
      {
        toWrite = !toWrite;
        if (toWrite)
        {
          if (CurMarkerIdx >= int(ids.size()))
            throw cv::Exception(999, " FiducidalMarkers::createMarkerMapImage_ChessMarkerMap",
                                "INTERNAL ERROR. REWRITE THIS!!", __FILE__, __LINE__);
          TInfo.push_back(Marker3DInfo(ids[CurMarkerIdx++]));

          /// \todo use const auto &
          for (auto p : aruco::Marker::get3DPoints(markerSizeFloat))
            TInfo.back().points.push_back(p + cv::Point3f(x * markerSizeFloat, y * markerSizeFloat, 0));
        }
      }
    }
  }

//  cv::Point3f center(0, 0, 0);
//  double n = 0;
//  for (auto &ti : TInfo)
//    for (auto p : ti)
//    {
//      center += p;
//      n++;
//    }
//  center *= 1./n;
  cv::Point3f center(gridSize.width / 2.0f * markerSizeFloat - markerSizeFloat / 2.0f,
                     gridSize.height / 2.0f * markerSizeFloat - markerSizeFloat / 2.0f, 0.f);
  for (auto &ti : TInfo)
    for (auto &p : ti.points)
      p -= center;

  return TInfo;
}

/**
 * @brief Dictionary::computeDictionaryDistance
 * @param dict
 * @return
 */
std::uint64_t Dictionary::computeDictionaryDistance(const Dictionary &dict)
{
  auto rotate = [](const cv::Mat &in)
  {
    cv::Mat out;
    in.copyTo(out);
    for (int i = 0; i < in.rows; i++)
    {
      for (int j = 0; j < in.cols; j++)
      {
        out.at< uchar >(i, j) = in.at< uchar >(in.cols - j - 1, i);
      }
    }
    return out;
  };

  auto getImage = [](std::uint64_t tag, int nbits)
  {
    std::bitset<64> bs(tag);
    cv::Mat im(nbits, nbits, CV_8UC1);
    int bit = (nbits * nbits) - 1;
    for (int i = 0; i < nbits; i++)
    for (int j = 0; j < nbits; j++)
    im.at<uchar>(i, j) = bs[bit--];
    return im;
  };

  auto getCode = [](const cv::Mat &in)
  {
    assert(in.type()==CV_8UC1);
    std::bitset<64> bs;
    std::size_t bit = in.total() - 1;
    for (int i = 0; i < in.rows; i++)
    for (int j = 0; j < in.cols; j++)
    bs[bit--] = in.at < uchar > (i, j);
    return bs.to_ullong();
  };

  // convert each element into its 4 rotations first
  std::vector<std::uint64_t> all_rotations;

  std::vector<std::uint64_t> ids;

  // for chilitag
  auto code_id2 = dict._code_id;
  for (auto tag_id : code_id2)
  {
    all_rotations.push_back(tag_id.first);
    auto m = getImage(tag_id.first, static_cast<int>(sqrt(dict._nbits)));
    for (int i = 0; i < 3; i++)
    {
      m = rotate(m);
      all_rotations.push_back(getCode(m));
      ids.push_back(tag_id.second);
    }
    assert(getCode(rotate(m)) == tag_id.first);
  }

  // now, compute minimum distance
  std::map<std::uint64_t, std::set<std::uint64_t> > errors;
  std::uint64_t mind = std::numeric_limits<std::uint64_t>::max();
  for (std::size_t i = 0; i < all_rotations.size(); i++)
  {
    for (std::size_t j = 0; j < all_rotations.size(); j++)
    {
      if (i != j)
      {

        auto d = std::bitset<64>(all_rotations[i] ^ all_rotations[j]).count();
        if (d == 0)
        {
          if (errors[ids[i]].count(ids[j]) == 0)
          {
            std::cerr << "  Dictionary::computeDictionaryDistance ERROR IN YOUR DICT!!!!!" << std::endl;
            if (ids[i] == ids[j])
            {
              std::cerr << "marker " << ids[i]
                  << " can be confused with one of its rotations. It is impossible to determine properly its "
                  << " rotation. You should remove this marker from the dictionary" << std::endl;
            }
            else
            {
              std::cerr << "marker " << ids[i] << " and   " << ids[j]
                  << " can be confused. It is impossible to determine distinguish them . You should remove any of "
                  << " this marker from the dictionary" << std::endl;
            }
            errors[ids[i]].insert(ids[j]);
            errors[ids[j]].insert(ids[i]);
          }
        }
        mind = std::min(mind, std::uint64_t(d));
      }
    }
  }
  return mind;
}

} // namespace aruco
