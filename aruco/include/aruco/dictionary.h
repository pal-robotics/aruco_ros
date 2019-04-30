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

#ifndef ARUCO_DICTIONARY_
#define ARUCO_DICTIONARY_

#include "aruco_export.h"

#include <opencv2/core.hpp>

#include <iostream>
#include <map>
#include <stdint.h>
#include <string>
#include <vector>

namespace aruco
{

class MarkerMap;

/**
 * Represents a set of valid marker ids with a maximum size of 8x8 = 64 bits.
 * In our approach, markers are seen as a pair code-id. The code is the internal binary code printed on the marker.
 * Maximum size is 8x8 bits. The id is a smaller number you can use to identify it. You will use only the id
 *
 * See enum DICT_TYPES for the set of available dictionaries
 */
class ARUCO_EXPORT Dictionary
{
public:

  // loads from a set of predefined ones
  enum DICT_TYPES
    : uint64_t
    {
      ARUCO_MIP_36h12 = 0x8, //*** recommended
    ARUCO = 0x1, // original aruco dictionary. By default
    ARUCO_MIP_25h7 = 0x2, ARUCO_MIP_16h3 = 0x4, ARTAG = 0x10, //
    ARTOOLKITPLUS = 0x20, ARTOOLKITPLUSBCH = 0x40, //
    TAG16h5 = 0x80, TAG25h7 = 0x100, TAG25h9 = 0x200, TAG36h11 = 0x400, TAG36h10 = 0x800, // april tags
    CHILITAGS = 0x1000, // chili tags dictionary . NOT RECOMMENDED. It has distance 0. Markers 806 and 682 should not be
    // used!!!
    CUSTOM = 0x4000, // for used defined dictionaries  (using loadFromfile).
    ALL_DICTS = 0xFFFF
  };

  // indicates if a code is in the dictionary
  bool is(uint64_t code) const
  {
    return _code_id.find(code) != _code_id.end();
  }

  DICT_TYPES getType() const
  {
    return _type;
  }

  // return the number of ids
  uint64_t size() const
  {
    return _code_id.size();
  }

  // returns the total number of bits of the binary code
  uint32_t nbits() const
  {
    return _nbits;
  }

  // returns the dictionary distance
  uint32_t tau() const
  {
    return _tau;
  }

  // returns the name
  std::string getName() const
  {
    return _name;
  }

  // return the set of ids
  const std::map<uint64_t, uint16_t>& getMapCode() const
  {
    return _code_id;
  }

  // returns the id of a given code.
  int operator[](uint64_t code)
  {
    return _code_id[code];
  }

  // returns the id of a given code.
  int at(uint64_t code)
  {
    return _code_id[code];
  }

  /**
   * Returns the image of the marker indicated by its id. It the id is not, returns empty matrix
   * @param id of the marker image to return
   * @param bit_size of the image will be  AxA, A = ( nbits() + 2) * bit_size
   * @param enclosed_corners if true, extra rectangles are added touching the marker corners. it can be used to
   * allow subpixel refinement
   */
  cv::Mat getMarkerImage_id(int id, int bit_size, bool addWaterMark = true, bool enclosed_corners = false,
                            bool printExternalWhiteBorder = false);

  // used for boards
  MarkerMap createMarkerMap(cv::Size gridSize, int MarkerSize, int MarkerDistance, const std::vector<int>& Ids,
                            bool chess_board = false);

  static Dictionary loadPredefined(DICT_TYPES type);
  static Dictionary loadPredefined(std::string type);

  /**
   * Loads a dictionary defined in a file
   * Please note that the parsing is very basic and you must be very strict.

   * Here is an example of a 3x3 dictionary of 3 markers
   * 010    111    000
   * 001    101    001
   * 001    010    100
   *
   *
   * File:  myown.dict
   *-------------------------------------------
   * name MYOWN
   * nbits  9
   * 010001001
   * 111101010
   * 000001100
   */
  static Dictionary loadFromFile(std::string path);

  /**
   * Loads a dictionary using the string passed. If it is a string of the predefined dictionaries, then returns it.
   * Otherwise, tries to load from a file
   */
  static Dictionary load(std::string info);

//  // io functions
//  void saveToFile(std::string file);
//  void readFromFile(std::string file);
//  void saveToStream(std::ostream & str);
//  void readFromStream(std::istream &str);

  // returns the dictionary distance
  static uint64_t computeDictionaryDistance(const Dictionary& d);

  // given a string, returns the type
  static DICT_TYPES getTypeFromString(std::string str);
  static std::string getTypeString(DICT_TYPES t);
  static bool isPredefinedDictinaryString(std::string str);
  static std::vector<std::string> getDicTypes();

private:
  void insert(uint64_t code, int id)
  {
    _code_id.insert(std::make_pair(code, id));
  }

  static void fromVector(const std::vector<uint64_t>& codes, std::map<uint64_t, uint16_t>& code_id_map);

  std::map<uint64_t, uint16_t> _code_id; // marker have and code (internal binary code), which correspond to an id.

  uint32_t _nbits; // total number of bits . So, there are sqrt(nbits) in each axis
  uint32_t _tau; // minimum distance between elements

  DICT_TYPES _type;
  std::string _name;
};

} // namespace aruco

#endif /* ARUCO_DICTIONARY_ */
