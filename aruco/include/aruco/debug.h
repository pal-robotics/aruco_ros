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

#ifndef __Debug_H
#define __Debug_H
#include <iostream>
#include <fstream>
#include <ctime>
#include "aruco_export.h"
#include <map>
#include <string>
namespace aruco
{

class ARUCO_EXPORT Debug
{
private:
  static int level;  // 0(no debug), 1 medium, 2 high
  static bool isInited;

  static std::map<std::string, std::string> strings;

public:
  static void init();
  static void setLevel(int l);
  static int getLevel();



  static void addString(std::string &label, std::string &data);
  static std::string getString(std::string &str);


  static std::string getFileName(std::string filepath)
  {
    // go backwards until finding a separator or start
    size_t i;
    for (i = filepath.size() - 1; i != 0; i--)
    {
      if (filepath[i] == '\\' || filepath[i] == '/')
        break;
    }
    std::string fn;
    fn.reserve(filepath.size() - i);
    for (size_t s = i; s < filepath.size(); s++)
      fn.push_back(filepath[s]);
    return fn;
  }

#ifdef WIN32
#define __func__ __FUNCTION__
#endif
};


// message print
#if (defined DEBUG || defined _DEBUG)
#define _debug_exec(level, x)                                                            \
  {                                                                                      \
    if (Debug::getLevel() >= level)                                                      \
    {                                                                                    \
      x                                                                                  \
    }                                                                                    \
  }
#define _debug_exec_(x) x
#ifndef WIN32
#define _debug_msg(x, level)                                                             \
  {                                                                                      \
    Debug::init();                                                                       \
    if (Debug::getLevel() >= level)                                                      \
      std::cout << "#" << Debug::getFileName(__FILE__) << ":" << __LINE__ << ":"         \
                << __func__ << "#" << x << std::endl;                                    \
  }

#define _debug_msg_(x)                                                                   \
  {                                                                                      \
    Debug::init();                                                                       \
    if (Debug::getLevel() >= 5)                                                          \
      std::cout << "#" << Debug::getFileName(__FILE__) << ":" << __LINE__ << ":"         \
                << __func__ << "#" << x << std::endl;                                    \
  }

#else
#define _debug_msg(x, level)                                                             \
  {                                                                                      \
    Debug::init();                                                                       \
    if (Debug::getLevel() >= level)                                                      \
      std::cout << __func__ << ":" << Debug::getFileName(__FILE__) << ":" << __LINE__    \
                << ":  " << x << std::endl;                                              \
  }

#define _debug_msg_(x)                                                                   \
  {                                                                                      \
    Debug::init();                                                                       \
    if (Debug::getLevel() >= 5)                                                          \
      std::cout << __func__ << ":" << Debug::getFileName(__FILE__) << ":" << __LINE__    \
                << ":  " << x << std::endl;                                              \
  }

#endif

#else
#define _debug_msg(x, level)
#define _debug_msg_(x)
#define _debug_exec(level, x) ;
#define _debug_exec_(x) ;

#endif



}  // namespace aruco

#endif
