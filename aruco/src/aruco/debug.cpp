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

#include "debug.h"
#include <fstream>
namespace aruco
{
int Debug::level = 2;
std::map<std::string, std::string> Debug::strings;
void Debug::addString(std::string &label, std::string &data)
{
  strings.insert(make_pair(label, data));
}

std::string Debug::getString(std::string &str)
{
  auto it = strings.find(str);
  if (it == strings.end())
    return "";
  else
    return it->second;
}


bool Debug::isInited = false;

void Debug::setLevel(int l)
{
  level = l;
  isInited = false;
  init();
}
int Debug::getLevel()
{
  init();
  return level;
}
void Debug::init()
{
  if (!isInited)
  {
    isInited = true;
    if (level >= 1)
    {
    }
  }
}


}  // namespace aruco
