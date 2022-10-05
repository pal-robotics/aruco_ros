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

#ifndef ARUCO_TIMERS_H
#define ARUCO_TIMERS_H


#include <chrono>
#include <string>
#include <vector>
#include <iostream>
#include "aruco_export.h"
namespace aruco
{

// timer
struct ScopeTimer
{
  std::chrono::high_resolution_clock::time_point begin, end;

  std::string name;
  bool use;
  enum SCALE
  {
    NSEC,
    MSEC,
    SEC
  };
  SCALE sc;
  ScopeTimer(std::string name_, bool use_ = true, SCALE _sc = MSEC)
  {
#ifdef USE_TIMERS
    name = name_;
    use = use_;
    sc = _sc;
    begin = std::chrono::high_resolution_clock::now();
#else
    (void)name_;
    (void)use_;
    (void)_sc;

#endif
  }
  ~ScopeTimer()
  {
#ifdef USE_TIMERS
    if (use)
    {
      end = std::chrono::high_resolution_clock::now();
      double fact = 1;
      std::string str;
      switch (sc)
      {
        case NSEC:
          fact = 1;
          str = "ns";
          break;
        case MSEC:
          fact = 1e6;
          str = "ms";
          break;
        case SEC:
          fact = 1e9;
          str = "s";
          break;
      };

      std::cout << "Time(" << name << ")= "
                << double(
                       std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count()) /
                       fact
                << str << std::endl;
      ;
    }
#endif
  }
};

struct ScopedTimerEvents
{
  enum SCALE
  {
    NSEC,
    MSEC,
    SEC
  };
  SCALE sc;
  std::vector<std::chrono::high_resolution_clock::time_point> vtimes;
  std::vector<std::string> names;
  std::string _name;

  ScopedTimerEvents(std::string name = "", bool start = true, SCALE _sc = MSEC)
  {
#ifdef USE_TIMERS
    if (start)
      add("start");
    sc = _sc;
    _name = name;
#else
    (void)name;
    (void)start;
    (void)_sc;
#endif
  }

  void add(std::string name)
  {
#ifdef USE_TIMERS
    vtimes.push_back(std::chrono::high_resolution_clock::now());
    names.push_back(name);
#else
    (void)name;
#endif
  }
  void addspaces(std::vector<std::string> &str)
  {
    // get max size
    size_t m = 0;
    for (auto &s : str)
      m = std::max(size_t(s.size()), m);
    for (auto &s : str)
    {
      while (s.size() < m)
        s.push_back(' ');
    }
  }

  ~ScopedTimerEvents()
  {
#ifdef USE_TIMERS
    double fact = 1;
    std::string str;
    switch (sc)
    {
      case NSEC:
        fact = 1;
        str = "ns";
        break;
      case MSEC:
        fact = 1e6;
        str = "ms";
        break;
      case SEC:
        fact = 1e9;
        str = "s";
        break;
    };

    add("total");
    addspaces(names);
    for (size_t i = 1; i < vtimes.size(); i++)
    {
      std::cout << "Time(" << _name << "|" << names[i] << " ):"
                << double(std::chrono::duration_cast<std::chrono::nanoseconds>(
                              vtimes[i] - vtimes[i - 1])
                              .count()) /
                       fact
                << str << " "
                << double(std::chrono::duration_cast<std::chrono::nanoseconds>(vtimes[i] -
                                                                               vtimes[0])
                              .count()) /
                       fact
                << str /*<<"\t"<< vtimes[i].time_since_epoch().count()*/ << std::endl;
    }
#endif
  }
};

struct ARUCO_EXPORT Timer
{
  enum SCALE
  {
    NSEC,
    MSEC,
    SEC
  };

  std::chrono::high_resolution_clock::time_point _s;
  double sum = 0, n = 0;
  std::string _name;
  Timer()
  {
  }

  Timer(std::string name) : _name(name)
  {
  }
  void setName(std::string name)
  {
    _name = name;
  }
  void start()
  {
    _s = std::chrono::high_resolution_clock::now();
  }
  void end()
  {
    auto e = std::chrono::high_resolution_clock::now();
    sum += double(std::chrono::duration_cast<std::chrono::nanoseconds>(e - _s).count());
    n++;
  }

  void print(SCALE sc = MSEC)
  {
#ifdef USE_TIMERS
    double fact = 1;
    std::string str;
    switch (sc)
    {
      case NSEC:
        fact = 1;
        str = "ns";
        break;
      case MSEC:
        fact = 1e6;
        str = "ms";
        break;
      case SEC:
        fact = 1e9;
        str = "s";
        break;
    };
    std::cout << "Time(" << _name << ")= " << (sum / n) / fact << str << std::endl;
#else
    (void)sc;

#endif
  }

  double getAverage(SCALE sc = MSEC) const
  {
    double fact = 1;
    switch (sc)
    {
      case NSEC:
        fact = 1;
        break;
      case MSEC:
        fact = 1e6;
        break;
      case SEC:
        fact = 1e9;
        break;
    };
    return (sum / n) / fact;
  }
};
inline std::string __pf_aruco_methodName(std::string prettyFunction)
{
  std::string res;
  res.reserve(prettyFunction.size());
  bool spaceFound = false;
  for (auto c : prettyFunction)
  {
    if (c == ' ' && !spaceFound)
      spaceFound = true;
    else if (c != '(' && spaceFound)
      res.push_back(c);
    else if (c == '(' && spaceFound)
      break;
  }
  return res;
}
#ifdef USE_TIMERS

#define __ARUCO_ADDTIMER__                                                               \
  ScopedTimerEvents XTIMER_X(__pf_aruco_methodName(__PRETTY_FUNCTION__));
#define __ARUCO_TIMER_EVENT__(Y) XTIMER_X.add(Y);
#else
#define __ARUCO_ADDTIMER__
#define __ARUCO_TIMER_EVENT__(Y)
#endif
}  // namespace aruco


#endif
