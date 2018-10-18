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

#ifndef __Debug_H
#define __Debug_H
#include <iostream>
#include <fstream>
#include <ctime>
#include "aruco_export.h"
#include <map>
#include <string>
namespace aruco{

class ARUCO_EXPORT  Debug{
private:
    static  int level;//0(no debug), 1 medium, 2 high
    static bool isInited;

    static std::map<std::string,std::string> strings;
public:
    static void init();
    static void setLevel(int l);
    static int getLevel();



    static void addString(std::string &label,std::string &data);
    static std::string getString(std::string &str);


static std::string getFileName(std::string filepath){
    //go backwards until finding a separator or start
    size_t i;
    for( i=filepath.size()-1;i!=0;i--){
        if ( filepath[i]=='\\' || filepath[i]=='/') break;
    }
    std::string fn;fn.reserve( filepath.size()-i);
    for(size_t s=i;s<filepath.size();s++)fn.push_back(filepath[s]);
    return fn;
}

#ifdef WIN32
  #define __func__ __FUNCTION__
#endif

};


//message print
#if (defined DEBUG || defined _DEBUG)
#define _debug_exec(level,x) {if (Debug::getLevel()>=level){x}}
#define _debug_exec_( x) x
  #ifndef WIN32
    #define _debug_msg(x,level) {Debug::init();\
    if (Debug::getLevel()>=level)\
              std::cout<<"#" <<Debug::getFileName(__FILE__)<<":"<<__LINE__<<":"<<__func__<<"#"<<x<<std::endl; }

    #define _debug_msg_(x) {Debug::init();\
    if (Debug::getLevel()>=5)\
          std::cout<<"#" <<Debug::getFileName(__FILE__)<<":"<<__LINE__<<":"<<__func__<<"#"<<x<<std::endl; }

  #else
     #define _debug_msg(x,level) {\
     Debug::init();\
     if (Debug::getLevel()>=level)\
       std::cout<<__func__<<":"<< Debug::getFileName(__FILE__)<<":"<<__LINE__  <<":  "<<x<<std::endl; }

#define _debug_msg_(x) {\
Debug::init();\
if (Debug::getLevel()>=5)\
  std::cout<<__func__<<":"<< Debug::getFileName(__FILE__)<<":"<<__LINE__  <<":  "<<x<<std::endl; }

#endif

#else
#define _debug_msg(x,level)
#define _debug_msg_(x)
#define _debug_exec(level,x) ;
#define _debug_exec_(x) ;

#endif



}

#endif

