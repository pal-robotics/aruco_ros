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

#ifndef ARUCO_TIMERS_H
#define ARUCO_TIMERS_H


#include <chrono>
#include <string>
#include <vector>
#include <iostream>
 namespace aruco{

//timer
struct ScopeTimer
{
    std::chrono::high_resolution_clock::time_point begin,end;

    std::string name;
    bool use;
    enum SCALE {NSEC,MSEC,SEC};
    SCALE sc;
    ScopeTimer(std::string name_,bool use_=true,SCALE _sc=MSEC)
    {
#ifdef USE_TIMERS
        name=name_;
        use=use_;
        sc=_sc;
        begin= std::chrono::high_resolution_clock::now();
#else
        (void)name_;
        (void)use_;
        (void)_sc;

#endif
    }
    ~ScopeTimer()
    {
#ifdef USE_TIMERS
        if (use){
            end= std::chrono::high_resolution_clock::now();
            double fact=1;
            std::string str;
            switch(sc)
            {
            case NSEC:fact=1;str="ns";break;
            case MSEC:fact=1e6;str="ms";break;
            case SEC:fact=1e9;str="s";break;
            };

            std::cout << "Time("<<name<<")= "<<double(std::chrono::duration_cast<std::chrono::nanoseconds>(end-begin).count())/fact<<str<<std::endl; ;
        }
#endif
    }
};

struct ScopedTimerEvents
{
    enum SCALE {NSEC,MSEC,SEC};
    SCALE sc;
    std::vector<std::chrono::high_resolution_clock::time_point> vtimes;
    std::vector<std::string> names;
    std::string _name;

    ScopedTimerEvents(std::string name="",bool start=true,SCALE _sc=MSEC){
#ifdef USE_TIMERS
        if(start) add("start");sc=_sc;_name=name;
#else
        (void)name;
        (void)start;
        (void)_sc;
#endif
    }

    void add(std::string name){
#ifdef USE_TIMERS
        vtimes.push_back(std::chrono::high_resolution_clock::now());
        names.push_back(name);
#else
        (void)name;
#endif
    }
    void addspaces(std::vector<std::string> &str ){
        //get max size
        size_t m=0;
        for(auto &s:str)m=std::max(size_t(s.size()),m);
        for(auto &s:str){
            while(s.size()<m) s.push_back(' ');
        }
    }

    ~ScopedTimerEvents(){
#ifdef USE_TIMERS
         double fact=1;
        std::string str;
        switch(sc)
        {
        case NSEC:fact=1;str="ns";break;
        case MSEC:fact=1e6;str="ms";break;
        case SEC:fact=1e9;str="s";break;
        };

        add("total");
        addspaces(names);
        for(size_t i=1;i<vtimes.size();i++){
            std::cout<<"Time("<<_name<<"|"<<names[i]<<" ):"<< double(std::chrono::duration_cast<std::chrono::nanoseconds>(vtimes[i]-vtimes[i-1]).count())/fact<<str<<" "<<double(std::chrono::duration_cast<std::chrono::nanoseconds>(vtimes[i]-vtimes[0]).count())/fact<<str/*<<"\t"<< vtimes[i].time_since_epoch().count()*/<<std::endl;
        }
#endif
    }
};

struct ARUCO_EXPORT Timer{
    enum SCALE {NSEC,MSEC,SEC};

    std::chrono::high_resolution_clock::time_point _s;
    double sum=0,n=0;
    std::string _name;
    Timer(){}

    Timer(std::string name):_name(name){}
    void setName(std::string name){_name=name;}
    void start(){_s=std::chrono::high_resolution_clock::now();}
    void end()
    {
        auto e=std::chrono::high_resolution_clock::now();
        sum+=double(std::chrono::duration_cast<std::chrono::nanoseconds>(e-_s).count());
        n++;
    }

    void print(SCALE sc=MSEC){
#ifdef USE_TIMERS
        double fact=1;
        std::string str;
        switch(sc)
        {
        case NSEC:fact=1;str="ns";break;
        case MSEC:fact=1e6;str="ms";break;
        case SEC:fact=1e9;str="s";break;
        };
        std::cout<<"Time("<<_name<<")= "<< ( sum/n)/fact<<str<<std::endl;
#else
        (void)sc;

#endif
    }

    double getAverage(SCALE sc=MSEC)const{
        double fact=1;
        switch(sc)
        {
            case NSEC:fact=1; break;
         case MSEC:fact=1e6; break;
         case SEC:fact=1e9; break;
        };
        return ( sum/n)/fact;

    }

};
}


#endif
