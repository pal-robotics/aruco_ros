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

#include "svmmarkers.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <string>
#include <fstream>
#include <opencv2/core.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/ml.hpp>
#include <iostream>

using namespace std;

namespace aruco {
namespace impl{
class SVMMarkers  {
    float _minFeatureValue, _maxFeatureValue; // image range for svm classification
    int _patchSize, _dictSize; // marker pathSize (total features = pathSize^2) and dictionary size (number of markers)
    bool _rotateMarkers; // interchange rotation 1 and 3 ??
#if  CV_MAJOR_VERSION >= 3
    cv::Ptr<cv::ml::SVM>  _model;
#else
    cv::Ptr<CvSVM>  _model  ;
#endif
public:
    SVMMarkers(){
        // static variables from SVMMarkers. Need to be here to avoid linking errors
        _minFeatureValue = -1;
        _maxFeatureValue = 1;
        _patchSize = 10;
        _dictSize = -1;
        _rotateMarkers = true;

    }


    int getBestInputSize(){return _patchSize;}
    bool load(string path) {
    if (path.empty())return false;


#if  CV_MAJOR_VERSION >= 3
    _model= cv::ml::StatModel::load<cv::ml::SVM>(path);
     _patchSize=sqrt(_model->getSupportVectors().size().width);

#else
    _model=new CvSVM;
    _model->load(path.c_str());
    _patchSize=sqrt(_model->get_var_count());

#endif
        //read in the file the number of classes
        ifstream svmfile(path.c_str());
        if (!svmfile)return false;
        _dictSize=-1;
        while(!svmfile.eof()){
            std::string str;
            std::getline(svmfile,str);
            if (str.find("class_count:")!=std::string::npos){
                stringstream sstr;sstr<<str;
                string cc;
                int  nclasses;
                sstr>>cc>>nclasses;
                _dictSize=(nclasses-1)/4;
            }
        }
        if (_dictSize==-1)return false;

        return true;
    }






    /**
 */
    bool  detect(const cv::Mat &in, int &mid,int &nRotations) {


        // convert to gray
        assert(in.rows == in.cols);
        cv::Mat grey;
        if (in.type() == CV_8UC1)
            grey = in;
        else
            cv::cvtColor(in, grey, CV_BGR2GRAY);

        // resize to svm path size
        cv::Mat greyResized;
        if(grey.cols != _patchSize)
            cv::resize(grey, greyResized, cv::Size(_patchSize, _patchSize) );
        else
            greyResized = grey;

        // normalize image range
        cv::Mat greyResizedNormalized;
        cv::normalize(greyResized, greyResizedNormalized, _minFeatureValue, _maxFeatureValue, cv::NORM_MINMAX, CV_32FC1);

        // rearrange data in a row
        cv::Mat dataRow;
        dataRow = greyResizedNormalized.reshape(1,1);

        // predict id with svm
#if  CV_VERSION_MAJOR >= 3
        int predict_id = (int)_model->predict(dataRow);
#else
        int predict_id = (int)_model->predict(dataRow, true);
#endif

        // get rotation of marker
        nRotations = predict_id%4;
        // if _rotateMarkers, interchange rotation 1 and 3
        if(_rotateMarkers)
            if(nRotations==1 || nRotations==3) nRotations = nRotations==1?3:1;

        // if dictSize is known (dictSize!=-1), and the marker id is equal to the number of markers * 4
        // then it is the invalid marker class
        if(_dictSize!=-1 && predict_id==_dictSize*4) return false;
        // else, divide by 4 to obtain the final marker id
        mid= predict_id/4;
        return true;

    }

};

}
SVMMarkers::SVMMarkers(){
    _impl=new impl::SVMMarkers;
}

bool SVMMarkers::load(string path) {
    return _impl->load(path);
}

/**
 * Detect marker in a canonical image.
 * Return marker id in 0 rotation, or -1 if not found
 * Assign the detected rotation of the marker to nRotation
 */
bool SVMMarkers::detect(const cv::Mat &in, int &marker_id, int &nRotations, string &additionalInfo) {
    bool res=_impl->detect(in,marker_id,nRotations);
    if (res) additionalInfo="SVM";
    return res;
}
int SVMMarkers::getBestInputSize(){return _impl->getBestInputSize();}

}
