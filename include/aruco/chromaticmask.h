/*****************************
Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

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
********************************/

#ifndef CHROMATICMASK_H
#define CHROMATICMASK_H

#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "exports.h"
#include "board.h"
#include "boarddetector.h"
#include "cvdrawingutils.h"


class ARUCO_EXPORTS EMClassifier {
public:
  EMClassifier(unsigned int nelements=200);
  void addSample(uchar s) { _samples.push_back(s); };
  void clearSamples() { _samples.clear();  } ;
  void train();
  bool classify(uchar s) { return _inside[s]; };
  double getProb(uchar s) { return _prob[s]; };
  unsigned int numsamples() {return _samples.size();};
  void setProb(double p) { _threshProb = p; }
  
//   double probConj[256];
  
private:
  cv::EM _classifier;
  vector<uchar> _samples;
  bool _inside[256];
  double _prob[256];
  double _histogram[256];
  unsigned int _nelem;
  double _threshProb;

};


class ARUCO_EXPORTS ChromaticMask
{
public:
  
  ChromaticMask() : _cellSize(20) { _isValid=false; };
  
  void setParams(unsigned int mc, unsigned int nc, double threshProb, aruco::CameraParameters CP, aruco::BoardConfiguration BC, vector<cv::Point3f> corners);
  void setParams(unsigned int mc, unsigned int nc, double threshProb, aruco::CameraParameters CP, aruco::BoardConfiguration BC, float markersize=-1.);
  
  void calculateGridImage(const aruco::Board &board);
  
  cv::Mat getCellMap() { return _cellMap; };
  cv::Mat getMask() { return _mask; };
  
  void train(const cv::Mat& in, const aruco::Board &board);
  void classify(const cv::Mat& in, const aruco::Board &board);
  void classify2(const cv::Mat& in, const aruco::Board &board);
  void update(const cv::Mat& in);
  
  bool isValid() {return _isValid;};
  void resetMask();
  
private:
  
  double getDistance(cv::Point2d pixel, unsigned int classifier) {
    cv::Vec2b canPos = _canonicalPos.at<cv::Vec2b>(pixel.y, pixel.x)[0];
    return norm(_cellCenters[classifier] - cv::Point2f(canPos[0], canPos[1]) );
  }
  
  vector<cv::Point2f> _imgCornerPoints;  
  vector<cv::Point3f> _objCornerPoints;
  cv::Mat _perpTrans;
  vector<EMClassifier> _classifiers;
  vector<cv::Point2f> _centers;
  vector<cv::Point2f> _pixelsVector;
  vector<cv::Point2f> _cellCenters;
  vector<vector<size_t> > _cell_neighbours;
  const float _cellSize;

  
  unsigned int _mc, _nc;
  aruco::BoardDetector _BD;
  aruco::CameraParameters _CP;
  cv::Mat _canonicalPos, _cellMap, _mask,_maskAux;
  bool _isValid;
  double _threshProb;


  
};

#endif // CHROMATICMASK_H
