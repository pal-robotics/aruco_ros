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

#include <aruco/chromaticmask.h>
#include <set>
// #include <omp.h>


/**
 */
EMClassifier::EMClassifier(unsigned int nelements) : _classifier(2, cv::EM::COV_MAT_DIAGONAL, cv::TermCriteria(cv::TermCriteria::COUNT , 4, FLT_EPSILON))
{
  _nelem = nelements;
  _threshProb = 0.0001;
  for(unsigned int i=0; i<256; i++) _prob[i] = 0.5;
}



/**
 */
void EMClassifier::train()
{

  // fill histogram
    
  for(unsigned int i=0; i<256;i++) _histogram[i]=0;
  
  for(unsigned int i=0; i<_samples.size(); i++) {
    uchar val = _samples[i];
    _histogram[val]+=3;
    if(val>0) _histogram[val-1]+=2;
    if(val<255) _histogram[val+1]+=2;
    if(val>1) _histogram[val-2]+=1;
    if(val<254) _histogram[val+2]+=1;	  
  }
  
  double sum=0.;     
  for(unsigned int i=0; i<256;i++) sum += _histogram[i];
  for(unsigned int i=0; i<256;i++) _histogram[i] /= sum;
  
  
  // discretize histogram to speed up training
  unsigned int histCount[256];
  int  n=0;
  for(unsigned int i=0; i<256; i++)
    n+= (histCount[i] = (unsigned int)(_nelem*_histogram[i]));

  if(n<10) return;
  
  cv::Mat samples(n,1,CV_64FC1);
  unsigned int idx=0;
  for(unsigned int i=0; i<256; i++) {
    for(unsigned int j=0; j<histCount[i]; j++) {
      samples.ptr<double>(0)[idx] = i;
      idx++;
    }
  }  
  
  _classifier.train(samples);
  
  cv::Mat sampleAux(1,1,CV_64FC1);
  for(unsigned int i=0; i<256; i++) {
    sampleAux.ptr<double>(0)[0] = i;
    cv::Vec2d r = _classifier.predict(sampleAux);
    _prob[i] = exp(r[0]);
    if(_prob[i]>_threshProb) _inside[i]=true;
    else _inside[i]=false;
  }  
  
  
}


void ChromaticMask::setParams(unsigned int mc, unsigned int nc, double threshProb, aruco::CameraParameters CP, aruco::BoardConfiguration BC, float markersize)
{
  if(markersize==-1) {
    std::cerr << "Invalid markersize in ChromaticMask::setParams" << std::endl;
    return;
  }
  
  if(BC.size()==0) {
    std::cerr << "Invalid BoardConfiguration size in ChromaticMask::setParams" << std::endl;
    return;
  }  
   
  //calculate corners from min and max in BC
  cv::Point3f min, max;
  min = max = BC[0][0];
  for(unsigned int i=0; i<BC.size(); i++) {
    for(unsigned int j=0; j<4; j++) {
      if(BC[i][j].x <= min.x &&  BC[i][j].y <= min.y) min = BC[i][j];
      if(BC[i][j].x >= max.x &&  BC[i][j].y >= max.y) max = BC[i][j];
    }
  }
  double pixSize = fabs(markersize/(BC[0][1].x - BC[0][0].x));
  min.x *= pixSize;
  min.y *= pixSize;
  max.x *= pixSize;
  max.y *= pixSize;  
        
  // calculate border corners coordinates
  vector<cv::Point3f> corners;
  corners.push_back(min);
  corners.push_back( cv::Point3f(min.x, max.y, 0) );
  corners.push_back(max);
  corners.push_back( cv::Point3f(max.x, min.y, 0) );
    
  setParams(mc, nc, threshProb, CP, BC, corners);
  
}



/**
 */
void ChromaticMask::setParams(unsigned int mc, unsigned int nc, double threshProb, aruco::CameraParameters CP, aruco::BoardConfiguration BC, vector<cv::Point3f> corners)
{
  _classifiers.resize(mc*nc);
  for(unsigned int i=0; i<_classifiers.size(); i++) _classifiers[i].setProb(threshProb);
  _mc = mc;
  _nc = nc;
  _threshProb = threshProb;
  _cell_neighbours.resize(mc*nc);
  int idx_=0;
  //SGJ: revisar, no engo claro sepa bien que es i y j y los puedo estar confundiendo!!!
    for(unsigned int j=0; j<nc; j++)
      for(unsigned int i=0; i<mc; i++,idx_++) {
      _centers.push_back(cv::Point2f(i+0.5, j+0.5));
      for(unsigned int nj=max(j-1,(unsigned int)0); nj<min(mc,j+1); nj++) {
         for(unsigned int ni=max(i-1,(unsigned int)0); ni<min(mc,i+1); ni++) {
	   size_t tidx=nj*mc+ni;
           _cell_neighbours[idx_].push_back(tidx);           
	}      
      }
    }
  
    //deterimne the idx of the neighbours
  _BD.getMarkerDetector().setThresholdParams(35,7);
  _BD.getMarkerDetector().enableErosion(false);
  _BD.getMarkerDetector().setCornerRefinementMethod(aruco::MarkerDetector::LINES);
  _BD.setYPerpendicular(false);
  
  _BD.setParams(BC, CP);
  
  _objCornerPoints = corners;
  _CP = CP;
  
  _pixelsVector.clear();
  for(unsigned int i=0; i<CP.CamSize.height/2; i++)
    for(unsigned int j=0; j<CP.CamSize.width/2; j++)
      _pixelsVector.push_back( cv::Point2f(2*j,2*i) );
  
  resetMask();
  _cellMap = cv::Mat(CP.CamSize.height, CP.CamSize.width, CV_8UC1, cv::Scalar::all(0));
  _canonicalPos = cv::Mat(CP.CamSize.height, CP.CamSize.width, CV_8UC2);
    
  _cellCenters.resize(_classifiers.size());
  for(unsigned int i=0; i<mc; i++) {
    for(unsigned int j=0; j<nc; j++) {
      _cellCenters[i*mc+j].x = ((2*i+1)*_cellSize)/2.;
      _cellCenters[i*mc+j].y = ((2*j+1)*_cellSize)/2.;
    }
  }
  
}

pair<double,double> AvrgTime(0,0) ;

/**
 */
void ChromaticMask::calculateGridImage(const aruco::Board &board )
{
  
      // project corner points to image
      cv::projectPoints(_objCornerPoints, board.Rvec, board.Tvec, _CP.CameraMatrix, _CP.Distorsion, _imgCornerPoints);    

      //obtain the perspective transform
      cv::Point2f  pointsRes[4],pointsIn[4];
      for ( int i=0;i<4;i++ ) pointsIn[i]=_imgCornerPoints[i];
      pointsRes[0]= ( cv::Point2f ( 0,0 ) );
      pointsRes[1]= cv::Point2f ( _cellSize*_mc-1, 0 );
      pointsRes[2]= cv::Point2f ( _cellSize*_mc-1, _cellSize*_nc-1 );
      pointsRes[3]= cv::Point2f ( 0, _cellSize*_nc-1 );
      _perpTrans=cv::getPerspectiveTransform ( pointsIn,pointsRes );
      
double tick = (double)cv::getTickCount();	
		
      vector<cv::Point2f> transformedPixels;
      cv::perspectiveTransform(_pixelsVector, transformedPixels, _perpTrans);        
		
             AvrgTime.first+=((double)cv::getTickCount()-tick)/cv::getTickFrequency(); AvrgTime.second++;      
 	    cout<<"Time cc detection="<<1000*AvrgTime.first/AvrgTime.second<<" milliseconds"<<endl;		
      
 
            
      cv::Rect cellRect(0,0,_mc,_nc);
      for(unsigned int i=0; i<transformedPixels.size(); i++) {
	//_canonicalPos.at<cv::Vec2b>(_pixelsVector[i].y, _pixelsVector[i].x) = cv::Vec2b(transformedPixels[i].x, transformedPixels[i].y);
	transformedPixels[i].x /= _cellSize;
	transformedPixels[i].y /= _cellSize;
	if( !transformedPixels[i].inside(cellRect) ) {
	  _cellMap.at<uchar>( _pixelsVector[i].y, _pixelsVector[i].x ) = 0;
	  _cellMap.at<uchar>( _pixelsVector[i].y+1, _pixelsVector[i].x ) = 0;
	  _cellMap.at<uchar>( _pixelsVector[i].y, _pixelsVector[i].x+1 ) = 0;
	  _cellMap.at<uchar>( _pixelsVector[i].y+1, _pixelsVector[i].x+1 ) = 0;
	}
	else {
      uchar cellNum = (unsigned int)transformedPixels[i].y*_nc + (unsigned int)transformedPixels[i].x;
	  _cellMap.at<uchar>( _pixelsVector[i].y, _pixelsVector[i].x ) = 1+cellNum;
	  _cellMap.at<uchar>( _pixelsVector[i].y+1, _pixelsVector[i].x ) = 1+cellNum;	  
	  _cellMap.at<uchar>( _pixelsVector[i].y, _pixelsVector[i].x+1 ) = 1+cellNum;
	  _cellMap.at<uchar>( _pixelsVector[i].y+1, _pixelsVector[i].x+1 ) = 1+cellNum;	  
	}
      }
          
}



/**
 */
void ChromaticMask::train(const cv::Mat& in, const aruco::Board &board)
{
  calculateGridImage(board);
  
  for(unsigned int i=0; i<_classifiers.size(); i++) _classifiers[i].clearSamples();
  
  for(unsigned int i=0; i<in.rows; i++) {
    for(unsigned int j=0; j<in.cols; j++) {
      uchar idx = _cellMap.at<uchar>(i,j);
      if(idx!=0) _classifiers[idx-1].addSample( in.at<uchar>(i,j) );
    }
  }
  
  for(unsigned int i=0; i<_classifiers.size(); i++) _classifiers[i].train();
  
  
//   for(uint i=0; i<_mc; i++) {
//     for(uint j=0; j<_nc; j++) {
//       vector<uint> neighbors;
//       for(int k=-1; k<=1; k++) {
// 	if((i+k)<0 || (i+k)>=_mc) continue; 
// 	for(int l=-1; l<=1; l++) {
// 	  if((j+l)<0 || (j+l)>=_nc) continue;
// 	  neighbors.push_back((i+k)*_mc + (j+l));
// 	}
//       }
//       
//       
//       for(uint l=0; l<256; l++) {
// 	_classifiers[i*_mc+j].probConj[l] = 0.; 
// 	for(uint k=0; k<neighbors.size(); k++) {
// 	  _classifiers[i*_mc+j].probConj[l] +=  _classifiers[k].getProb(l);
// 	}
// 	_classifiers[i*_mc+j].probConj[l] +=  _classifiers[i*_mc+j].getProb(l); // center prob x2
// 	_classifiers[i*_mc+j].probConj[l] / (float)(neighbors.size()+1);
//       }
//       
//     }
//   }
  
  _isValid = true;
  
}


/**
 */
void ChromaticMask::classify(const cv::Mat& in, const aruco::Board &board)
{
  calculateGridImage(board);
  
  resetMask();
  
  for(unsigned int i=0; i<in.rows; i++) {
    const uchar* in_ptr = in.ptr<uchar>(i);
    const uchar* _cellMap_ptr = _cellMap.ptr<uchar>(i);
    for(unsigned int j=0; j<in.cols; j++) {
      uchar idx = _cellMap_ptr[j];
      if(idx!=0) {
	
	// considering neighbours
// 	float prob=0.0;
// 	float totalW = 0.0;
// 	cv::Point2d pix(i,j);
// 	for(uint k=0; k<_classifiers.size()-17; k++) {
// 	  float w = 1./(1.+getDistance(pix,k));
// 	  totalW += w;
// 	  prob += w*_classifiers[k].getProb(in_ptr[j] );
// 	}
// 	prob /= totalW;
// 	if(prob > _threshProb) _mask.at<uchar>(i,j)=1;
	
	// not considering neighbours
 	if(_classifiers[idx-1].classify( in.at<uchar>(i,j) )) _mask.at<uchar>(i,j)=1;
// 	if(_classifiers[idx-1].probConj[ in.at<uchar>(i,j)]>_threshProb ) _mask.at<uchar>(i,j)=1;

      }
    }
  } 
  
    // apply closing to mask
   cv::Mat maskClose;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
  cv::morphologyEx(_mask, maskClose, CV_MOP_CLOSE, element);  
  _mask = maskClose;
  
}

cv::Rect fitRectToSize(cv::Rect r,cv::Size size){
  
       r.x=max(r.x,0);
      r.y=max(r.y,0);
      int endx=r.x+r.width;
      int endy=r.y+r.height;
      endx=min(endx,size.width);
      endy=min(endy,size.height);
      r.width=endx-r.x;
      r.height=endy-r.y;
      return r;
 
}

/**
 */
void ChromaticMask::classify2(const cv::Mat& in, const aruco::Board &board)
{
  
    _mask.create(_CP.CamSize.height, _CP.CamSize.width, CV_8UC1); 
    _maskAux.create(_CP.CamSize.height, _CP.CamSize.width, CV_8UC1); 
      _maskAux.setTo(cv::Scalar::all(0));
  
      cv::projectPoints(_objCornerPoints, board.Rvec, board.Tvec, _CP.CameraMatrix, _CP.Distorsion, _imgCornerPoints);    
      //obtain the perspective transform
      cv::Point2f  pointsRes[4],pointsIn[4];
      for ( int i=0;i<4;i++ ) pointsIn[i]=_imgCornerPoints[i];

      pointsRes[0]= ( cv::Point2f ( 0,0 ) );
      pointsRes[1]= cv::Point2f ( _mc-1, 0 );
      pointsRes[2]= cv::Point2f ( _mc-1, _nc-1 );
      pointsRes[3]= cv::Point2f ( 0, _nc-1 );
      _perpTrans=cv::getPerspectiveTransform ( pointsIn,pointsRes );
      
      cv::Mat pT_32;
      _perpTrans.convertTo(pT_32,CV_32F);//RMS: CAMBIA A FLOAT       
      
//       cout << _perpTrans << endl;
      
      cv::Rect r = cv::boundingRect(_imgCornerPoints);
      r=fitRectToSize(r,in.size());//fit rectangle to image limits
      float *H=pT_32.ptr<float>(0);
// #pragma omp parallel for
      int ny=0;
      for(unsigned int y=r.y; y<r.y+r.height; y+=2,ny++) {
	const uchar* in_ptr = in.ptr<uchar>(y);
	uchar *_mask_ptr=_maskAux.ptr<uchar>(y);
	int startx=r.x+ny%2;//alternate starting point
    for(unsigned int x=startx; x<r.x+r.width; x+=2) {
	  cv::Point2f point;
	  float _inv_pointz = 1./ (x*H[6] + y*H[7] + H[8]);
 	  point.x =_inv_pointz*( x*H[0] + y*H[1] + H[2]);
 	  point.y =_inv_pointz*( x*H[3] + y*H[4] + H[5]);
	  cv::Point2i c;
	  c.x = int(point.x+0.5);
	  c.y = int(point.y+0.5);
	  if(c.x<0 || c.x>_mc-1 || c.y<0 || c.y>_nc-1) continue;
 	  size_t cell_idx=c.y*_mc+ c.x;//SGJ: revisar si esto esta bien!!
	  float prob=0.0,totalW=0.0,dist,w;
	  
      for(unsigned int k=0; k<_cell_neighbours[cell_idx].size(); k++) {
   	    dist = fabs(point.x-_centers[k].x)+fabs( point.y-_centers[k].y);
  	    w= (2-dist);
 	    w*=w;
// 	    w=1/(1- sqrt( (point.x-_centers[k].x)*(point.x-_centers[k].x) + (point.y-_centers[k].y)*(point.y-_centers[k].y));
  	    totalW += w;
  	    prob += w*_classifiers[ _cell_neighbours[cell_idx][k] ] .getProb(in_ptr[x] );
	  }
   	  prob /= totalW;
//    prob/=float(_classifiers.size()-17);
	  if(prob > _threshProb) {
	    _mask_ptr[x]=1;
	  }
	}
      }
//     // apply closing to mask
// _mask=_maskAux;
   cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
   cv::morphologyEx(_maskAux,_mask, CV_MOP_CLOSE, element);  
}



void ChromaticMask::update(const cv::Mat& in)
{
  cv::Mat maskCells;
  maskCells = _cellMap.mul(_mask);
  
  for(unsigned int i=0; i<_classifiers.size(); i++) _classifiers[i].clearSamples();
  
  for(unsigned int i=0; i<maskCells.rows; i++) {
      uchar* maskCells_ptr = maskCells.ptr<uchar>(i);
      const uchar* in_ptr = in.ptr<uchar>(i);
       for(unsigned int j=0; j<maskCells.cols; j++) {
	if(maskCells_ptr[j]!=0) _classifiers[maskCells_ptr[j]-1].addSample( in_ptr[j] );
       }
  }
  
  for(unsigned int i=0; i<_classifiers.size(); i++)
    if(_classifiers[i].numsamples() > 50) {
      _classifiers[i].train();
    }
  
    
    
}



void ChromaticMask::resetMask()
{
  
  _mask.create(_CP.CamSize.height, _CP.CamSize.width, CV_8UC1); 
  _mask.setTo(cv::Scalar::all(0));
}





















