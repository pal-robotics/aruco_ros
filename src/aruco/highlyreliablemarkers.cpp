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

#include <aruco/highlyreliablemarkers.h>

namespace aruco {

  // static variables from HighlyReliableMarkers. Need to be here to avoid linking errors
  Dictionary HighlyReliableMarkers::_D;
  HighlyReliableMarkers::BalancedBinaryTree HighlyReliableMarkers::_binaryTree;
  unsigned int HighlyReliableMarkers::_n, HighlyReliableMarkers::_ncellsBorder, HighlyReliableMarkers::_correctionDistance;
  int HighlyReliableMarkers::_swidth;


  /**
  */
  MarkerCode::MarkerCode(unsigned int n) {
    // resize bits vectors and initialize to 0
    for(unsigned int i=0; i<4; i++) {
      _bits[i].resize(n*n); 
      for(unsigned int j=0; j<_bits[i].size(); j++) _bits[i][j]=0;
      _ids[i] = 0; // ids are also 0
    }
    _n = n;
  };
  
  
  /**
   */
  MarkerCode::MarkerCode(const MarkerCode& MC)
  {
    for(unsigned int i=0; i<4; i++) {
      _bits[i] = MC._bits[i];
      _ids[i] = MC._ids[i];
    }
    _n = MC._n;
  }

  
  
  /**
   */
  void MarkerCode::set(unsigned int pos, bool val) {
    // if not the same value
    if( get(pos) != val ) {
      for(unsigned int i=0; i<4; i++) { // calculate bit coordinates for each rotation
    unsigned int y=pos/n(), x=pos%n(); // if rotation 0, dont do anything
	// else calculate bit position in that rotation
    if(i==1) { unsigned int aux=y; y=x; x=n()-aux-1; }
	else if(i==2) { y=n()-y-1; x=n()-x-1; }
    else if(i==3) { unsigned int aux=y; y=n()-x-1; x=aux; }
    unsigned int rotPos = y*n()+x; // calculate position in the unidimensional string
	_bits[i][rotPos] = val; // modify value
	// update identifier in that rotation
    if(val==true) _ids[i] += (unsigned int)pow(float(2),float(rotPos)); // if 1, add 2^pos
    else _ids[i] -= (unsigned int)pow(float(2),float(rotPos)); // if 0, substract 2^pos
      }   
    }
  }
  
  
  /**
   */
  unsigned int MarkerCode::selfDistance(unsigned int &minRot) {
    unsigned int res = _bits[0].size(); // init to n*n (max value)
    for(unsigned int i=1; i<4; i++) { // self distance is not calculated for rotation 0
      unsigned int hammdist = hammingDistance(_bits[0], _bits[i]);
      if(hammdist<res) {
	minRot = i;
	res = hammdist;
      }
    }
    return res;
  }
  
  
  /**
   */
  unsigned int MarkerCode::distance(MarkerCode m, unsigned int &minRot) {
    unsigned int res = _bits[0].size(); // init to n*n (max value)
    for(unsigned int i=0; i<4; i++) {
      unsigned int hammdist = hammingDistance(_bits[0], m.getRotation(i));
      if(hammdist<res) {
	minRot = i;
	res = hammdist;
      }      
    }
    return res;    
  };
  

  /**
   */
  void MarkerCode::fromString(std::string s)
  {
    for(unsigned int i=0; i<s.length(); i++) {
      if(s[i]=='0') set(i, false);
      else set(i, true);
    }
  }

  /**
   */
  std::string MarkerCode::toString()
  {
    std::string s;
    s.resize(size());
    for(unsigned int i=0; i<size(); i++) {
      if(get(i)) s[i]='1';
      else s[i]='0';
    }
    return s;
  }  
  
  
  /**
   */
  cv::Mat MarkerCode::getImg(unsigned int pixSize) {
    const unsigned int borderSize=1;
    unsigned int nrows = n()+2*borderSize;
    if(pixSize%nrows != 0) pixSize = pixSize + nrows - pixSize%nrows;
    unsigned int cellSize = pixSize / nrows;
    cv::Mat img(pixSize, pixSize, CV_8U, cv::Scalar::all(0)); // create black image (init image to 0s)
    // double for to go over all the cells
    for(unsigned int i=0; i<n(); i++) {
      for(unsigned int j=0; j<n(); j++) {
	if(_bits[0][i*n()+j]!=0) { // just draw if it is 1, since the image has been init to 0
	  // double for to go over all the pixels in the cell
      for(unsigned int k=0; k<cellSize; k++) {
        for(unsigned int l=0; l<cellSize; l++) {
	      img.at<uchar>((i+borderSize)*cellSize+k, (j+borderSize)*cellSize+l) = 255;
	    }
	  }
	}
      }
    }
    return img;    
  }  
  
  
  /**
   */
  unsigned int MarkerCode::hammingDistance(std::vector<bool> m1, std::vector<bool> m2) {
    unsigned int res=0;
    for(unsigned int i=0; i<m1.size(); i++)
      if(m1[i]!=m2[i]) res++;
    return res;
  };  
  
  
  
  
  
  
   /**
   */
  bool Dictionary::fromFile(std::string filename) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    int nmarkers, markersize;
    
    // read number of markers
    fs["nmarkers"] >> nmarkers; // cardinal of D
    fs["markersize"] >> markersize; // n
       
    // read each marker info
    for (int i=0; i<nmarkers; i++) {  
	std::string s;
        fs["marker_" + toStr(i)] >> s;
	MarkerCode m(markersize);
	m.fromString(s);
 	push_back(m);
    }
    fs.release();   
    return true;
  };
  
  /**
   */
  bool Dictionary::toFile(std::string filename) {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    // save number of markers
    fs << "nmarkers" << (int)size(); // cardinal of D 
    fs << "markersize" << (int)( (*this)[0].n() ); // n
    // save each marker code
    for (unsigned int i=0; i<size(); i++) {
	std::string s =  ((*this)[i]).toString();
        fs << "marker_" + toStr(i) << s;
    }
    fs.release();  
    return true;
  };
  
  /**
   */
  unsigned int Dictionary::distance(MarkerCode m, unsigned int &minMarker, unsigned int &minRot) {
    unsigned int res = m.size();
    for(unsigned int i=0; i<size(); i++) {
      unsigned int minRotAux;
      unsigned int distance = (*this)[i].distance(m, minRotAux);
      if(distance<res) {
	minMarker = i;
	minRot = minRotAux;
	res=distance;
      }
    }
    return res;
  }
  
  
  /**
   */
  unsigned int Dictionary::minimunDistance()
  {
    if(size()==0) return 0;
    unsigned int minDist = (*this)[0].size();
    // for each marker in D
    for(unsigned int i=0; i<size(); i++) {
      // calculate self distance of the marker
      minDist = std::min( minDist, (*this)[i].selfDistance() );
      
      // calculate distance to all the following markers
      for(unsigned int j=i+1; j<size(); j++) {
	minDist = std::min( minDist, (*this)[i].distance((*this)[j]) );
      }
    }
    return minDist;
  }
  
  
  
  
  
  
  
  
  
  
  /**
   */
  bool HighlyReliableMarkers::loadDictionary(Dictionary D){  
    if(D.size()==0) return false;
    _D = D;
    _n = _D[0].n();
    _ncellsBorder = (_D[0].n()+2);
    _correctionDistance = (unsigned int)floor( (_D.minimunDistance()-1)/2. ); //maximun correction distance
    
    _binaryTree.loadDictionary(&D);   
    
    return true;
    
  };
  
  bool HighlyReliableMarkers::loadDictionary(std::string filename){ 
    Dictionary D;
    D.fromFile(filename); 
    return loadDictionary(D);
  };  
    
  
  /**
   */
  int HighlyReliableMarkers::detect(const cv::Mat& in, int& nRotations)
  {  

    assert(in.rows==in.cols);
    cv::Mat grey;
    if ( in.type()==CV_8UC1) grey=in;
    else cv::cvtColor(in,grey,CV_BGR2GRAY);
    //threshold image
    cv::threshold(grey, grey,125, 255, cv::THRESH_BINARY|cv::THRESH_OTSU);
    _swidth=grey.rows/_ncellsBorder;    
    
    // check borders, even not necesary for the highly reliable markers
    //if(!checkBorders(grey)) return -1; 
    
    // obtain inner code
    MarkerCode candidate = getMarkerCode(grey);

    // search each marker id in the balanced binary tree
    unsigned int orgPos;
    for(unsigned int i=0; i<4; i++) {
      if(_binaryTree.findId( candidate.getId(i), orgPos )) {
	  nRotations = i;
	  return candidate.getId(i);
 	  //return orgPos;
      }
    }
    
    // alternative version without using the balanced binary tree (less eficient)
    //     for(uint i=0; i<_D.size(); i++) {
    //       for(uint j=0; j<4; j++) {
    // 	if(_D[i].getId() == candidate.getId(j)) {
    // 	  nRotations = j;
    // 	  return candidate.getId(j);
    // 	}
    //       }
    //     }
    
    // correct errors
    unsigned int minMarker, minRot;
    if(_D.distance(candidate, minMarker, minRot) <= _correctionDistance) {
      nRotations = minRot;
      //return minMarker;
     return _D[minMarker].getId();
    }
        
    return -1;

  }  
  
  
  /**
   */
  bool HighlyReliableMarkers::checkBorders(cv::Mat grey) {
    for (int y=0;y<_ncellsBorder;y++)
    {
        int inc=_ncellsBorder-1;
        if (y==0 || y==_ncellsBorder-1) inc=1;//for first and last row, check the whole border
        for (int x=0;x<_ncellsBorder;x+=inc)
        {
            int Xstart=(x)*(_swidth);
            int Ystart=(y)*(_swidth);
            cv::Mat square=grey(cv::Rect(Xstart,Ystart,_swidth,_swidth));
            int nZ=cv::countNonZero(square);
            if (nZ> (_swidth*_swidth) /2) {
                return false;//can not be a marker because the border element is not black!
            }
        }
    }    
    return true;
  }
  
  /**
   */
  MarkerCode HighlyReliableMarkers::getMarkerCode(cv::Mat grey) {
    MarkerCode candidate( _n );
    for (int y=0;y<_n;y++)
    {
        for (int x=0;x<_n;x++)
        {
            int Xstart=(x+1)*(_swidth);
            int Ystart=(y+1)*(_swidth);
            cv::Mat square=grey(cv::Rect(Xstart,Ystart,_swidth,_swidth));
            int nZ=countNonZero(square);
            if (nZ> (_swidth*_swidth) /2)  candidate.set(y*_n+x, 1);
        }
     } 
     return candidate;
   };
   
   
  
  /**
   */   
  void HighlyReliableMarkers::BalancedBinaryTree::loadDictionary(Dictionary *D) {
    // create _orderD wich is a sorted version of D
    _orderD.clear();
    for(unsigned int i=0; i<D->size(); i++) {
      _orderD.push_back( std::pair<unsigned int,unsigned int>( (*D)[i].getId() ,i) );
    }
    std::sort(_orderD.begin(), _orderD.end());
    
    // calculate the number of levels of the tree 
    unsigned int levels=0;
    while( pow(float(2),float(levels)) <= _orderD.size() ) levels++;
    //       levels-=1; // only count full levels
    
    // auxiliar vector to know which elements are already in the tree
    std::vector<bool> visited;
    visited.resize(_orderD.size());
    for(unsigned int i=0; i<_orderD.size(); i++) visited[i]=false;
    
    // calculate position of the root element
    unsigned int rootIdx = _orderD.size()/2;
    visited[rootIdx] = true; // mark it as visited
    
    // auxiliar vector to store the ids intervals (max and min) during the creation of the tree
    std::vector< std::pair<unsigned int, unsigned int> > intervals;
    // first, add the two intervals at each side of root element
    intervals.push_back( std::pair<unsigned int, unsigned int>(0,rootIdx) );
    intervals.push_back( std::pair<unsigned int, unsigned int>(rootIdx,_orderD.size()) );

    // init the tree
    _binaryTree.clear();
    _binaryTree.resize(_orderD.size());
    
    // add root information to the tree (make sure child do not coincide with self root for small sizes of D)
    if(!visited[(0+rootIdx)/2]) _binaryTree[rootIdx].first = (0+rootIdx)/2;
    else _binaryTree[rootIdx].first = -1;
    if(!visited[(rootIdx+_orderD.size())/2]) _binaryTree[rootIdx].second = (rootIdx+_orderD.size())/2;
    else _binaryTree[rootIdx].second = -1;
	    
    // for each tree level
    for(unsigned int i=1; i<levels; i++) {
      unsigned int nintervals = intervals.size(); // count number of intervals and process them
      for(unsigned int j=0; j<nintervals; j++) {
	// store interval information and delete it
    unsigned int lowerBound, higherBound;
	lowerBound = intervals.back().first;
	higherBound = intervals.back().second;
	intervals.pop_back();
	
	// center of the interval
    unsigned int center = (higherBound + lowerBound)/2;
	
	// if center not visited, continue
	if(!visited[center])	visited[center]=true;
	else continue;
	
	// calculate centers of the child intervals
    unsigned int lowerChild = (lowerBound + center)/2;
    unsigned int higherChild = (center + higherBound)/2;
	
	// if not visited (lower child)
	if(!visited[lowerChild]) {
      intervals.insert( intervals.begin(), std::pair<unsigned int, unsigned int>(lowerBound, center) ); // add the interval to analyze later
	  _binaryTree[center].first = lowerChild; // add as a child in the tree
	}
	else _binaryTree[center].first = -1; // if not, mark as no child

	// (higher child, same as lower child)
	if(!visited[higherChild]) {
      intervals.insert( intervals.begin(), std::pair<unsigned int, unsigned int>(center, higherBound) );
	  _binaryTree[center].second = higherChild;
	}
	else _binaryTree[center].second = -1;
	
      }
    }      
    
    // print tree
    // for(uint i=0; i<_binaryTree.size(); i++) std::cout << _binaryTree[i].first << " " << _binaryTree[i].second << std::endl;
    // std::cout << std::endl;
    
  };   
  
  
  /**
   */     
  bool HighlyReliableMarkers::BalancedBinaryTree::findId(unsigned int id, unsigned int &orgPos) {
    unsigned int pos = _root; // first position is root
    while(pos!=-1) { // while having a valid position
      unsigned int posId = _orderD[pos].first; // calculate id of the node
      if(posId == id ) {
	orgPos = _orderD[pos].second;
	return true; // if is the desire id, return true
      }
      else if(posId < id) pos = _binaryTree[pos].second; // if desired id is higher, look in higher child
      else pos = _binaryTree[pos].first; // if it is lower, look in lower child
    }
    return false; // if nothing found, return false
  }
  
  
};
