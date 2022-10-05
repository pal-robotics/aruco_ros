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
#ifndef PicoFlann_H
#define PicoFlann_H
#include <vector>
#include <cassert>
#include <iostream>
#include <cstdint>
#include <limits>
#include <algorithm>
#include <cstring>
namespace picoflann
{


/**
 * @brief The KdTreeIndex class is the simplest an minimal method to use kdtrees.
 * You only must define an adapter, that tells the class how to access the i-th dimension
of your element. Here are two examples showing how easy it is:
 *
 * You can use any container providing the methods size() ant at().
 * You must create the adapter that must implement the method :  T operator( )(const E
&elem, int dim)const; T=float or double, E is your element type
 *
 * You can easily save/read the index to/from an stream using toStream() and fromStream().
Please notice that the data of the container is not copied in the KdTreeIndex so you
 * must be sure it is available when doing the searchs
 *
 * See examples below

 *
#include <random>
#include <vector>
#include <fstream>
#include "picoflann.h"
void example1(){
    //Data type
    struct Point2f{
        Point2f(float X,float Y) { x=X;y=Y; }
        float x,y;
    };

    // Adapter.
    // Given an Point2f element, it returns the element of the dimension specified such
that dim=0 is x and dim=1 is y struct PicoFlann_Point2fAdapter{ inline  float operator(
)(const Point2f &elem, int dim)const { return dim==0?elem.x:elem.y; }
    };

    //create the points randomly
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-1000.0,1000.0);
    std::vector<Point2f> data;
    for(size_t i=0;i<1000;i++)
        data.push_back( Point2f ( distribution(generator),distribution(generator)));
    ///------------------------------------------------------------
    /// Create the kdtree
    picoflann::KdTreeIndex<2,PicoFlann_Point2fAdapter>  kdtree;//2 is the number of
dimensions kdtree.build(data);
    //search 10 nearest neibors to point (0,0)
    std::vector<std::pair<uint32_t,double> > res=kdtree.searchKnn(data,Point2f(0,0),10);

    //radius search in a radius of 30 (the resulting distances are squared)
    res=kdtree.radiusSearch(data,Point2f(0,0),30);
    //another version
    kdtree.radiusSearch(res,data,Point2f(0,0),30);

    //you can save to a file
    std::ofstream file_out("out.bin",std::ios::binary);
    kdtree.toStream(file_out);

    //recover from the file
    picoflann::KdTreeIndex<2,PicoFlann_Point2fAdapter>  kdtree2;
    std::ifstream file_in("out.bin",std::ios::binary);
    kdtree2.fromStream(file_in);
    res=kdtree2.radiusSearch(data,Point2f(0,0),30);

}


//Using an array of 3d points
void example2(){

    struct Point3f{
        Point3f(float X,float Y,float Z) { data[0]=X;data[1]=Y;data[2]=Z; }
        float data[3];
    };
    struct PicoFlann_Array3f_Adapter{
        inline   float operator( )(const Point3f &elem, int dim)const{ return
elem.data[dim]; }
    };
    struct PicoFlann_Array3f_Container{
        const Point3f *_array;
        size_t _size;
        PicoFlann_Array3f_Container(float *array,size_t
Size):_array((Point3f*)array),_size(Size){} inline size_t size()const{return _size;} inline
const Point3f &at(int idx)const{ return _array [idx];}
    };
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-1000.0,1000.0);

    int nPoints=1000;
    float *array=new float[nPoints*3];
    for(size_t i=0;i<1000*3;i++)
        array[i]= distribution(generator);

    ///------------------------------------------------------------
    picoflann::KdTreeIndex<3,PicoFlann_Array3f_Adapter> kdtree;// 3 is the number of
dimensions, L2 is the type of distance kdtree.build(
PicoFlann_Array3f_Container(array,nPoints)); PicoFlann_Array3f_Container
p3container(array,nPoints); std::vector<std::pair<uint32_t,double> >
res=kdtree.searchKnn(p3container,Point3f(0,0,0),10);
    res=kdtree.radiusSearch(p3container,Point3f(0,0,0),30);
}
 *
 */
struct L2
{
  template <typename ElementType, typename ElementType2, typename Adapter>
  double compute_distance(const ElementType &elema, const ElementType2 &elemb,
                          const Adapter &adapter, int ndims, double worstDist) const
  {
    // compute dist
    double sqd = 0;
    for (int i = 0; i < ndims; i++)
    {
      double d = adapter(elema, i) - adapter(elemb, i);
      sqd += d * d;
      if (sqd > worstDist)
        return sqd;
    }
    return sqd;
  }
};

template <int DIMS, typename Adapter, typename DistanceType = L2>
class KdTreeIndex
{
public:
  /**
   *Builds the index using the data  passes in your container and the adapter
   */
  template <typename Container>
  inline void build(const Container &container)
  {
    _index.clear();
    _index.reserve(container.size() * 2);
    _index.dims = DIMS;
    _index.nValues = container.size();
    // Create root and assign all items
    all_indices.resize(container.size());
    for (size_t i = 0; i < container.size(); i++)
      all_indices[i] = i;
    if (container.size() == 0)
      return;
    computeBoundingBox<Container>(_index.rootBBox, 0, all_indices.size(), container);
    _index.push_back(Node());
    divideTree<Container>(_index, 0, 0, all_indices.size(), _index.rootBBox, container);
  }


  inline void clear()
  {
    _index.clear();
    all_indices.clear();
  }

  // saves to a stream. Note that the container is not saved!
  inline void toStream(std::ostream &str) const;
  // reads from an stream. Note that the container is not readed!
  inline void fromStream(std::istream &str);

  template <typename Type, typename Container>
  inline std::vector<std::pair<uint32_t, double> > searchKnn(const Container &container,
                                                             const Type &val, int nn,
                                                             bool sorted = true)
  {
    std::vector<std::pair<uint32_t, double> > res;
    generalSearch<Type, Container>(res, container, val, -1, sorted, nn);
    return res;
  }


  template <typename Type, typename Container>
  inline std::vector<std::pair<uint32_t, double> > radiusSearch(const Container &container,
                                                                const Type &val, double dist,
                                                                bool sorted = true,
                                                                int maxNN = -1) const
  {
    std::vector<std::pair<uint32_t, double> > res;
    generalSearch<Type, Container>(res, container, val, dist, sorted, maxNN);
    return res;
  }


  template <typename Type, typename Container>
  inline void radiusSearch(std::vector<std::pair<uint32_t, double> > &res,
                           const Container &container, const Type &val, double dist,
                           bool sorted = true, int maxNN = -1)
  {
    generalSearch<Type, Container>(res, container, val, dist, sorted, maxNN);
  }



private:
  struct Node
  {
    inline bool isLeaf() const
    {
      return _ileft == -1 && _iright == -1;
    }
    inline void setNodesInfo(uint32_t l, uint32_t r)
    {
      _ileft = l;
      _iright = r;
    }
    double div_val;
    uint16_t col_index;  // column index of the feature vector
    std::vector<int> idx;
    float divhigh, divlow;
    int64_t _ileft = -1, _iright = -1;  // children
    void toStream(std::ostream &str) const;
    void fromStream(std::istream &str);
  };


  typedef std::vector<std::pair<double, double> > BoundingBox;

  struct Index : public std::vector<Node>
  {
    BoundingBox rootBBox;
    int dims = 0;
    int nValues = 0;  // number of elements of the set when call to build
    inline void toStream(std::ostream &str) const;
    inline void fromStream(std::istream &str);
  };
  Index _index;
  DistanceType _distance;
  Adapter adapter;
  // next are only used during build
  std::vector<uint32_t> all_indices;
  int _maxLeafSize = 10;



  // temporal used during creation of the tree
  template <typename Container>
  void divideTree(Index &index, uint64_t nodeIdx, int startIndex, int endIndex,
                  BoundingBox &bbox, const Container &container)
  {
    // std::cout<<"CREATE="<<startIndex<<"-"<<endIndex<<"|";toStream(std::cout,bbox);
    Node &currNode = index[nodeIdx];
    int count = endIndex - startIndex;
    assert(startIndex < endIndex);

    if (count <= _maxLeafSize)
    {
      currNode.idx.resize(count);
      for (int i = 0; i < count; i++)
        currNode.idx[i] = all_indices[startIndex + i];
      computeBoundingBox<Container>(bbox, startIndex, endIndex, container);
      //  std::cout<<std::endl;
      return;
    }


    currNode.setNodesInfo(index.size(), index.size() + 1);
    index.push_back(Node());
    int leftNode = index.size() - 1;
    index.push_back(Node());
    int rightNode = index.size() - 1;


    /// SELECT THE COL (DIMENSION) ON WHICH PARTITION IS MADE
    if (0)
    {
      BoundingBox _bbox;
      computeBoundingBox<Container>(_bbox, startIndex, endIndex, container);
      //        //get the dimension with highest distnaces
      double max_spread = -1;
      currNode.col_index = 0;
      for (int i = 0; i < DIMS; i++)
      {
        double spread = _bbox[i].second - _bbox[i].first;  // maxV[i]-minV[i];
        if (spread > max_spread)
        {
          max_spread = spread;
          currNode.col_index = i;
        }
      }
      // select the split val
      double split_val = (bbox[currNode.col_index].first + bbox[currNode.col_index].second) / 2;
      if (split_val < _bbox[currNode.col_index].first)
        currNode.div_val = _bbox[currNode.col_index].first;
      else if (split_val > _bbox[currNode.col_index].second)
        currNode.div_val = _bbox[currNode.col_index].second;
      else
        currNode.div_val = split_val;
    }
    else
    {
      /// SELECT THE COL (DIMENSION) ON WHICH PARTITION IS MADE
      double var[DIMS], mean[DIMS];
      // compute the variance of the features to  select the highest one
      mean_var_calculate<Container>(startIndex, endIndex, var, mean, container);
      currNode.col_index = 0;
      // select element with highest variance
      for (int i = 1; i < DIMS; i++)
        if (var[i] > var[currNode.col_index])
          currNode.col_index = i;

      // now sort all indices according to the selected value

      currNode.div_val = mean[currNode.col_index];
    }



    // compute the variance of the features to  select the highest one
    // now sort all indices according to the selected value

    // std::cout<<" CUT FEAT="<<currNode.col_index<< " VAL="<<currNode.div_val<<std::endl;
    int lim1, lim2;
    planeSplit<Container>(&all_indices[startIndex], count, currNode.col_index,
                          currNode.div_val, lim1, lim2, container);

    int split_index;

    if (lim1 > count / 2)
      split_index = lim1;
    else if (lim2 < count / 2)
      split_index = lim2;
    else
      split_index = count / 2;

    //        /* If either list is empty, it means that all remaining features
    //              * are identical. Split in the middle to maintain a balanced tree.
    //              */
    if ((lim1 == count) || (lim2 == 0))
      split_index = count / 2;
    // create partitions with at least minLeafSize elements
    if (_maxLeafSize != 1)
      if (split_index < _maxLeafSize || count - split_index < _maxLeafSize)
      {
        std::sort(all_indices.begin() + startIndex, all_indices.begin() + endIndex,
                  [&](const uint32_t &a, const uint32_t &b)
                  {
                    return adapter(container.at(a), currNode.col_index) <
                           adapter(container.at(b), currNode.col_index);
                  });
        split_index = count / 2;
        currNode.div_val =
            adapter(container.at(all_indices[startIndex + split_index]), currNode.col_index);
      }



    //  currNode.div_val=_features.ptr<float>(all_indices[split_index])[currNode.col_index];

    BoundingBox left_bbox(bbox);
    left_bbox[currNode.col_index].second = currNode.div_val;
    divideTree<Container>(index, leftNode, startIndex, startIndex + split_index,
                          left_bbox, container);
    left_bbox[currNode.col_index].second = currNode.div_val;
    assert(left_bbox[currNode.col_index].second <= currNode.div_val);
    BoundingBox right_bbox(bbox);
    right_bbox[currNode.col_index].first = currNode.div_val;
    divideTree<Container>(index, rightNode, startIndex + split_index, endIndex,
                          right_bbox, container);

    currNode.divlow = left_bbox[currNode.col_index].second;
    currNode.divhigh = right_bbox[currNode.col_index].first;
    assert(currNode.divlow <= currNode.divhigh);

    for (int i = 0; i < DIMS; ++i)
    {
      bbox[i].first = std::min(left_bbox[i].first, right_bbox[i].first);
      bbox[i].second = std::max(left_bbox[i].second, right_bbox[i].second);
    }
  }



  template <typename Container>
  void computeBoundingBox(BoundingBox &bbox, int start, int end, const Container &container)
  {
    bbox.resize(DIMS);
    for (int i = 0; i < DIMS; ++i)
      bbox[i].second = bbox[i].first = adapter(container.at(all_indices[start]), i);

    for (int k = start + 1; k < end; ++k)
    {
      for (int i = 0; i < DIMS; ++i)
      {
        float v = adapter(container.at(all_indices[k]), i);
        if (v < bbox[i].first)
          bbox[i].first = v;
        if (v > bbox[i].second)
          bbox[i].second = v;
      }
    }
  }

  template <typename Container>
  void mean_var_calculate(int startindex, int endIndex, double var[], double mean[],
                          const Container &container)
  {
    const int MAX_ELEM_MEAN = 100;
    // recompute centers
    // compute new center
    memset(mean, 0, sizeof(double) * DIMS);
    double sum2[DIMS];
    memset(sum2, 0, sizeof(double) * DIMS);
    // finish when at least MAX_ELEM_MEAN elements computed
    int cnt = 0;
    // std::min(MAX_ELEM_MEAN,endIndex-startindex );
    int increment = 1;
    if (endIndex - startindex >= 2 * MAX_ELEM_MEAN)
      increment = (endIndex - startindex) / MAX_ELEM_MEAN;
    for (int i = startindex; i < endIndex; i += increment)
    {
      for (int c = 0; c < DIMS; c++)
      {
        auto val = adapter(container.at(all_indices[i]), c);
        mean[c] += val;
        sum2[c] += val * val;
      }
      cnt++;
    }

    double invcnt = 1. / double(cnt);
    for (int c = 0; c < DIMS; c++)
    {
      mean[c] *= invcnt;
      var[c] = sum2[c] * invcnt - mean[c] * mean[c];
    }
  }


  /**
   *  Subdivide the list of points by a plane perpendicular on axe corresponding
   *  to the 'cutfeat' dimension at 'cutval' position.
   *
   *  On return:
   *  dataset[ind[0..lim1-1]][cutfeat]<cutval
   *  dataset[ind[lim1..lim2-1]][cutfeat]==cutval
   *  dataset[ind[lim2..count]][cutfeat]>cutval
   */
  template <typename Container>
  void planeSplit(uint32_t *ind, int count, int cutfeat, float cutval, int &lim1,
                  int &lim2, const Container &container)
  {
    /* Move vector indices for left subtree to front of list. */
    int left = 0;
    int right = count - 1;
    for (;;)
    {
      while (left <= right && adapter(container.at(ind[left]), cutfeat) < cutval)
        ++left;
      while (left <= right && adapter(container.at(ind[right]), cutfeat) >= cutval)
        --right;
      if (left > right)
        break;
      std::swap(ind[left], ind[right]);
      ++left;
      --right;
    }
    lim1 = left;
    right = count - 1;
    for (;;)
    {
      while (left <= right && adapter(container.at(ind[left]), cutfeat) <= cutval)
        ++left;
      while (left <= right && adapter(container.at(ind[right]), cutfeat) > cutval)
        --right;
      if (left > right)
        break;
      std::swap(ind[left], ind[right]);
      ++left;
      --right;
    }
    lim2 = left;
  }


  template <typename Type>
  inline double computeInitialDistances(const Type &elem, double dists[],
                                        const BoundingBox &bbox) const
  {
    float distsq = 0.0;

    for (int i = 0; i < DIMS; ++i)
    {
      double elem_i = adapter(elem, i);
      if (elem_i < bbox[i].first)
      {
        auto d = elem_i - bbox[i].first;
        dists[i] = d * d;  // distance_.accum_dist(vec[i], root_bbox_[i].first, i);
        distsq += dists[i];
      }
      if (elem_i > bbox[i].second)
      {
        auto d = elem_i - bbox[i].second;
        dists[i] = d * d;  // distance_.accum_dist(vec[i], root_bbox_[i].second, i);
        distsq += dists[i];
      }
    }
    return distsq;
  }
  // THe function that does the search in all exact methods
  template <typename Type, typename Container>
  inline void generalSearch(std::vector<std::pair<uint32_t, double> > &res,
                            const Container &container, const Type &val, double dist,
                            bool sorted = true,
                            uint32_t maxNn = std::numeric_limits<int>::max()) const
  {
    double dists[DIMS];
    memset(dists, 0, sizeof(double) * DIMS);
    res.clear();
    ResultSet hres(res, maxNn, dist > 0 ? dist * dist : -1.f);
    float distsq = computeInitialDistances<Type>(val, dists, _index.rootBBox);
    searchExactLevel<Type, Container>(_index, 0, val, hres, distsq, dists, 1, container);
    if (sorted && res.size() > 1)
      std::sort(res.begin(), res.end(),
                [](const std::pair<uint32_t, double> &a, const std::pair<uint32_t, double> &b)
                { return a.second < b.second; });
  }

  // heap having at the top the maximum element

  class ResultSet
  {
  public:
    std::vector<std::pair<uint32_t, double> > &array;
    int maxSize;
    double maxValue = std::numeric_limits<double>::max();
    bool radius_search = false;

  public:
    ResultSet(std::vector<std::pair<uint32_t, double> > &data_ref,
              uint32_t MaxSize = std::numeric_limits<uint32_t>::max(), double MaxV = -1)
      : array(data_ref)
    {
      maxSize = MaxSize;
      // set value for radius search
      if (MaxV > 0)
      {
        maxValue = MaxV;
        radius_search = true;
      }
    }


    inline void push(const std::pair<uint32_t, double> &val)
    {
      if (radius_search && val.second < maxValue)
      {
        array.push_back(val);
      }
      else
      {
        if (array.size() >= size_t(maxSize))
        {
          // check if the maxium must be replaced by this
          if (val.second < array[0].second)
          {
            swap(array.front(), array.back());
            array.pop_back();
            if (array.size() > 1)
              up(0);
          }
          else
            return;
        }
        array.push_back(val);
        if (array.size() > 1)
          down(array.size() - 1);
      }
      //            array_size++;
    }

    inline double worstDist() const
    {
      if (radius_search)
        return maxValue;  // radius search
      else if (array.size() < size_t(maxSize))
        return std::numeric_limits<double>::max();
      return array[0].second;
    }
    inline double top() const
    {
      assert(!array.empty());
      return array[0].second;
    }

  private:
    inline void down(size_t index)
    {
      if (index == 0)
        return;
      size_t parentIndex = (index - 1) / 2;
      if (array[parentIndex].second < array[index].second)
      {
        swap(array[index], array[parentIndex]);
        down(parentIndex);
      }
    }
    inline void up(size_t index)
    {
      size_t leftIndex = 2 * index + 1;   // vl_heap_left_child (index) ;
      size_t rightIndex = 2 * index + 2;  // vl_heap_right_child (index) ;

      /* no childer: stop */
      if (leftIndex >= array.size())
        return;

      /* only left childer: easy */
      if (rightIndex >= array.size())
      {
        if (array[index].second < array[leftIndex].second)
          swap(array[index], array[leftIndex]);
        return;
      }

      /* both childern */
      {
        if (array[rightIndex].second < array[leftIndex].second)
        {
          /* swap with left */
          if (array[index].second < array[leftIndex].second)
          {
            swap(array[index], array[leftIndex]);
            up(leftIndex);
          }
        }
        else
        {
          /* swap with right */
          if (array[index].second < array[rightIndex].second)
          {
            swap(array[index], array[rightIndex]);
            up(rightIndex);
          }
        }
      }
    }
  };

  template <typename Type, typename Container>
  inline void searchExactLevel(const Index &index, int64_t nodeIdx, const Type &elem,
                               ResultSet &res, double mindistsq, double dists[],
                               double epsError, const Container &container) const
  {
    const Node &currNode = index[nodeIdx];
    if (currNode.isLeaf())
    {
      double worstDist = res.worstDist();
      for (size_t i = 0; i < currNode.idx.size(); i++)
      {
        double sqd = _distance.compute_distance(elem, container.at(currNode.idx[i]),
                                                adapter, DIMS, worstDist);
        if (sqd < worstDist)
        {
          res.push({ currNode.idx[i], sqd });
          worstDist = res.worstDist();
        }
      }
    }
    else
    {
      double val = adapter(elem, currNode.col_index);
      double diff1 = val - currNode.divlow;
      double diff2 = val - currNode.divhigh;

      uint32_t bestChild;
      uint32_t otherChild;
      double cut_dist;
      if ((diff1 + diff2) < 0)
      {
        bestChild = currNode._ileft;
        otherChild = currNode._iright;
        cut_dist = diff2 * diff2;
      }
      else
      {
        bestChild = currNode._iright;
        otherChild = currNode._ileft;
        cut_dist = diff1 * diff1;
      }
      /* Call recursively to search next level down. */
      searchExactLevel<Type, Container>(index, bestChild, elem, res, mindistsq, dists,
                                        epsError, container);

      float dst = dists[currNode.col_index];
      mindistsq = mindistsq + cut_dist - dst;
      dists[currNode.col_index] = cut_dist;
      if (mindistsq * epsError <= res.worstDist())
        searchExactLevel<Type, Container>(index, otherChild, elem, res, mindistsq, dists,
                                          epsError, container);
      dists[currNode.col_index] = dst;
    }
  }
};
template <int DIMS, typename AAdapter, typename DistanceType>
void KdTreeIndex<DIMS, AAdapter, DistanceType>::Node::toStream(std::ostream &str) const
{
  str.write((char *)&div_val, sizeof(div_val));
  str.write((char *)&col_index, sizeof(col_index));
  str.write((char *)&divhigh, sizeof(divhigh));
  str.write((char *)&divlow, sizeof(divlow));
  str.write((char *)&_ileft, sizeof(_ileft));
  str.write((char *)&_iright, sizeof(_iright));
  uint64_t s = idx.size();
  str.write((char *)&s, sizeof(s));
  str.write((char *)&idx[0], sizeof(idx[0]) * idx.size());
}

template <int DIMS, typename AAdapter, typename DistanceType>
void KdTreeIndex<DIMS, AAdapter, DistanceType>::Node::fromStream(std::istream &str)
{
  str.read((char *)&div_val, sizeof(div_val));
  str.read((char *)&col_index, sizeof(col_index));
  str.read((char *)&divhigh, sizeof(divhigh));
  str.read((char *)&divlow, sizeof(divlow));
  str.read((char *)&_ileft, sizeof(_ileft));
  str.read((char *)&_iright, sizeof(_iright));
  uint64_t s;
  str.read((char *)&s, sizeof(s));
  idx.resize(s);
  str.read((char *)&idx[0], sizeof(idx[0]) * idx.size());
}

template <int DIMS, typename AAdapter, typename DistanceType>
void KdTreeIndex<DIMS, AAdapter, DistanceType>::Index::toStream(std::ostream &str) const
{
  str.write((char *)&dims, sizeof(dims));
  str.write((char *)&rootBBox[0], sizeof(rootBBox[0]) * dims);
  str.write((char *)&nValues, sizeof(nValues));

  uint64_t s = std::vector<Node>::size();
  str.write((char *)&s, sizeof(s));
  for (size_t i = 0; i < std::vector<Node>::size(); i++)
    std::vector<Node>::at(i).toStream(str);
}

template <int DIMS, typename AAdapter, typename DistanceType>
void KdTreeIndex<DIMS, AAdapter, DistanceType>::Index::fromStream(std::istream &str)
{
  str.read((char *)&dims, sizeof(dims));
  rootBBox.resize(dims);
  str.read((char *)&rootBBox[0], sizeof(rootBBox[0]) * dims);
  str.read((char *)&nValues, sizeof(nValues));


  uint64_t s;
  ;
  str.read((char *)&s, sizeof(s));
  std::vector<Node>::resize(s);
  for (size_t i = 0; i < std::vector<Node>::size(); i++)
    std::vector<Node>::at(i).fromStream(str);
  if (dims != DIMS && this->size() != 0 && nValues != 0)
    throw std::runtime_error(
        "Number of dimensions of the index in the stream is different from the number of dimensions of this");
}

template <int DIMS, typename AAdapter, typename DistanceType>
void KdTreeIndex<DIMS, AAdapter, DistanceType>::toStream(std::ostream &str) const
{
  _index.toStream(str);
}

template <int DIMS, typename AAdapter, typename DistanceType>
void KdTreeIndex<DIMS, AAdapter, DistanceType>::fromStream(std::istream &str)
{
  _index.fromStream(str);
}
}  // namespace picoflann

#endif
