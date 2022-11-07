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

#ifndef _Aruco_MarkerMap_h
#define _Aruco_MarkerMap_h

#include "aruco_export.h"
#include "marker.h"

#include <opencv2/core/core.hpp>

#include <string>
#include <vector>

namespace aruco
{
/**
 * 3d representation of a marker
 */
class ARUCO_EXPORT Marker3DInfo
{
public:
  std::vector<cv::Point3f> points;
  int id;  // maker id

  Marker3DInfo();
  Marker3DInfo(int _id);
  inline bool operator==(const Marker3DInfo& MI)
  {
    return id == MI.id;
  }

  // returns the distance of the marker side
  inline float getMarkerSize() const
  {
    return static_cast<float>(cv::norm(points[0] - points[1]));
  }
  inline cv::Point3f at(size_t idx) const
  {
    return points.at(idx);
  }
  inline cv::Point3f& operator[](size_t idx)
  {
    return points[idx];
  }
  inline const cv::Point3f& operator[](size_t idx) const
  {
    return points[idx];
  }
  inline void push_back(const cv::Point3f& p)
  {
    points.push_back(p);
  }
  inline size_t size() const
  {
    return points.size();
  }

public:
  inline void toStream(std::ostream& str)
  {
    str << id << " " << size() << " ";
    for (size_t i = 0; i < size(); i++)
      str << at(i).x << " " << at(i).y << " " << at(i).z << " ";
  }
  inline void fromStream(std::istream& str)
  {
    int s;
    str >> id >> s;
    points.resize(s);
    for (size_t i = 0; i < size(); i++)
      str >> points[i].x >> points[i].y >> points[i].z;
  }
};

/**\brief This class defines a set of markers whose locations are attached to a common
 * reference system, i.e., they do not move wrt each other. A MarkerMap contains several
 * markers so that they are more robustly detected.
 *
 * A MarkerMap is only a list  of the id of the markers along with the position of their
 * corners. A MarkerMap may have information about the dictionary the markers belongs to
 * @see getDictionary()
 *
 * The position of the corners can be specified either in pixels (in a non-specific size)
 * or in meters. The first is the typical case in which you generate the image of  board
 * and the print it. Since you do not know in advance the real size of the markers, their
 * corners are specified in pixels, and then, the translation to meters can be made once
 * you know the real size.
 *
 * On the other hand, you may want to have the information of your boards in meters. The
 * MarkerMap allows you to do so.
 *
 * The point is in the mInfoType variable. It can be either PIX or METERS according to
 * your needs.
 *
 */

class ARUCO_EXPORT MarkerMap : public std::vector<Marker3DInfo>
{
public:
  /**
   */
  MarkerMap();

  /**Loads from file
   * @param filePath to the config file
   */
  MarkerMap(std::string filePath);

  /**Indicates if the corners are expressed in meters
   */
  bool isExpressedInMeters() const
  {
    return mInfoType == METERS;
  }
  /**Indicates if the corners are expressed in meters
   */
  bool isExpressedInPixels() const
  {
    return mInfoType == PIX;
  }
  /**converts the passed board into meters
   */
  MarkerMap convertToMeters(float markerSize) const;
  // simple way of knowing which elements detected in an image are from this markermap
  // returns the indices of the elements in the vector 'markers' that belong to this set
  // Example: The set has the elements with ids 10,21,31,41,92
  // The input vector has the markers with ids 10,88,9,12,41
  // function returns {0,4}, because element 0 (10) of the vector belongs to the set, and
  // also element 4 (41) belongs to the set
  std::vector<int> getIndices(const vector<Marker>& markers) const;

  /**Returns the Info of the marker with id specified. If not in the set, throws exception
   */
  const Marker3DInfo& getMarker3DInfo(int id) const;

  /**Returns the index of the marker (in this object) with id indicated, if is in the vector
   */
  int getIndexOfMarkerId(int id) const;
  /**Set in the list passed the set of the ids
   */
  void getIdList(vector<int>& ids, bool append = true) const;

  /**Returns an image of this to be printed. This object must be in pixels @see
   * isExpressedInPixels(). If not,please provide the METER2PIX conversion parameter
   */
  cv::Mat getImage(float METER2PIX = 0) const;

  /**Saves the board info to a file
   */
  void saveToFile(std::string sfile);
  /**Reads board info from a file
   */
  void readFromFile(std::string sfile);

  // calculates the camera location w.r.t. the map using the information provided
  // returns the <rvec,tvec>
  std::pair<cv::Mat, cv::Mat> calculateExtrinsics(const std::vector<aruco::Marker>& markers,
                                                  float markerSize, cv::Mat CameraMatrix,
                                                  cv::Mat Distorsion);

  // returns string indicating the dictionary
  inline std::string getDictionary() const
  {
    return dictionary;
  }

  enum Marker3DInfoType
  {
    NONE = -1,
    PIX = 0,
    METERS = 1
  };  // indicates if the data in MakersInfo is expressed in meters or in pixels so as to do conversion internally
  // returns string indicating the dictionary
  void setDictionary(std::string d)
  {
    dictionary = d;
  }

  // variable indicates if the data in MakersInfo is expressed in meters or in pixels so
  // as to do conversion internally
  int mInfoType;

private:
  // dictionary it belongs to (if any)
  std::string dictionary;

private:
  /**Saves the board info to a file
   */
  void saveToFile(cv::FileStorage& fs);
  /**Reads board info from a file
   */
  void readFromFile(cv::FileStorage& fs);

public:
  void toStream(std::ostream& str);
  void fromStream(std::istream& str);
};
}  // namespace aruco

#endif
