#include "fractalmarker.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <map>
#include "../aruco_export.h"

namespace aruco
{
class ARUCO_EXPORT FractalMarkerSet
{
public:
  enum CONF_TYPES : uint64_t
  {
    FRACTAL_2L_6 = 0,
    FRACTAL_3L_6 = 1,
    FRACTAL_4L_6 = 2,
    FRACTAL_5L_6 = 3,
    CUSTOM = 4  // for used defined dictionaries  (using load).
  };

  /**     create set of markers
   * @brief create
   * @param regionsConfig {N(f1),K(f1)}{N(f2):K(f2)}...{N(fn):K(fn)}
   * @param pixSize
   */
  void create(std::vector<std::pair<int, int>> regionsConfig, float pixSize);

  /**     configure bits of inner marker
   * @brief configureMat
   * @param nVal N region
   * @param kVal K region
   * @param maxIter Number of iteration
   * @return Mat configurated marker
   */
  cv::Mat configureMat(int nVal, int kVal, int maxIter = 10000);
  // computes the distance of a marker to itself
  int dstMarker(const cv::Mat m);

  // computes distance between marker to marker
  int dstMarkerToMarker(const cv::Mat m1, const cv::Mat m2);

  // computes distance between marker to set of markers
  int dstMarkerToFractalDict(cv::Mat m);

  // saves to a binary stream
  static void _toStream(FractalMarkerSet &configuration, std::ostream &str);

  // load from a binary stream
  static void _fromStream(FractalMarkerSet &configuration, std::istream &str);

  static bool isPredefinedConfigurationString(std::string str);

  static std::string getTypeString(FractalMarkerSet::CONF_TYPES t);

  static CONF_TYPES getTypeFromString(std::string str);

  static FractalMarkerSet load(std::string info);

  static FractalMarkerSet loadPredefined(std::string info);

  static FractalMarkerSet loadPredefined(CONF_TYPES info);

  static FractalMarkerSet readFromFile(std::string path);

  // saves configuration to a text file
  void saveToFile(cv::FileStorage &fs);

  // Fractal configuration. id_marker
  std::map<int, FractalMarker> fractalMarkerCollection;
  // Nbits_idmarkers
  std::map<int, std::vector<int>> nbits_fractalMarkerIDs;

  enum Fractal3DInfoType
  {
    NONE = -1,
    PIX = 0,
    METERS = 1,
    NORM = 2
  };  // indicates if the data in Fractal is expressed in meters or in pixels

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
  /**Indicates if the corners are normalized. -1..1 external marker
   */
  bool isNormalize() const
  {
    return mInfoType == NORM;
  }

  // Normalize fractal marker. The corners will go on to take the values (-1,1)(1,1),(1,-1)(-1,-1)
  FractalMarkerSet normalize();

  // Convert marker to meters
  FractalMarkerSet convertToMeters(float fractalSize_meters);

  static std::vector<std::string> getConfigurations();

  // Get fractal size (external marker)
  float getFractalSize() const
  {
    FractalMarker externalMarker = fractalMarkerCollection.at(_idExternal);
    return externalMarker.getMarkerSize();
  }

  // Get number of bits (external marker)
  int nBits() const
  {
    FractalMarker externalMarker = fractalMarkerCollection.at(_idExternal);
    return externalMarker.nBits();
  }

  // Check if m is a inner marker, and get its id.
  bool isFractalMarker(cv::Mat &m, int nbits, int &id);

  // Get all inners corners
  std::map<int, std::vector<cv::Point3f>> getInnerCorners();

  cv::Mat getFractalMarkerImage(int pixSize, bool border = false);

  // variable indicates if the data is expressed in meters or in pixels or are normalized
  int mInfoType; /* -1:NONE, 0:PIX, 1:METERS, 2:NORMALIZE*/

private:
  // Number of levels
  int _nmarkers;
  // ID external marker
  int _idExternal = 0;
  // Configuration dictionary
  std::string config;
};
}  // namespace aruco
