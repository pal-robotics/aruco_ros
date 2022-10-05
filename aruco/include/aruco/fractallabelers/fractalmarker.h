#ifndef FRACTALMARKER_H
#define FRACTALMARKER_H

#include <vector>
#include <bitset>
#include <opencv2/imgproc/imgproc.hpp>
#include "../markermap.h"

namespace aruco
{
class FractalMarker : public aruco::Marker3DInfo
{
public:
  FractalMarker();
  FractalMarker(int id, cv::Mat m, std::vector<cv::Point3f> corners,
                std::vector<int> id_submarkers);

  // Add new submarker
  void addSubFractalMarker(FractalMarker submarker);

  // Find inner corners
  std::vector<cv::Point3f> findInnerCorners();

  // Marker MAT
  const cv::Mat mat() const
  {
    return _M;
  }

  // Marker mask (mask applied to submarkers)
  const cv::Mat mask() const
  {
    return _mask;
  }

  // Total number of bits
  int nBits()
  {
    return _M.total();
  }

  // Submarkers ids
  std::vector<int> subMarkers()
  {
    return _submarkers;
  }

  // Get inner corners
  std::vector<cv::Point3f> getInnerCorners()
  {
    if (innerCorners.empty())
      innerCorners = findInnerCorners();

    return innerCorners;
  }

private:
  cv::Mat _M;
  cv::Mat _mask;
  std::vector<int> _submarkers;  // id subfractalmarkers
  std::vector<cv::Point3f> innerCorners;
};
}  // namespace aruco

#endif  // FRACTALMARKER_H
