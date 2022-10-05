#include "../markerlabeler.h"
#include "fractalposetracker.h"

namespace aruco
{
class FractalMarkerLabeler : public MarkerLabeler
{
public:
  static cv::Ptr<FractalMarkerLabeler> create(std::string params)
  {
    FractalMarkerSet fractalMarkerSet = FractalMarkerSet::load(params);
    FractalMarkerLabeler* fml = new FractalMarkerLabeler();
    fml->setConfiguration(fractalMarkerSet);
    return fml;
  }

  static cv::Ptr<FractalMarkerLabeler> create(FractalMarkerSet::CONF_TYPES conf)
  {
    FractalMarkerSet fractalMarkerSet = FractalMarkerSet::loadPredefined(conf);
    FractalMarkerLabeler* fml = new FractalMarkerLabeler();
    fml->setConfiguration(fractalMarkerSet);
    return fml;
  }

  void setConfiguration(const FractalMarkerSet& fractMarkerSet);

  static bool isFractalDictionaryFile(const std::string& path);

  virtual ~FractalMarkerLabeler()
  {
  }

  bool load(const std::string& path);

  // returns the configuration name
  std::string getName() const;

  // main virtual class to o detection
  bool detect(const cv::Mat& in, int& marker_id, int& nRotations, std::string& additionalInfo);

  int getNSubdivisions() const
  {
    return (sqrt(_fractalMarkerSet.nBits()) + 2);
  }

  FractalMarkerSet _fractalMarkerSet;

private:
  bool getInnerCode(const cv::Mat& thres_img, int total_nbits, std::vector<cv::Mat>& ids);
  cv::Mat rotate(const cv::Mat& in);
};
}  // namespace aruco
