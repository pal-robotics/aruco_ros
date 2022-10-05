#include "fractallabelers/fractallabeler.h"

#include "aruco_cvversioning.h"
namespace aruco
{

void FractalMarkerLabeler::setConfiguration(const FractalMarkerSet& fractMarkerSet)
{
  _fractalMarkerSet = fractMarkerSet;
}

bool FractalMarkerLabeler::detect(const cv::Mat& in, int& marker_id, int& nRotations,
                                  std::string& additionalInfo)
{
  assert(in.rows == in.cols);
  cv::Mat grey;
  if (in.type() == CV_8UC1)
    grey = in;
  else
    cv::cvtColor(in, grey, CV_BGR2GRAY);
  // threshold image
  cv::threshold(grey, grey, 125, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  std::map<uint32_t, std::vector<cv::Mat> > nbits_innerCodes;

  for (auto bitsids : _fractalMarkerSet.nbits_fractalMarkerIDs)
  {
    int nbits = bitsids.first;
    std::vector<cv::Mat> innerCodes;
    getInnerCode(grey, nbits, innerCodes);

    if (innerCodes.size() > 0)
    {
      if (sum(innerCodes[0])[0] != 0)
      {
        nbits_innerCodes[nbits] = innerCodes;
      }
    }
  }

  if (nbits_innerCodes.size() == 0)
    return false;

  // check if any dictionary recognizes it
  for (auto bit_innerCodes : nbits_innerCodes)
  {
    uint32_t nb = bit_innerCodes.first;
    auto innerCodes = bit_innerCodes.second;

    for (int i = 0; i < 4; i++)
    {
      if (_fractalMarkerSet.isFractalMarker(innerCodes[i], nb, marker_id))
      {
        // is in the set?
        nRotations = i;  // how many rotations are and its id
        return true;     // bye bye
      }
    }
  }
  return false;
}


std::string FractalMarkerLabeler::getName() const
{
  return "fractal";
  ;
}

bool FractalMarkerLabeler::getInnerCode(const cv::Mat& thres_img, int total_nbits,
                                        std::vector<cv::Mat>& innerCodes)
{
  int bits_noborder = static_cast<int>(std::sqrt(total_nbits));
  int bits_withborder = bits_noborder + 2;
  // Markers  are divided in (bits_a+2)x(bits_a+2) regions, of which the inner
  // bits_axbits_a belongs to marker info the external border shoould be entirely black
  cv::Mat nonZeros(bits_withborder, bits_withborder, CV_32SC1);
  cv::Mat nValues(bits_withborder, bits_withborder, CV_32SC1);
  nonZeros.setTo(cv::Scalar::all(0));
  nValues.setTo(cv::Scalar::all(0));
  for (int y = 0; y < thres_img.rows; y++)
  {
    const uchar* ptr = thres_img.ptr<uchar>(y);
    int my = float(bits_withborder) * float(y) / float(thres_img.rows);
    for (int x = 0; x < thres_img.cols; x++)
    {
      int mx = float(bits_withborder) * float(x) / float(thres_img.cols);
      if (ptr[x] > 125)
        nonZeros.at<int>(my, mx)++;
      nValues.at<int>(my, mx)++;
    }
  }
  cv::Mat binaryCode(bits_withborder, bits_withborder, CV_8UC1);
  // now, make the theshold
  for (int y = 0; y < bits_withborder; y++)
    for (int x = 0; x < bits_withborder; x++)
    {
      if (nonZeros.at<int>(y, x) > nValues.at<int>(y, x) / 2)
        binaryCode.at<uchar>(y, x) = 1;
      else
        binaryCode.at<uchar>(y, x) = 0;
    }

  // check if border is completely black
  for (int y = 0; y < bits_withborder; y++)
  {
    int inc = bits_withborder - 1;
    if (y == 0 || y == bits_withborder - 1)
      inc = 1;  // for first and last row, check the whole border
    for (int x = 0; x < bits_withborder; x += inc)
      if (binaryCode.at<uchar>(y, x) != 0)
        return false;
  }

  // take the inner code
  cv::Mat _bits(bits_noborder, bits_noborder, CV_8UC1);
  for (int y = 0; y < bits_noborder; y++)
    for (int x = 0; x < bits_noborder; x++)
      _bits.at<uchar>(y, x) = binaryCode.at<uchar>(y + 1, x + 1);

  // now, get the 64bits ids
  int nr = 0;
  do
  {
    innerCodes.push_back(_bits);
    _bits = rotate(_bits);
    nr++;
  } while (nr < 4);
  return true;
}

cv::Mat FractalMarkerLabeler::rotate(const cv::Mat& in)
{
  cv::Mat out;
  in.copyTo(out);
  for (int i = 0; i < in.rows; i++)
  {
    for (int j = 0; j < in.cols; j++)
    {
      out.at<uchar>(i, j) = in.at<uchar>(in.cols - j - 1, i);
    }
  }
  return out;
}
}  // namespace aruco
