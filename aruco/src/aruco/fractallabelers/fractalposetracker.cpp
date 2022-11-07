#include "fractallabelers/fractalposetracker.h"
#include "levmarq.h"
#include "ippe.h"
#include "aruco_export.h"
#include "timers.h"


#include "opencv2/calib3d/calib3d.hpp"
#include "aruco_cvversioning.h"

namespace aruco
{

/*
 * KeyPoint cornersubpixel
 */
void kcornerSubPix(const cv::Mat image, std::vector<cv::KeyPoint> &kpoints)
{
  std::vector<cv::Point2f> points;
  cv::KeyPoint::convert(kpoints, points);

  cv::Size winSize = cv::Size(4, 4);
  cv::Size zeroZone = cv::Size(-1, -1);
  cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 12,
                            0.005);  // cv::cornerSubPix(image, points, winSize, zeroZone, criteria);
  cornerSubPix(image, points, winSize, zeroZone, criteria);

  // Update kpoints
  uint32_t i = 0;
  for (auto &k : kpoints)
  {
    k.pt = points[i];
    i++;
  }
}

/*
 * KeyPoints Filter. Delete kpoints with low response and duplicated.
 */
void kfilter(std::vector<cv::KeyPoint> &kpoints)
{
  float minResp = kpoints[0].response;
  float maxResp = kpoints[0].response;
  for (auto &p : kpoints)
  {
    p.size = 40;
    if (p.response < minResp)
      minResp = p.response;
    if (p.response > maxResp)
      maxResp = p.response;
  }
  float thresoldResp = (maxResp - minResp) * 0.20f + minResp;

  for (uint32_t xi = 0; xi < kpoints.size(); xi++)
  {
    // Erase keypoints with low response (20%)
    if (kpoints[xi].response < thresoldResp)
    {
      kpoints[xi].size = -1;
      continue;
    }

    // Duplicated keypoints (closer)
    for (uint32_t xj = xi + 1; xj < kpoints.size(); xj++)
    {
      if (pow(kpoints[xi].pt.x - kpoints[xj].pt.x, 2) +
              pow(kpoints[xi].pt.y - kpoints[xj].pt.y, 2) <
          200)
      {
        if (kpoints[xj].response > kpoints[xi].response)
          kpoints[xi] = kpoints[xj];

        kpoints[xj].size = -1;
      }
    }
  }
  kpoints.erase(std::remove_if(kpoints.begin(), kpoints.end(),
                               [](const cv::KeyPoint &kpt) { return kpt.size == -1; }),
                kpoints.end());
}

void assignClass(const cv::Mat &im, std::vector<cv::KeyPoint> &kpoints, float sizeNorm, int wsize)
{
  if (im.type() != CV_8UC1)
    throw std::runtime_error("assignClass Input image must be 8UC1");
  int wsizeFull = wsize * 2 + 1;

  cv::Mat labels = cv::Mat::zeros(wsize * 2 + 1, wsize * 2 + 1, CV_8UC1);
  cv::Mat thresIm = cv::Mat(wsize * 2 + 1, wsize * 2 + 1, CV_8UC1);

  for (auto &kp : kpoints)
  {
    float x = kp.pt.x;
    float y = kp.pt.y;

    // Convert point range from norm (-size/2, size/2) to (0,imageSize)
    if (sizeNorm > 0)
    {
      //                x = im.cols * (x/_fractalMarker.getFractalSize() + 0.5f);
      //                y = im.rows * (-y/_fractalMarker.getFractalSize() + 0.5f);
      x = im.cols * (x / sizeNorm + 0.5f);
      y = im.rows * (-y / sizeNorm + 0.5f);
    }

    x = int(x + 0.5f);
    y = int(y + 0.5f);

    cv::Rect r = cv::Rect(x - wsize, y - wsize, wsize * 2 + 1, wsize * 2 + 1);
    // Check boundaries
    if (r.x < 0 || r.x + r.width > im.cols || r.y < 0 || r.y + r.height > im.rows)
      continue;

    int endX = r.x + r.width;
    int endY = r.y + r.height;
    uchar minV = 255, maxV = 0;
    for (int y = r.y; y < endY; y++)
    {
      const uchar *ptr = im.ptr<uchar>(y);
      for (int x = r.x; x < endX; x++)
      {
        if (minV > ptr[x])
          minV = ptr[x];
        if (maxV < ptr[x])
          maxV = ptr[x];
      }
    }

    if ((maxV - minV) < 25)
    {
      kp.class_id = 0;
      continue;
    }

    double thres = (maxV + minV) / 2.0;

    unsigned int nZ = 0;
    // count non zero considering the threshold
    for (int y = 0; y < wsizeFull; y++)
    {
      const uchar *ptr = im.ptr<uchar>(r.y + y) + r.x;
      uchar *thresPtr = thresIm.ptr<uchar>(y);
      for (int x = 0; x < wsizeFull; x++)
      {
        if (ptr[x] > thres)
        {
          nZ++;
          thresPtr[x] = 255;
        }
        else
          thresPtr[x] = 0;
      }
    }
    // set all to zero labels.setTo(cv::Scalar::all(0));
    for (int y = 0; y < thresIm.rows; y++)
    {
      uchar *labelsPtr = labels.ptr<uchar>(y);
      for (int x = 0; x < thresIm.cols; x++)
        labelsPtr[x] = 0;
    }

    uchar newLab = 1;
    std::map<uchar, uchar> unions;
    for (int y = 0; y < thresIm.rows; y++)
    {
      uchar *thresPtr = thresIm.ptr<uchar>(y);
      uchar *labelsPtr = labels.ptr<uchar>(y);
      for (int x = 0; x < thresIm.cols; x++)
      {
        uchar reg = thresPtr[x];
        uchar lleft_px = 0;
        uchar ltop_px = 0;

        if (x - 1 > -1)
        {
          if (reg == thresPtr[x - 1])
            lleft_px = labelsPtr[x - 1];
        }

        if (y - 1 > -1)
        {
          if (reg == thresIm.ptr<uchar>(y - 1)[x])  // thresIm.at<uchar>(y-1, x)
            ltop_px = labels.at<uchar>(y - 1, x);
        }

        if (lleft_px == 0 && ltop_px == 0)
          labelsPtr[x] = newLab++;

        else if (lleft_px != 0 && ltop_px != 0)
        {
          if (lleft_px < ltop_px)
          {
            labelsPtr[x] = lleft_px;
            unions[ltop_px] = lleft_px;
          }
          else if (lleft_px > ltop_px)
          {
            labelsPtr[x] = ltop_px;
            unions[lleft_px] = ltop_px;
          }
          else
          {  // IGuales
            labelsPtr[x] = ltop_px;
          }
        }
        else
        {
          if (lleft_px != 0)
            labelsPtr[x] = lleft_px;
          else
            labelsPtr[x] = ltop_px;
        }
      }
    }

    int nc = newLab - 1 - unions.size();
    if (nc == 2)
    {
      if (nZ > thresIm.total() - nZ)
        kp.class_id = 0;
      else
        kp.class_id = 1;
    }
    else if (nc > 2)
    {
      kp.class_id = 2;
    }
  }
}

FractalPoseTracker::FractalPoseTracker()
{
}

void FractalPoseTracker::setParams(const CameraParameters &cam_params,
                                   const FractalMarkerSet &msconf, const float markerSize)
{
  _cam_params = cam_params;
  _fractalMarker = msconf;

  if (!cam_params.isValid())
    throw cv::Exception(9001, "Invalid camera parameters",
                        "FractalPoseTracker::setParams", __FILE__, __LINE__);
  if (_fractalMarker.mInfoType == aruco::FractalMarkerSet::NONE)
    throw cv::Exception(9001, "Invalid FractalMarker", "FractalPoseTracker::setParams",
                        __FILE__, __LINE__);
  if ((_fractalMarker.mInfoType == aruco::FractalMarkerSet::PIX ||
       _fractalMarker.mInfoType == aruco::FractalMarkerSet::NORM) &&
      markerSize <= 0)
    throw cv::Exception(9001, "You should indicate the markersize since the Fractal Marker is in pixels or normalized",
                        "FractalPoseTracker::setParams", __FILE__, __LINE__);
  if (_fractalMarker.mInfoType == aruco::FractalMarkerSet::PIX ||
      _fractalMarker.mInfoType == aruco::FractalMarkerSet::NORM)
    _fractalMarker = _fractalMarker.convertToMeters(markerSize);

  for (auto id_innerMarker : _fractalMarker.fractalMarkerCollection)
  {
    int markerId = id_innerMarker.first;
    FractalMarker innerMarker = id_innerMarker.second;

    // Inner corners
    _id_innerp3d[markerId] = innerMarker.getInnerCorners();
    for (auto pt : _id_innerp3d[markerId])
    {
      _innerp3d.push_back(pt);
      _innerkpoints.push_back(cv::KeyPoint(cv::Point2f(pt.x, pt.y), 20, -1, 0, markerId));
    }

    // radius search by marker
    float radiusF = 0.25;
    float ratio = innerMarker.getMarkerSize() / _fractalMarker.getFractalSize();
    float NBits = float(sqrt(_fractalMarker.nBits()) + 2) * ratio;
    _id_radius[markerId] =
        ((2 * NBits) / ((sqrt(innerMarker.nBits()) + 2) * (sqrt(_fractalMarker.nBits()) + 2))) *
        radiusF * _fractalMarker.getFractalSize() / 2;
  }

  // Get synthetic image (pixsize 6 in order to get inner corners classification)
  cv::Mat imageGray = _fractalMarker.getFractalMarkerImage(6) * 255;
  assignClass(imageGray, _innerkpoints, _fractalMarker.getFractalSize());
  _kdtree.build(_innerkpoints);

  //        #define _fractal_debug_classification
#ifdef _fractal_debug_classification
  drawKeyPoints(imageGray, _innerkpoints, false, true);
#endif
}

bool FractalPoseTracker::fractalRefinement(const cv::Ptr<MarkerDetector> markerDetector,
                                           int markerWarpPix)
{
  // ScopedTimerEvents Timer("fractal-refinement");

  std::vector<cv::Mat> imagePyramid = markerDetector->getImagePyramid();
  std::vector<cv::Point3f> _ref_inner3d;
  std::vector<cv::Point2f> _ref_inner2d;

  cv::Mat _p_rvec;
  _rvec.copyTo(_p_rvec);
  cv::Mat _p_tvec;
  _tvec.copyTo(_p_tvec);

  cv::Mat rot;
  cv::Rodrigues(_rvec, rot);

  for (auto id_marker : _fractalMarker.fractalMarkerCollection)
  {
    // Check z value for 4 external corners
    std::vector<cv::Point3f> marker3d;
    for (auto pt : id_marker.second.points)
    {
      cv::Mat_<double> src(3, 1, rot.type());
      src(0, 0) = pt.x;
      src(1, 0) = pt.y;
      src(2, 0) = pt.z;

      cv::Mat cam_image_point = rot * src + _tvec;
      cam_image_point = cam_image_point / cv::norm(cam_image_point);

      if (cam_image_point.at<double>(2, 0) > 0.85)
        marker3d.push_back(pt);
      else
        break;
    }

    std::vector<cv::Point3f> _inners3d;
    std::vector<cv::Point2f> _inners2d;
    float area = 0;
    if (marker3d.size() < 4)
    {
      if (_id_area.find(id_marker.first) != _id_area.end())
        area = _id_area[id_marker.first];
      else
        return false;

      for (auto pt : _id_innerp3d[id_marker.first])
      {
        cv::Mat_<double> src(3, 1, _rvec.type());
        src(0, 0) = pt.x;
        src(1, 0) = pt.y;
        src(2, 0) = pt.z;

        cv::Mat cam_image_point = rot * src + _tvec;
        cam_image_point = cam_image_point / cv::norm(cam_image_point);

        if (cam_image_point.at<double>(2, 0) > 0.85)
          _inners3d.push_back(pt);
      }

      if (_inners3d.size() == 0)
        return false;

      cv::projectPoints(_inners3d, _rvec, _tvec, _cam_params.CameraMatrix,
                        _cam_params.Distorsion, _inners2d);
    }
    else
    {
      std::vector<cv::Point2f> marker2d;
      cv::projectPoints(marker3d, _rvec, _tvec, _cam_params.CameraMatrix,
                        _cam_params.Distorsion, marker2d);

      cv::Point2f v01 = marker2d[1] - marker2d[0];
      cv::Point2f v03 = marker2d[3] - marker2d[0];
      float area1 = fabs(v01.x * v03.y - v01.y * v03.x);
      cv::Point2f v21 = marker2d[1] - marker2d[2];
      cv::Point2f v23 = marker2d[3] - marker2d[2];
      float area2 = fabs(v21.x * v23.y - v21.y * v23.x);

      area = (area2 + area1) / 2.f;
      _id_area[id_marker.first] = area;

      _inners3d = _id_innerp3d[id_marker.first];
      cv::projectPoints(_inners3d, _rvec, _tvec, _cam_params.CameraMatrix,
                        _cam_params.Distorsion, _inners2d);
    }

    size_t imgPyrIdx = 0;
    auto markerWarpSize = (sqrt(id_marker.second.nBits()) + 2) * markerWarpPix;
    float desiredarea = std::pow(static_cast<float>(markerWarpSize), 2.f);
    for (size_t p = 1; p < imagePyramid.size(); p++)
    {
      if (area / pow(4, p) >= desiredarea)
        imgPyrIdx = p;
      else
        break;
    }

    float ratio = float(imagePyramid[imgPyrIdx].cols) / float(imagePyramid[0].cols);

    // std::cout << "REFINE["<< id_marker.first <<"], imgPyrId:"<<imgPyrIdx << ", ratio:"<< ratio<<std::endl;

    std::vector<double> _inners2d_error;
    if (ratio == 1 && area >= desiredarea)
    {
      int halfwsize =
          4 * float(imagePyramid[imgPyrIdx].cols) / float(imagePyramid[imgPyrIdx].cols) + 0.5;

      std::vector<cv::Point2f> _inners2d_copy;
      for (auto pt : _inners2d)
      {
        _inners2d_copy.push_back(pt);
      }
      cornerSubPix(
          imagePyramid[imgPyrIdx], _inners2d, cv::Size(halfwsize, halfwsize), cv::Size(-1, -1),
          cv::TermCriteria(cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS, 12, 0.005));
      int idx = 0;
      for (auto pt : _inners2d)
      {
        _inners2d_error.push_back(sqrt(pow(_inners2d_copy[idx].x - pt.x, 2) +
                                       pow(_inners2d_copy[idx].y - pt.y, 2)));
        idx++;
      }
    }
    else if (ratio != 1 && area >= desiredarea)
    {
      std::vector<cv::Point2f> _inners2d_copy;
      for (auto &pt : _inners2d)
      {
        _inners2d_copy.push_back(pt);
        pt *= ratio;
      }

      std::vector<std::vector<cv::Point2f>> vpnts;
      vpnts.push_back(_inners2d);
      markerDetector->cornerUpsample(vpnts, imagePyramid[imgPyrIdx].size());

      int idx = 0;
      for (auto pt : vpnts[0])
      {
        _inners2d_error.push_back(sqrt(pow(_inners2d_copy[idx].x - pt.x, 2) +
                                       pow(_inners2d_copy[idx].y - pt.y, 2)));
        _inners2d[idx++] = pt;
      }
    }

    // Elimina puntos que no son esquinas
    if (area >= desiredarea)
    {
      // We discard outliers. Points above limit Q3+3*(Q3-1)
      std::vector<double> _inners2d_error_copy;
      for (auto err : _inners2d_error)
        _inners2d_error_copy.push_back(err);
      sort(_inners2d_error_copy.begin(), _inners2d_error_copy.end());
      int q1 = (_inners2d_error_copy.size() + 1) / 4;
      int q3 = 3 * (_inners2d_error_copy.size() + 1) / 4;

      double limit = _inners2d_error_copy[q3] +
                     3 * (_inners2d_error_copy[q3] - _inners2d_error_copy[q1]);

      int wsize = 10;
      for (int idx = 0; idx < _inners2d.size(); idx++)
      {
        if (_inners2d_error[idx] > limit)
          continue;

        float x = int(_inners2d[idx].x + 0.5f);
        float y = int(_inners2d[idx].y + 0.5f);

        cv::Rect r = cv::Rect(x - wsize, y - wsize, wsize * 2 + 1, wsize * 2 + 1);
        // Check boundaries
        if (r.x < 0 || r.x + r.width > imagePyramid[0].cols || r.y < 0 ||
            r.y + r.height > imagePyramid[0].rows)
          continue;

        int endX = r.x + r.width;
        int endY = r.y + r.height;
        uchar minV = 255, maxV = 0;
        for (int y = r.y; y < endY; y++)
        {
          const uchar *ptr = imagePyramid[0].ptr<uchar>(y);
          for (int x = r.x; x < endX; x++)
          {
            if (minV > ptr[x])
              minV = ptr[x];
            if (maxV < ptr[x])
              maxV = ptr[x];
          }
        }

        if ((maxV - minV) < 25)
          continue;

        _ref_inner3d.push_back(_inners3d[idx]);
        _ref_inner2d.push_back(_inners2d[idx]);
      }
    }
    // Timer.add("refine-"+std::to_string(id_marker.first));
  }

  // Solve
  if (_ref_inner3d.size() > 4)
  {
    aruco::solvePnP(_ref_inner3d, _ref_inner2d, _cam_params.CameraMatrix,
                    _cam_params.Distorsion, _rvec, _tvec);

//#define _fractal_debug_inners
#ifdef _fractal_debug_inners
    cv::Mat InImageCopy;
    cv::cvtColor(imagePyramid[0], InImageCopy, CV_GRAY2RGB);

    // Show first position (estimation with detected markers)
    std::vector<cv::Point2f> preinnersPrj;
    for (auto id_marker : _fractalMarker.fractalMarkerCollection)
    {
      cv::projectPoints(_id_innerp3d[id_marker.first], _p_rvec, _p_tvec,
                        _cam_params.CameraMatrix, _cam_params.Distorsion, preinnersPrj);
      for (auto pt : preinnersPrj)
        cv::circle(InImageCopy, pt, 5, cv::Scalar(0, 0, 255), CV_FILLED);
    }

    // Show first position with refinement
    for (auto p : _ref_inner2d)
      cv::circle(InImageCopy, p, 5, cv::Scalar(255, 0, 0), CV_FILLED);

    _rvec.convertTo(_rvec, CV_32F);
    _tvec.convertTo(_tvec, CV_32F);
    cv::Rodrigues(_rvec, rot);

    std::vector<cv::Point3f> _inners;
    for (auto pt : _inner_corners_3d)
    {
      cv::Mat_<float> src(3, 1, rot.type());
      src(0, 0) = pt.x;
      src(1, 0) = pt.y;
      src(2, 0) = pt.z;

      cv::Mat cam_image_point = rot * src + _tvec;
      cam_image_point = cam_image_point / cv::norm(cam_image_point);

      if (cam_image_point.at<float>(2, 0) > 0.85f)
        _inners.push_back(pt);
    }
    std::vector<cv::Point2f> _inners_prj;
    cv::projectPoints(_inners, _rvec, _tvec, _cam_params.CameraMatrix,
                      _cam_params.Distorsion, _inners_prj);

    // Show new projection using all inner points
    for (auto pt : _inners_prj)
      cv::circle(InImageCopy, pt, 5, cv::Scalar(0, 255, 0), CV_FILLED);

    cv::namedWindow("AA", cv::WINDOW_NORMAL);
    imshow("AA", InImageCopy);
    cv::waitKey();
#endif
    // Timer.add("solve");

    return true;
  }
  else
    return false;
}

bool FractalPoseTracker::fractalInnerPose(const cv::Ptr<aruco::MarkerDetector> markerDetector,
                                          const std::vector<aruco::Marker> &vmarkers,
                                          bool refinement)
{
  if (vmarkers.size() > 0)
  {
    //            ScopedTimerEvents Timer("pnp");
    //            std::cout << "[Case 1]"<< std::endl;
    std::vector<cv::Point2f> p2d;
    std::vector<cv::Point3f> p3d;
    for (auto marker : vmarkers)
    {
      if (_fractalMarker.fractalMarkerCollection.find(marker.id) !=
          _fractalMarker.fractalMarkerCollection.end())
      {
        for (auto p : marker)
          p2d.push_back(p);

        for (auto p : _fractalMarker.fractalMarkerCollection[marker.id].points)
          p3d.push_back(p);
      }
    }

    // Initial pose estimation
    aruco::solvePnP(p3d, p2d, _cam_params.CameraMatrix, _cam_params.Distorsion, _rvec, _tvec);

    //            Timer.add("solve");

    // REFINE
    if (refinement)
    {
      fractalRefinement(markerDetector);
      //                Timer.add("refine-solution");
    }

    return true;
  }
  else
  {
    if (!_rvec.empty())
    {
      //                std::cout << "[Case 2]"<< std::endl;
      //                ScopedTimerEvents Timer("ransac");
      //                Timer.add("detect");

      std::vector<cv::Point2f> innerPoints2d;
      std::vector<cv::Point3f> innerPoints3d;

      float radius = 0;
      cv::Mat rot;
      cv::Rodrigues(_rvec, rot);

      // Getting the keypoints search radius
      for (auto id_marker : _fractalMarker.fractalMarkerCollection)
      {
        std::vector<cv::Point2f> marker2d;
        std::vector<cv::Point3f> marker3d;
        for (auto pt : _fractalMarker.fractalMarkerCollection[id_marker.first].points)
        {
          cv::Mat_<double> src(3, 1, rot.type());
          src(0, 0) = pt.x;
          src(1, 0) = pt.y;
          src(2, 0) = pt.z;
          cv::Mat cam_image_point = rot * src + _tvec;
          cam_image_point = cam_image_point / cv::norm(cam_image_point);

          if (cam_image_point.at<double>(2, 0) > 0.85)
            marker3d.push_back(pt);
          else
            break;
        }

        if (marker3d.size() == 4)
        {
          cv::projectPoints(marker3d, _rvec, _tvec, _cam_params.CameraMatrix,
                            _cam_params.Distorsion, marker2d);

          // Find marker area
          cv::Point2f v01 = marker2d[1] - marker2d[0];
          cv::Point2f v03 = marker2d[3] - marker2d[0];
          float area1 = fabs(v01.x * v03.y - v01.y * v03.x);
          cv::Point2f v21 = marker2d[1] - marker2d[2];
          cv::Point2f v23 = marker2d[3] - marker2d[2];
          float area2 = fabs(v21.x * v23.y - v21.y * v23.x);
          double area = (area2 + area1) / 2.f;

          auto markerWarpSize =
              (sqrt(_fractalMarker.fractalMarkerCollection[id_marker.first].nBits()) + 2) * 10;
          float desiredarea = std::pow(static_cast<float>(markerWarpSize), 2.f);
          if (area >= desiredarea)
          {
            for (auto pt : _id_innerp3d[id_marker.first])
              innerPoints3d.push_back(pt);
          }

          if (radius == 0.f)
            radius = sqrt(area) / (sqrt(id_marker.second.nBits()) + 2.f);
        }
        else
        {
          for (auto pt : _id_innerp3d[id_marker.first])
            innerPoints3d.push_back(pt);
        }
      }

      if (radius == 0)
        radius = _preRadius;
      _preRadius = radius;

      if (innerPoints3d.size() > 0 && radius > 0)
      {
        cv::projectPoints(innerPoints3d, _rvec, _tvec, _cam_params.CameraMatrix,
                          _cam_params.Distorsion, innerPoints2d);

        cv::Mat region;
        cv::Point2f offset;
        float ratio;
        if (!ROI(markerDetector->getImagePyramid(), region, innerPoints2d, offset, ratio))
          return false;
        radius = radius * ratio;
        //                    Timer.add("roi");


        std::cout << "radius: " << radius << std::endl;

//#define _fractal_debug_region
#ifdef _fractal_debug_region
        cv::Mat out;
        cv::cvtColor(region, out, CV_GRAY2RGB);
        for (uint32_t i = 0; i < innerPoints2d.size(); i++)
          circle(out, innerPoints2d[i], radius, cv::Scalar(0, 0, 255), 2);
        cv::imshow("REGION ", out);
        cv::waitKey();
#endif

        // FAST
        std::vector<cv::KeyPoint> kpoints;
        cv::Ptr<cv::FastFeatureDetector> fd = cv::FastFeatureDetector::create();
        fd->detect(region, kpoints);
        //                    Timer.add("fast");


        if (kpoints.size() > 0)
        {
          // Filter kpoints (low response) and removing duplicated.
          kfilter(kpoints);
          //                        Timer.add("fast-filter");
          //                        std::cout << "fast-filter" << std::endl;

          // Assign class to keypoints
          assignClass(region, kpoints);
//                        Timer.add("fast-class");

//#define _fractal_debug_classification
#ifdef _fractal_debug_classification
          drawKeyPoints(region, kpoints);
          cv::waitKey();
#endif
          // Get keypoints with better response in a radius
          picoflann::KdTreeIndex<2, PicoFlann_KeyPointAdapter> kdtreeImg;
          kdtreeImg.build(kpoints);
          std::vector<std::pair<uint, std::vector<uint>>> inner_candidates;
          for (uint idx = 0; idx < innerPoints2d.size(); idx++)
          {
            std::vector<std::pair<uint32_t, double>> res =
                kdtreeImg.radiusSearch(kpoints, innerPoints2d[idx], radius);

            std::vector<uint> candidates;
            for (auto r : res)
            {
              if (kpoints[r.first].class_id == _innerkpoints[idx].class_id)
                candidates.push_back(r.first);
            }
            if (candidates.size() > 0)
              inner_candidates.push_back(make_pair(idx, candidates));
          }

          //                        Timer.add("candidates");
          cv::Mat bestModel =
              fractal_solve_ransac(innerPoints2d.size(), inner_candidates, kpoints);
          //                        Timer.add("find-solution");

          if (!bestModel.empty())
          {
            std::vector<cv::Point3f> p3d;
            std::vector<cv::Point2f> p2d;

            std::vector<cv::Point2f> pnts, pntsDst;
            cv::KeyPoint::convert(kpoints, pnts);
            perspectiveTransform(pnts, pntsDst, bestModel);
            for (uint32_t id = 0; id < pntsDst.size(); id++)
            {
              std::vector<std::pair<uint32_t, double>> res =
                  _kdtree.radiusSearch(_innerkpoints, pntsDst[id], _id_radius[0]);

              int i = 0;
              for (auto r : res)
              {
                uint32_t innerId = r.first;
                double dist = sqrt(r.second);

                uint32_t fmarkerId = _innerkpoints[innerId].octave;
                if (dist > _id_radius[fmarkerId])
                  res.erase(res.begin() + i);
                else
                  i++;
              }
              if (res.size() > 0)
              {
                p3d.push_back(_innerp3d[res[0].first]);
                p2d.push_back(
                    cv::Point2f(kpoints[id].pt.x + offset.x, kpoints[id].pt.y + offset.y) / ratio);
              }
            }

            if (p3d.size() >= 4)
            {
              //                                std::cout << "solves" << std::endl;
              aruco::solvePnP(p3d, p2d, _cam_params.CameraMatrix, _cam_params.Distorsion,
                              _rvec, _tvec);
              //                                Timer.add("solves");

              if (refinement)
              {
                //                                    std::cout << "refine-solution" << std::endl;
                fractalRefinement(markerDetector);
                //                                    Timer.add("refine-solution");
              }

              return true;
            }
          }
        }
      }
    }
    _rvec = cv::Mat();
    _tvec = cv::Mat();

    return false;
  }
}

bool FractalPoseTracker::ROI(const std::vector<cv::Mat> imagePyramid, cv::Mat &img,
                             std::vector<cv::Point2f> &innerPoints2d, cv::Point2f &offset,
                             float &ratio)
{
  cv::Mat rot;
  cv::Rodrigues(_rvec, rot);

  // Biggest marker projection
  std::vector<cv::Point2f> biggest_p2d;
  std::vector<cv::Point3f> biggest_p3d;
  for (int idx = 0;
       idx < _fractalMarker.fractalMarkerCollection.size() & biggest_p3d.size() < 4; idx++)
  {
    biggest_p3d.empty();
    for (auto pt : _fractalMarker.fractalMarkerCollection[idx].points)
    {
      cv::Mat_<double> src(3, 1, rot.type());
      src(0, 0) = pt.x;
      src(1, 0) = pt.y;
      src(2, 0) = pt.z;
      cv::Mat cam_image_point = rot * src + _tvec;
      cam_image_point = cam_image_point / cv::norm(cam_image_point);

      if (cam_image_point.at<double>(2, 0) > 0.85)
        biggest_p3d.push_back(pt);
      else
        break;
    }
  }

  if (!biggest_p3d.empty())
  {
    cv::projectPoints(biggest_p3d, _rvec, _tvec, _cam_params.CameraMatrix,
                      _cam_params.Distorsion, biggest_p2d);

    // Smallest marker projection
    std::vector<cv::Point2f> smallest_p2d;
    std::vector<cv::Point3f> smallest_p3d;
    auto marker_smallest = (--_fractalMarker.fractalMarkerCollection.end())->second;
    for (auto pt : marker_smallest.points)
      smallest_p3d.push_back(pt);
    cv::projectPoints(smallest_p3d, _rvec, _tvec, _cam_params.CameraMatrix,
                      _cam_params.Distorsion, smallest_p2d);

    // Smallest marker area
    cv::Point2f v01 = smallest_p2d[1] - smallest_p2d[0];
    cv::Point2f v03 = smallest_p2d[3] - smallest_p2d[0];
    float area1 = fabs(v01.x * v03.y - v01.y * v03.x);
    cv::Point2f v21 = smallest_p2d[1] - smallest_p2d[2];
    cv::Point2f v23 = smallest_p2d[3] - smallest_p2d[2];
    float area2 = fabs(v21.x * v23.y - v21.y * v23.x);
    double area = (area2 + area1) / 2.f;


    // Compute boundaries region
    float border = sqrt(area) / sqrt(marker_smallest.nBits()) + 2;
    int minX = imagePyramid[0].cols, minY = imagePyramid[0].rows, maxX = 0, maxY = 0;
    for (auto p : biggest_p2d)
    {
      if (p.x < minX)
        minX = p.x - border;
      if (p.x > maxX)
        maxX = p.x + border;
      if (p.y < minY)
        minY = p.y - border;
      if (p.y > maxY)
        maxY = p.y + border;
    }
    if (minX < 0)
      minX = 0;
    if (minY < 0)
      minY = 0;
    if (maxX > imagePyramid[0].cols)
      maxX = imagePyramid[0].cols;
    if (maxY > imagePyramid[0].rows)
      maxY = imagePyramid[0].rows;


    // Select imagePyramid
    size_t imgPyrIdx = 0;
    auto markerWarpSize = (sqrt(marker_smallest.nBits()) + 2) * 10;
    float desiredarea = std::pow(static_cast<float>(markerWarpSize), 2.f);
    for (size_t p = 1; p < imagePyramid.size(); p++)
    {
      if (area / pow(4, p) >= desiredarea)
        imgPyrIdx = p;
      else
        break;
    }

    ratio = float(imagePyramid[imgPyrIdx].cols) / float(imagePyramid[0].cols);
    offset = cv::Point2i(minX, minY) * ratio;
    cv::Rect region = cv::Rect(cv::Point2i(minX, minY) * ratio, cv::Point2i(maxX, maxY) * ratio);
    img = imagePyramid[imgPyrIdx](region);

    for (auto &pt : innerPoints2d)
    {
      pt.x = pt.x * ratio - region.x;
      pt.y = pt.y * ratio - region.y;
    }
    return true;
  }
  else
    return false;
}

cv::Mat FractalPoseTracker::fractal_solve_ransac(
    int ninners, std::vector<std::pair<uint, std::vector<uint>>> inner_kpnt,
    std::vector<cv::KeyPoint> kpnts, uint32_t maxIter, float _minInliers, float _thresInliers)
{
  std::vector<cv::Point2f> pnts;
  cv::KeyPoint::convert(kpnts, pnts);

  // Number randomly values selected
  uint32_t numInliers = 4;
  // Number of inliers to consider it good model, stop iterating!!!
  uint32_t thresInliers = uint32_t(ninners * _thresInliers);
  // Enough number of inliers to consider the model as valid
  uint32_t minInliers = uint32_t(ninners * _minInliers);

  // Number of inliers
  uint32_t bestInliers = 0;
  // Best model
  cv::Mat bestH = cv::Mat();

  //        struct timespec ts;
  //        clock_gettime(CLOCK_MONOTONIC, &ts);
  //        srand((time_t)ts.tv_nsec);

  uint32_t count = 0;
  for (auto ik : inner_kpnt)
    if (ik.second.size() > 0)
      count++;
  if (count < minInliers)
    return cv::Mat();

  uint32_t iter = 0;
  do
  {
    // New stop condition to avoid infinite loop, when it tries to find the initial inliers
    // For instance, for innerkpts group: 23{2} 18{3}, 14{3}, 3{7,6,5}, 2{3},
    // when the initial group of inliers is selected:
    // 23{2},18{3},3{6} there's no way to find the fourth ...
    uint32_t iter2 = 0;
    std::vector<std::pair<uint, uint>> inliers;
    while (inliers.size() < numInliers && iter2++ < maxIter)
    {
      uint idx = rand() % inner_kpnt.size();
      uint id_dst = inner_kpnt[idx].first;

      uint idxc = rand() % inner_kpnt[idx].second.size();
      uint id_src = inner_kpnt[idx].second[idxc];

      // avoid duplicate observations
      bool exist = false;
      for (auto in : inliers)
      {
        if ((in.first != id_dst) && (in.second != id_src))
          continue;
        else
        {
          exist = true;
          break;
        }
      }

      // if it is a new observation, add it!
      if (!exist)
        inliers.push_back(std::make_pair(id_dst, id_src));
    }

    if (iter2 >= maxIter)
      return cv::Mat();

    std::vector<cv::Point2f> srcInliers;
    std::vector<cv::Point2f> dstInliers;
    for (auto in : inliers)
    {
      dstInliers.push_back(_innerkpoints[in.first].pt);
      srcInliers.push_back(kpnts[in.second].pt);
    }
    // Fit model with initial random inliers
    cv::Mat H = findHomography(srcInliers, dstInliers);

    if (!H.empty())
    {
      std::vector<std::pair<uint, uint>> newInliers;
      std::vector<std::pair<uint, uint>> newInliers2;

      std::vector<cv::Point2f> dstNewInliers;
      std::vector<cv::Point2f> srcNewInliers;
      std::vector<cv::Point2f> pntsTranf;
      perspectiveTransform(pnts, pntsTranf, H);

      for (uint idP = 0; idP < pntsTranf.size(); idP++)
      {
        std::vector<std::pair<uint32_t, double>> res =
            _kdtree.radiusSearch(_innerkpoints, pntsTranf[idP], _id_radius[0]);

        int i = 0;
        for (auto r : res)
        {
          uint32_t innerId = r.first;
          double dist = sqrt(r.second);

          uint32_t fmarkerId = _innerkpoints[innerId].octave;
          if (dist > _id_radius[fmarkerId])
            res.erase(res.begin() + i);
          else
            i++;
        }

        if (res.size() > 0)
        {
          // avoid duplicate observations
          bool exist = false;
          for (auto in : newInliers)
          {
            if (in.first != res[0].first)
              continue;
            else
            {
              exist = true;
              break;
            }
          }
          // if it is a new observation and these belong to the same class, add it!
          if (!exist)
          {
            if (_innerkpoints[res[0].first].class_id == kpnts[idP].class_id)
              newInliers.push_back(std::make_pair(res[0].first, idP));
          }
        }
      }
      if (newInliers.size() > bestInliers)
      {
        bestInliers = newInliers.size();
        bestH = H;
      }
    }
    iter++;
  } while (iter < maxIter && bestInliers < thresInliers);

  //        std::cout << "[RANSAC] minInliers: "<< minInliers ;
  //        std::cout << " ,bestInliers: "<< bestInliers;
  //        std::cout << " ,iterations: "<< iter << std::endl;

  if (bestInliers < minInliers)
    bestH = cv::Mat();

  return bestH;
}

void FractalPoseTracker::drawKeyPoints(const cv::Mat image, std::vector<cv::KeyPoint> kpoints,
                                       bool text, bool transf)
{
  if (transf)
  {
    // Convert point range from norm (-size/2, size/2) to (0,imageSize)
    for (auto &k : kpoints)
    {
      k.pt.x = image.cols * (k.pt.x / _fractalMarker.getFractalSize() + 0.5f);
      k.pt.y = image.rows * (-k.pt.y / _fractalMarker.getFractalSize() + 0.5f);
    }
  }

  cv::Mat out;
  cv::cvtColor(image, out, CV_GRAY2BGR);
  // drawKeypoints(image, kpoints, out, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

  cv::Scalar color;
  for (auto kp : kpoints)
  {
    if (kp.class_id == -1)
      color = cv::Scalar(255, 0, 255);
    if (kp.class_id == 0)
      color = cv::Scalar(0, 0, 255);
    if (kp.class_id == 1)
      color = cv::Scalar(0, 255, 0);
    if (kp.class_id == 2)
      color = cv::Scalar(255, 0, 0);
    circle(out, kp.pt, 2, color, -1);
  }

  if (text)
  {
    int nkm = 0;
    for (auto kp : kpoints)
    {
      putText(out, std::to_string(nkm++), kp.pt, CV_FONT_HERSHEY_COMPLEX, 1,
              cv::Scalar(0, 0, 255), 2, 8);
    }
  }

  //#ifdef _DEBUG
  //        imshow("KPoints", out);
  //        cv::waitKey();
  //#endif
}
}  // namespace aruco
