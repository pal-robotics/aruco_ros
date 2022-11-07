/**
 Copyright 2017 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas
 OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 */

#include "posetracker.h"
#include "ippe.h"
#include <set>
#include "levmarq.h"
#include <opencv2/calib3d.hpp>

namespace aruco
{
namespace aruco_private
{
cv::Mat impl__aruco_getRTMatrix(const cv::Mat& _rvec, const cv::Mat& _tvec)
{
  assert(_rvec.type() == CV_32F && _rvec.total() == 3);
  assert(_tvec.type() == CV_32F && _tvec.total() == 3);

  cv::Mat Matrix(4, 4, CV_32F);
  float* rt_44 = Matrix.ptr<float>(0);

  // makes a fast conversion to the 4x4 array passed
  float rx = _rvec.ptr<float>(0)[0];
  float ry = _rvec.ptr<float>(0)[1];
  float rz = _rvec.ptr<float>(0)[2];
  float tx = _tvec.ptr<float>(0)[0];
  float ty = _tvec.ptr<float>(0)[1];
  float tz = _tvec.ptr<float>(0)[2];
  float nsqa = rx * rx + ry * ry + rz * rz;
  float a = std::sqrt(nsqa);
  float i_a = a ? 1. / a : 0;
  float rnx = rx * i_a;
  float rny = ry * i_a;
  float rnz = rz * i_a;
  float cos_a = cos(a);
  float sin_a = sin(a);
  float _1_cos_a = 1. - cos_a;
  rt_44[0] = cos_a + rnx * rnx * _1_cos_a;
  rt_44[1] = rnx * rny * _1_cos_a - rnz * sin_a;
  rt_44[2] = rny * sin_a + rnx * rnz * _1_cos_a;
  rt_44[3] = tx;
  rt_44[4] = rnz * sin_a + rnx * rny * _1_cos_a;
  rt_44[5] = cos_a + rny * rny * _1_cos_a;
  rt_44[6] = -rnx * sin_a + rny * rnz * _1_cos_a;
  rt_44[7] = ty;
  rt_44[8] = -rny * sin_a + rnx * rnz * _1_cos_a;
  rt_44[9] = rnx * sin_a + rny * rnz * _1_cos_a;
  rt_44[10] = cos_a + rnz * rnz * _1_cos_a;
  rt_44[11] = tz;
  rt_44[12] = rt_44[13] = rt_44[14] = 0;
  rt_44[15] = 1;
  return Matrix;
}

void impl__aruco_getRTfromMatrix44(const cv::Mat& M, cv::Mat& R, cv::Mat& T)
{
  assert(M.cols == M.rows && M.cols == 4);
  assert(M.type() == CV_32F || M.type() == CV_64F);

  // extract the rotation part
  cv::Mat r33 = cv::Mat(M, cv::Rect(0, 0, 3, 3));
  cv::SVD svd(r33);
  cv::Mat Rpure = svd.u * svd.vt;
  cv::Rodrigues(Rpure, R);
  T.create(1, 3, M.type());
  if (M.type() == CV_32F)
    for (int i = 0; i < 3; i++)
      T.ptr<float>(0)[i] = M.at<float>(i, 3);
  else
    for (int i = 0; i < 3; i++)
      T.ptr<double>(0)[i] = M.at<double>(i, 3);
}

double reprj_error(const std::vector<cv::Point3f>& objPoints,
                   const std::vector<cv::Point2f> points2d, const CameraParameters& imp,
                   const cv::Mat& rt44)
{
  std::vector<cv::Point2f> prepj;
  cv::Mat rv, tv;
  impl__aruco_getRTfromMatrix44(rt44, rv, tv);
  cv::projectPoints(objPoints, rv, tv, imp.CameraMatrix, imp.Distorsion, prepj);
  double sum = 0;
  int nvalid = 0;
  for (size_t i = 0; i < prepj.size(); i++)
  {
    if (!std::isnan(objPoints[i].x))
    {
      sum += cv::norm(points2d[i] - prepj[i]);
      nvalid++;
    }
  }
  return sum / double(nvalid);
}

/**
 *
 *
 */
float rigidBodyTransformation_Horn1987(const std::vector<cv::Point3f>& POrg,
                                       const std::vector<cv::Point3f>& PDst, cv::Mat& RT_4x4)
{
  struct Quaternion
  {
    Quaternion(float q0, float q1, float q2, float q3)
    {
      q[0] = q0;
      q[1] = q1;
      q[2] = q2;
      q[3] = q3;
    }
    cv::Mat getRotation() const
    {
      cv::Mat R(3, 3, CV_32F);
      R.at<float>(0, 0) = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
      R.at<float>(0, 1) = 2.f * (q[1] * q[2] - q[0] * q[3]);
      R.at<float>(0, 2) = 2.f * (q[1] * q[3] + q[0] * q[2]);

      R.at<float>(1, 0) = 2.f * (q[1] * q[2] + q[0] * q[3]);
      R.at<float>(1, 1) = q[0] * q[0] + q[2] * q[2] - q[1] * q[1] - q[3] * q[3];
      R.at<float>(1, 2) = 2.f * (q[2] * q[3] - q[0] * q[1]);

      R.at<float>(2, 0) = 2.f * (q[1] * q[3] - q[0] * q[2]);
      R.at<float>(2, 1) = 2.f * (q[2] * q[3] + q[0] * q[1]);
      R.at<float>(2, 2) = q[0] * q[0] + q[3] * q[3] - q[1] * q[1] - q[2] * q[2];
      return R;
    }
    float q[4];
  };

  assert(POrg.size() == PDst.size());

  cv::Mat _org(POrg.size(), 3, CV_32F, (float*)&POrg[0]);
  cv::Mat _dst(PDst.size(), 3, CV_32F, (float*)&PDst[0]);

  //  _org = _org.reshape(1);
  //  _dst = _dst.reshape(1);
  cv::Mat Mu_s = cv::Mat::zeros(1, 3, CV_32F);
  cv::Mat Mu_m = cv::Mat::zeros(1, 3, CV_32F);
  //  cout << _s << endl << _m << endl;

  // calculate means
  for (int i = 0; i < _org.rows; i++)
  {
    Mu_s += _org(cv::Range(i, i + 1), cv::Range(0, 3));
    Mu_m += _dst(cv::Range(i, i + 1), cv::Range(0, 3));
  }

  // now, divide
  for (int i = 0; i < 3; i++)
  {
    Mu_s.ptr<float>(0)[i] /= float(_org.rows);
    Mu_m.ptr<float>(0)[i] /= float(_dst.rows);
  }

  //  cout << "Mu_s = " << Mu_s << endl;
  //  cout << "Mu_m = " << Mu_m << endl;

  cv::Mat Mu_st = Mu_s.t() * Mu_m;
  //  cout << "Mu_st = " << Mu_st << endl;
  cv::Mat Var_sm = cv::Mat::zeros(3, 3, CV_32F);
  for (int i = 0; i < _org.rows; i++)
    Var_sm += (_org(cv::Range(i, i + 1), cv::Range(0, 3)).t() *
               _dst(cv::Range(i, i + 1), cv::Range(0, 3))) -
              Mu_st;
  //  cout << "Var_sm=" << Var_sm << endl;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      Var_sm.at<float>(i, j) /= float(_org.rows);
  //  cout << "Var_sm = " << Var_sm << endl;

  cv::Mat AA = Var_sm - Var_sm.t();
  //  cout << "AA = " << AA << endl;
  cv::Mat A(3, 1, CV_32F);
  A.at<float>(0, 0) = AA.at<float>(1, 2);
  A.at<float>(1, 0) = AA.at<float>(2, 0);
  A.at<float>(2, 0) = AA.at<float>(0, 1);
  //  cout << "A =" << A << endl;
  cv::Mat Q_Var_sm(4, 4, CV_32F);
  Q_Var_sm.at<float>(0, 0) = static_cast<float>(trace(Var_sm)[0]);
  for (int i = 1; i < 4; i++)
  {
    Q_Var_sm.at<float>(0, i) = A.ptr<float>(0)[i - 1];
    Q_Var_sm.at<float>(i, 0) = A.ptr<float>(0)[i - 1];
  }
  cv::Mat q33 = Var_sm + Var_sm.t() - (trace(Var_sm)[0] * cv::Mat::eye(3, 3, CV_32F));

  cv::Mat Q33 = Q_Var_sm(cv::Range(1, 4), cv::Range(1, 4));
  q33.copyTo(Q33);
  //  cout << "Q_Var_sm" << endl << Q_Var_sm << endl;
  cv::Mat eigenvalues, eigenvectors;
  eigen(Q_Var_sm, eigenvalues, eigenvectors);
  //  cout << "EEI = " << eigenvalues << endl;
  //  cout << "V = " << (eigenvectors.type() == CV_32F) << " " << eigenvectors << endl;

  Quaternion rot(eigenvectors.at<float>(0, 0), eigenvectors.at<float>(0, 1),
                 eigenvectors.at<float>(0, 2), eigenvectors.at<float>(0, 3));
  cv::Mat RR = rot.getRotation();
  //  cout << "RESULT = " << endl << RR << endl;
  cv::Mat T = Mu_m.t() - RR * Mu_s.t();
  //  cout << "T = " << T << endl;

  RT_4x4 = cv::Mat::eye(4, 4, CV_32F);
  cv::Mat r33 = RT_4x4(cv::Range(0, 3), cv::Range(0, 3));
  RR.copyTo(r33);
  for (int i = 0; i < 3; i++)
    RT_4x4.at<float>(i, 3) = T.ptr<float>(0)[i];
  //  cout << "RESS = " << RT << endl;

  // compute the average transform error
  float err = 0;
  float* matrix = RT_4x4.ptr<float>(0);
  for (size_t i = 0; i < POrg.size(); i++)
  {
    cv::Point3f org = POrg[i];
    cv::Point3f dest_est;
    dest_est.x = matrix[0] * org.x + matrix[1] * org.y + matrix[2] * org.z + matrix[3];
    dest_est.y = matrix[4] * org.x + matrix[5] * org.y + matrix[6] * org.z + matrix[7];
    dest_est.z = matrix[8] * org.x + matrix[9] * org.y + matrix[10] * org.z + matrix[11];
    cv::Point3f dest_real = PDst[i];
    err += static_cast<float>(cv::norm(dest_est - dest_real));
  }

  return err / float(POrg.size());
  ;
}

}  // namespace aruco_private

inline double hubber(double e, double _delta)
{
  double dsqr = _delta * _delta;
  if (e <= dsqr)
  {
    // inlier
    return e;
  }
  else
  {
    // outlier
    double sqrte = sqrt(e);            // absolute value of the error
    return 2 * sqrte * _delta - dsqr;  // rho(e)   = 2 * delta * e^(1/2) - delta^2
  }
}

inline double hubberMono(double e)
{
  if (e <= 5.991)
  {
    // inlier
    return e;
  }
  else
    // outlier
    return 4.895303872 * sqrt(e) - 5.991;  // rho(e)   = 2 * delta * e^(1/2) - delta^2
}

inline double getHubberMonoWeight(double SqErr, double Information)
{
  return sqrt(hubberMono(Information * SqErr) / SqErr);
}
template <typename T>
double __aruco_solve_pnp(const std::vector<cv::Point3f>& p3d,
                         const std::vector<cv::Point2f>& p2d, const cv::Mat& cam_matrix,
                         const cv::Mat& dist, cv::Mat& r_io, cv::Mat& t_io)
{
  assert(r_io.type() == CV_32F);
  assert(t_io.type() == CV_32F);
  assert(t_io.total() == r_io.total());
  assert(t_io.total() == 3);
  auto toSol = [](const cv::Mat& r, const cv::Mat& t)
  {
    typename LevMarq<T>::eVector sol(6);
    for (int i = 0; i < 3; i++)
    {
      sol(i) = r.ptr<float>(0)[i];
      sol(i + 3) = t.ptr<float>(0)[i];
    }
    return sol;
  };
  auto fromSol = [](const typename LevMarq<T>::eVector& sol, cv::Mat& r, cv::Mat& t)
  {
    r.create(1, 3, CV_32F);
    t.create(1, 3, CV_32F);
    for (int i = 0; i < 3; i++)
    {
      r.ptr<float>(0)[i] = sol(i);
      t.ptr<float>(0)[i] = sol(i + 3);
    }
  };

  cv::Mat Jacb;
  auto err_f = [&](const typename LevMarq<T>::eVector& sol, typename LevMarq<T>::eVector& err)
  {
    std::vector<cv::Point2f> p2d_rej;
    cv::Mat r, t;
    fromSol(sol, r, t);
    cv::projectPoints(p3d, r, t, cam_matrix, dist, p2d_rej, Jacb);
    err.resize(p3d.size() * 2);
    int err_idx = 0;
    for (size_t i = 0; i < p3d.size(); i++)
    {
      cv::Point2f errP = p2d_rej[i] - p2d[i];

      double SqErr = (errP.x * errP.x + errP.y * errP.y);

      float robuse_weight = getHubberMonoWeight(SqErr, 1);
      err(err_idx++) = robuse_weight * errP.x;  // p2d_rej[i].x - p2d[i].x;
      err(err_idx++) = robuse_weight * errP.y;  // p2d_rej[i].y - p2d[i].y;
    }
  };
  auto jac_f = [&](const typename LevMarq<T>::eVector& sol,
                   Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& J)
  {
    (void)(sol);
    J.resize(p3d.size() * 2, 6);
    for (size_t i = 0; i < p3d.size() * 2; i++)
    {
      double* jacb = Jacb.ptr<double>(i);
      for (int j = 0; j < 6; j++)
        J(i, j) = jacb[j];
    }
  };

  LevMarq<T> solver;
  solver.setParams(100, 0.01, 0.01);

  //  solver.verbose() = true;
  typename LevMarq<T>::eVector sol = toSol(r_io, t_io);
  auto err = solver.solve(sol, err_f, jac_f);

  fromSol(sol, r_io, t_io);
  return err;
}

double __aruco_solve_pnp(const std::vector<cv::Point3f>& p3d,
                         const std::vector<cv::Point2f>& p2d, const cv::Mat& cam_matrix,
                         const cv::Mat& dist, cv::Mat& r_io, cv::Mat& t_io)
{
#ifdef DOUBLE_PRECISION_PNP
  return __aruco_solve_pnp<double>(p3d, p2d, cam_matrix, dist, r_io, t_io);
#else
  return __aruco_solve_pnp<float>(p3d, p2d, cam_matrix, dist, r_io, t_io);
#endif
}

bool MarkerPoseTracker::estimatePose(Marker& m, const CameraParameters& _cam_params,
                                     float _msize, float minerrorRatio)
{
  if (_rvec.empty())
  {
    // if no previous data, use from scratch
    cv::Mat rv, tv;
    auto solutions = solvePnP_(Marker::get3DPoints(_msize), m, _cam_params.CameraMatrix,
                               _cam_params.Distorsion);
    double errorRatio = solutions[1].second / solutions[0].second;
    if (errorRatio < minerrorRatio)
      return false;

    //    // is the error ratio big enough
    //    cv::solvePnP(Marker::get3DPoints(_msize), m, _cam_params.CameraMatrix,
    //    _cam_params.Distorsion, rv, tv);
    //    __aruco_solve_pnp(Marker::get3DPoints(_msize), m, _cam_params.CameraMatrix,
    //    _cam_params.Distorsion, _rvec, _tvec);
    //    rv.convertTo(_rvec, CV_32F);
    //    tv.convertTo(_tvec, CV_32F);
    //    aruco_private::impl__aruco_getRTfromMatrix44(solutions[0].first, _rvec, _tvec);
  }
  else
  {
    __aruco_solve_pnp(Marker::get3DPoints(_msize), m, _cam_params.CameraMatrix,
                      _cam_params.Distorsion, _rvec, _tvec);
  }

  _rvec.convertTo(m.Rvec, CV_32F);
  _tvec.convertTo(m.Tvec, CV_32F);
  m.ssize = _msize;
  return true;
}

MarkerMapPoseTracker::MarkerMapPoseTracker()
{
  _isValid = false;
  _initial_err = -1;
  aruco_minerrratio_valid = 3;
}

void MarkerMapPoseTracker::setParams(const CameraParameters& cam_params,
                                     const MarkerMap& msconf, float markerSize)
{
  _msconf = msconf;
  _cam_params = cam_params;
  if (!cam_params.isValid())
    throw cv::Exception(9001, "Invalid camera parameters",
                        "MarkerMapPoseTracker::setParams", __FILE__, __LINE__);
  if (_msconf.mInfoType == MarkerMap::PIX && markerSize <= 0)
    throw cv::Exception(9001, "You should indicate the markersize since the MarkerMap is in pixels",
                        "MarkerMapPoseTracker::setParams", __FILE__, __LINE__);
  if (_msconf.mInfoType == MarkerMap::NONE)
    throw cv::Exception(9001, "Invalid MarkerMap", "MarkerMapPoseTracker::setParams",
                        __FILE__, __LINE__);
  if (_msconf.mInfoType == MarkerMap::PIX)
    _msconf = _msconf.convertToMeters(markerSize);

  _isValid = true;

  // create a map for fast access to elements
  _map_mm.clear();
  for (auto m : msconf)
    _map_mm.insert(std::make_pair(m.id, m));

  // now, compute the marker_m2g map
  for (auto m : _map_mm)
  {
    const Marker3DInfo& m3dinfo = m.second;
    auto p3d_marker = Marker::get3DPoints(m3dinfo.getMarkerSize());

    // compute the transform going from global to marker to using Horn
    cv::Mat RT;
    aruco_private::rigidBodyTransformation_Horn1987(m3dinfo.points, p3d_marker, RT);
    marker_m2g[m.first] = RT;
  }
}

cv::Mat MarkerMapPoseTracker::relocalization(const std::vector<Marker>& v_m)
{
  // get the markers in v_m that are in the map
  std::vector<Marker> mapMarkers;
  for (auto marker : v_m)
  {
    if (_map_mm.find(marker.id) != _map_mm.end())
      mapMarkers.push_back(marker);
  }

  if (mapMarkers.size() == 0)
    return cv::Mat();
  struct minfo
  {
    int id;
    cv::Mat rt_f2m;
    double err;
  };
  struct se3
  {
    float rt[6];
  };

  cv::Mat pose_f2g_out;  // result

  // estimate the markers locations and see if there is at least one good enough
  std::vector<minfo> good_marker_locations;
  std::vector<minfo> all_marker_locations;

  for (const Marker& marker : mapMarkers)
  {
    // for each visible marker
    auto mpi = solvePnP_(_map_mm[marker.id].getMarkerSize(), marker,
                         _cam_params.CameraMatrix, _cam_params.Distorsion);
    minfo mi;
    mi.id = marker.id;
    mi.err = mpi[0].second;
    mi.rt_f2m = mpi[0].first;
    all_marker_locations.push_back(mi);
    if (mpi[1].second / mpi[0].second > aruco_minerrratio_valid)
      good_marker_locations.push_back(mi);
    mi.rt_f2m = mpi[1].first;
    mi.err = mpi[1].second;
    all_marker_locations.push_back(mi);
  }

  // try using more than one marker approach
  if (mapMarkers.size() >= 2)
  {
    // collect all the markers 3D locations
    std::vector<cv::Point2f> markerPoints2d;
    std::vector<cv::Point3f> markerPoints3d;
    for (const Marker& marker : mapMarkers)
    {
      markerPoints2d.insert(markerPoints2d.end(), marker.begin(), marker.end());
      auto p3d = _map_mm[marker.id].points;
      markerPoints3d.insert(markerPoints3d.end(), p3d.begin(), p3d.end());
    }

    // take the all poses and select the one that minimizes the global reproj error
    for (auto& ml : all_marker_locations)
    {
      auto pose = ml.rt_f2m * marker_m2g[ml.id];
      // now, compute the repj error of all markers using this info
      ml.err = aruco_private::reprj_error(markerPoints3d, markerPoints2d, _cam_params, pose);
    }
    // sort and get the best
    std::sort(all_marker_locations.begin(), all_marker_locations.end(),
              [](const minfo& a, const minfo& b) { return a.err < b.err; });
    _initial_err = all_marker_locations.front().err;
    auto& best = all_marker_locations.front();
    pose_f2g_out = best.rt_f2m * marker_m2g[best.id];
  }

  if (pose_f2g_out.empty() && good_marker_locations.size() > 0)
  {
    std::sort(good_marker_locations.begin(), good_marker_locations.end(),
              [](const minfo& a, const minfo& b) { return a.err < b.err; });
    auto best = good_marker_locations[0];

    // estimate current location
    pose_f2g_out = best.rt_f2m * marker_m2g[best.id];
  }
  return pose_f2g_out;
}

bool MarkerMapPoseTracker::estimatePose(const std::vector<Marker>& v_m)
{
  std::vector<cv::Point2f> p2d;
  std::vector<cv::Point3f> p3d;
  for (auto marker : v_m)
  {
    if (_map_mm.find(marker.id) != _map_mm.end())
    {
      // is the marker part of the map?
      for (auto p : marker)
        p2d.push_back(p);
      for (auto p : _map_mm[marker.id].points)
        p3d.push_back(p);
    }
  }

  if (p2d.size() == 0)
  {
    // no points in the vector
    _rvec = cv::Mat();
    _tvec = cv::Mat();
    return false;
  }
  else
  {
    if (_rvec.empty())
    {
      // requires relocalization since past pose is ALL_DICTS
      // relocalization provides an initial position that will be further refined
      auto InitialPose = relocalization(v_m);
      if (InitialPose.empty())
      {
        return false;
      }

      aruco_private::impl__aruco_getRTfromMatrix44(InitialPose, _rvec, _tvec);
    }

    // refine
    __aruco_solve_pnp(p3d, p2d, _cam_params.CameraMatrix, _cam_params.Distorsion, _rvec, _tvec);

    return true;
  }
}

cv::Mat MarkerMapPoseTracker::getRTMatrix() const
{
  if (_rvec.empty() || _tvec.empty())
    return cv::Mat();
  return aruco_private::impl__aruco_getRTMatrix(_rvec, _tvec);
}

cv::Mat MarkerPoseTracker::getRTMatrix() const
{
  if (_rvec.empty() || _tvec.empty())
    return cv::Mat();
  return aruco_private::impl__aruco_getRTMatrix(_rvec, _tvec);
}

}  // namespace aruco
