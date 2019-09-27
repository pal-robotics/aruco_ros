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
 */

#ifndef ARUCO_MM__LevMarq_H
#define ARUCO_MM__LevMarq_H

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <functional>
#include <iostream>
#include <cmath>
#include <ctime>
#include <cstring>
#include <vector>
#include <chrono>
#include <iomanip>

namespace aruco
{

// Levenberg-Marquardt method for general problems Inspired in
//@MISC\{IMM2004-03215,
//    author       = "K. Madsen and H. B. Nielsen and O. Tingleff",
//    title        = "Methods for Non-Linear Least Squares Problems (2nd ed.)",
//    year         = "2004",
//    pages        = "60",
//    publisher    = "Informatics and Mathematical Modelling, Technical University of Denmark, {DTU}",
//    address      = "Richard Petersens Plads, Building 321, {DK-}2800 Kgs. Lyngby",
//    url          = "http://www.ltu.se/cms_fs/1.51590!/nonlinear_least_squares.pdf"
//}
template<typename T>
class LevMarq
{
public:
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> eVector;
  typedef std::function<void(const eVector &, eVector &)> F_z_x;
  typedef std::function<void(const eVector &, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &)> F_z_J;

  LevMarq();

  /**
   * @brief Constructor with params
   * @param maxIters maximum number of iterations of the algorithm
   * @param minError to stop the algorithm before reaching the max iterations
   * @param min_step_error_diff minimum error difference between two iterations. If below this level, then stop.
   * @param tau parameter indicating how near the initial solution is estimated to be to the real one. If 1, it means that it is very far and the first
   * @param der_epsilon increment to calculate the derivative of the evaluation function
   * step will be very short. If near 0, means the opposite. This value is auto calculated in the subsequent iterations.
   */
  LevMarq(int maxIters, double minError, double min_step_error_diff = 0, double tau = 1, double der_epsilon = 1e-3);

  /**
   * @brief setParams
   * @param maxIters maximum number of iterations of the algorithm
   * @param minError to stop the algorithm before reaching the max iterations
   * @param min_step_error_diff minimum error difference between two iterations. If below this level, then stop.
   * @param tau parameter indicating how near the initial solution is estimated to be to the real one. If 1, it means that it is very far and the first
   * @param der_epsilon increment to calculate the derivative of the evaluation function
   * step will be very short. If near 0, means the opposite. This value is auto calculated in the subsequent iterations.
   */
  void setParams(int maxIters, double minError, double min_step_error_diff = 0, double tau = 1, double der_epsilon =
                     1e-3);

  /**
   * @brief solve  non linear minimization problem ||F(z)||, where F(z)=f(z) f(z)^t
   * @param z  function params 1xP to be estimated. input-output. Contains the result of the optimization
   * @param f_z_x evaluation function  f(z)=x
   *          first parameter : z :  input. Data is in double precision as a row vector (1xp)
   *          second parameter : x :  output. Data must be returned in double
   * @param f_J  computes the jacobian of f(z)
   *          first parameter : z :  input. Data is in double precision as a row vector (1xp)
   *          second parameter : J :  output. Data must be returned in double
   * @return final error
   */
  double solve(eVector &z, F_z_x, F_z_J);
  // Step by step solve mode

  /**
   * @brief init initializes the search engine
   * @param z
   */
  void init(eVector &z, F_z_x);

  /**
   * @brief step gives a step of the search
   * @param f_z_x error evaluation function
   * @param f_z_J Jacobian function
   * @return error of current solution
   */
  bool step(F_z_x f_z_x, F_z_J f_z_J);
  bool step(F_z_x f_z_x);

  /**
   * @brief getCurrentSolution returns the current solution
   * @param z output
   * @return error of the solution
   */
  double getCurrentSolution(eVector &z);

  /**
   * @brief getBestSolution sets in z the best solution up to this moment
   * @param z output
   * @return  error of the solution
   */
  double getBestSolution(eVector &z);

  /**  Automatic jacobian estimation
   * @brief solve  non linear minimization problem ||F(z)||, where F(z) = f(z) f(z)^t
   * @param z  function params 1xP to be estimated. input-output. Contains the result of the optimization
   * @param f_z_x evaluation function  f(z) = x
   *          first parameter : z :  input. Data is in double precision as a row vector (1xp)
   *          second parameter : x :  output. Data must be returned in double
   * @return final error
   */
  double solve(eVector &z, F_z_x);

  // to enable verbose mode
  bool & verbose()
  {
    return _verbose;
  }

  // sets a callback func call at each step
  void setStepCallBackFunc(std::function<void(const eVector &)> callback)
  {
    _step_callback = callback;
  }

  // sets a function that indicates when the algorithm must be stop. returns true if must stop and false otherwise
  void setStopFunction(std::function<bool(const eVector &)> stop_function)
  {
    _stopFunction = stop_function;
  }

  void calcDerivates(const eVector & z, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &, F_z_x);
private:
  int _maxIters;
  double _minErrorAllowed, _der_epsilon, _tau, _min_step_error_diff;
  bool _verbose;

  //--------
  eVector curr_z, x64;
  double currErr, prevErr, minErr;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> I, _J;
  double mu, v;
  std::function<void(const eVector &)> _step_callback;
  std::function<bool(const eVector &)> _stopFunction;

};

template<typename T>
LevMarq<T>::LevMarq()
{
  _maxIters = 1000;
  _minErrorAllowed = 0;
  _der_epsilon = 1e-3;
  _verbose = false;
  _tau = 1;
  v = 5;
  _min_step_error_diff = 0;
}

/**
 * @brief Constructor with params
 * @param maxIters maximum number of iterations of the algorithm
 * @param minError to stop the algorithm before reaching the max iterations
 * @param min_step_error_diff minimum error difference between two iterations. If below this level, then stop.
 * @param tau parameter indicating how near the initial solution is estimated to be to the real one. If 1, it means that it is very far and the first
 * @param der_epsilon increment to calculate the derivative of the evaluation function
 * step will be very short. If near 0, means the opposite. This value is auto calculated in the subsequent iterations.
 */
template<typename T>
LevMarq<T>::LevMarq(int maxIters, double minError, double min_step_error_diff, double tau, double der_epsilon)
{
  _maxIters = maxIters;
  _minErrorAllowed = minError;
  _der_epsilon = der_epsilon;
  _verbose = false;
  _tau = tau;
  v = 5;
  _min_step_error_diff = min_step_error_diff;
}

/**
 * @brief setParams
 * @param maxIters maximum number of iterations of the algoritm
 * @param minError to stop the algorithm before reaching the max iterations
 * @param min_step_error_diff minimum error difference between two iterations. If below this level, then stop.
 * @param tau parameter indicating how near the initial solution is estimated to be to the real one. If 1, it means that it is very far and the first
 * @param der_epsilon increment to calculate the derivate of the evaluation function
 * step will be very short. If near 0, means the opposite. This value is auto calculated in the subsequent iterations.
 */
template<typename T>
void LevMarq<T>::setParams(int maxIters, double minError, double min_step_error_diff, double tau, double der_epsilon)
{
  _maxIters = maxIters;
  _minErrorAllowed = minError;
  _der_epsilon = der_epsilon;
  _tau = tau;
  _min_step_error_diff = min_step_error_diff;
}

template<typename T>
void LevMarq<T>::calcDerivates(const eVector & z, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &J, F_z_x f_z_x)
{
  for (int i = 0; i < z.rows(); i++)
  {
    eVector zp(z), zm(z);
    zp(i) += _der_epsilon;
    zm(i) -= _der_epsilon;
    eVector xp, xm;
    f_z_x(zp, xp);
    f_z_x(zm, xm);
    eVector dif = (xp - xm) / (2.f * _der_epsilon);
    J.middleCols(i, 1) = dif;
  }
}

template<typename T>
double LevMarq<T>::solve(eVector &z, F_z_x f_z_x)
{
  return solve(z, f_z_x, std::bind(&LevMarq::calcDerivates, this, std::placeholders::_1, std::placeholders::_2, f_z_x));
}

template<typename T>
bool LevMarq<T>::step(F_z_x f_z_x)
{
  return step(f_z_x, std::bind(&LevMarq::calcDerivates, this, std::placeholders::_1, std::placeholders::_2, f_z_x));
}

template<typename T>
void LevMarq<T>::init(eVector &z, F_z_x f_z_x)
{
  curr_z = z;
  I.resize(z.rows(), z.rows());
  I.setIdentity();
  f_z_x(curr_z, x64);
  minErr = currErr = prevErr = x64.cwiseProduct(x64).sum();
  _J.resize(x64.rows(), z.rows());
  mu = -1;
}

#define splm_get_time(a,b) std::chrono::duration_cast<std::chrono::duration<double>>(a - b).count()

template<typename T>
bool LevMarq<T>::step(F_z_x f_z_x, F_z_J f_J)
{
  f_J(curr_z, _J);
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> Jt = _J.transpose();
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> JtJ = (Jt * _J);

  eVector B = -Jt * x64;
  if (mu < 0)
  {
    // first time only
    int max = 0;
    for (int j = 1; j < JtJ.cols(); j++)
      if (JtJ(j, j) > JtJ(max, max))
        max = j;
    mu = JtJ(max, max) * _tau;
  }

  double gain = 0, prev_mu = 0;
  int ntries = 0;
  bool isStepAccepted = false;
  do
  {
    // add/update dumping factor to JtJ.
    // very efficient in any case, but particularly if initial dump does not produce improvement and must reenter
    for (int j = 0; j < JtJ.cols(); j++)
      JtJ(j, j) += mu - prev_mu; // update mu
    prev_mu = mu;
    eVector delta = JtJ.ldlt().solve(B);
    eVector estimated_z = curr_z + delta;

    // compute error
    f_z_x(estimated_z, x64);
    auto err = x64.cwiseProduct(x64).sum();
    auto L = 0.5 * delta.transpose() * ((mu * delta) - B);
    gain = (err - prevErr) / L(0, 0);

    // get gain
    if (gain > 0)
    {
      mu = mu * std::max(double(0.33), 1. - pow(2 * gain - 1, 3));
      v = 5.f;
      currErr = err;
      curr_z = estimated_z;
      isStepAccepted = true;
    }
    else
    {
      mu = mu * v;
      v = v * 5;
    }
  } while (gain <= 0 && ntries++ < 5);

  if (_verbose)
    std::cout << std::setprecision(5) << "Curr Error=" << currErr << " AErr(prev - curr) = " << prevErr - currErr
        << " gain = " << gain << " dumping factor = " << mu << std::endl;
  // check if we must move to the new position or exit
  if (currErr < prevErr)
    std::swap(currErr, prevErr);

  return isStepAccepted;

}

template<typename T>
double LevMarq<T>::getCurrentSolution(eVector &z)
{
  z = curr_z;
  return currErr;
}

template<typename T>
double LevMarq<T>::solve(eVector &z, F_z_x f_z_x, F_z_J f_J)
{
  init(z, f_z_x);

  if (_stopFunction)
  {
    do
    {
      step(f_z_x, f_J);
      if (_step_callback)
        _step_callback(curr_z);
    } while (!_stopFunction(curr_z));
  }
  else
  {
    // Initial error estimation
    int mustExit = 0;
    for (int i = 0; i < _maxIters && !mustExit; i++)
    {
      if (_verbose)
        std::cerr << "iteration " << i << "/" << _maxIters << "  ";
      bool isStepAccepted = step(f_z_x, f_J);

      // check if we must exit
      if (currErr < _minErrorAllowed)
        mustExit = 1;
      if (fabs(prevErr - currErr) <= _min_step_error_diff || !isStepAccepted)
        mustExit = 2;

      // exit if error increment
      if (currErr < prevErr)
        mustExit = 3;
//      if ((prevErr - currErr) < 1e-5)
//        mustExit = true;
      if (_step_callback)
        _step_callback(curr_z);
    }

//    std::cout << "Exit code = " << mustExit << std::endl;
  }

  z = curr_z;
  return currErr;
}

} // namespace aruco

#endif /* ARUCO_MM__LevMarq_H */
