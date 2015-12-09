#ifndef _SPLINE_H_
#define _SPLINE_H_

#include "eigen3/Eigen/Dense"
#include <memory>

class CubicSpline{
 private:
  // what do you need?
  Eigen::MatrixXd coeffecients_;
  
 public:
  //constructor, modify as needed
  CubicSpline(){};
  // a few suggestions for methods you might want to define.
  // put your implementations in spline.cpp
  void OptimizeCoefficients(const Eigen::VectorXd& x, const Eigen::VectorXd& y);
  double GetOutput(double x);
  Eigen::VectorXd GetOutput(const Eigen::VectorXd& x);
};

#endif // _SPLINE_H_
