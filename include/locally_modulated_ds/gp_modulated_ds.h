#ifndef _GP_MODULATED_DS_H_
#define _GP_MODULATED_DS_H_

#include "eigen3/Eigen/Dense"
#include "locally_modulated_ds/locally_modulated_ds.h"
#include "gaussian_process_regression/gaussian_process_regression.h"
#include <memory>

template <typename realtype>
class GaussianProcessModulatedDS : public LocallyModulatedDS<Eigen::Matrix<realtype,3,1>,Eigen::Matrix<realtype,3,3>>
{

 private:
  static const realtype MIN_ANGLE;
  typedef Eigen::Matrix<realtype,3,1> Vec;
  typedef Eigen::Matrix<realtype,3,3> Mat;
  std::unique_ptr<GaussianProcessRegression<realtype> > gpr_;
  
 public:
  GaussianProcessModulatedDS(std::function<Vec(Vec)> original_dynamics) : LocallyModulatedDS<Vec,Mat>(original_dynamics), gpr_(new GaussianProcessRegression<realtype>(3,3)){};
  

  virtual ~GaussianProcessModulatedDS(){};
  virtual Mat ModulationFunction(const Vec& in); 
  Mat ModulationFunction(const Vec& aa,realtype speed_scaling); 

  virtual void AddData(const Vec& new_pos, const Vec& new_vel);
};

#include "locally_modulated_ds/gp_modulated_ds.hxx"
#endif /* _GP_MODULATED_DS_H_ */

