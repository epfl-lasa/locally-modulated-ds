#include "locally_modulated_ds/spline_modulated_ds.h"
#include <iostream>
// helper function that I copied from other code

Eigen::Matrix<spl_real,4,1> ComputeReshapingParameters(const Eigen::Matrix<spl_real,3,1>& act_vel, const Eigen::Matrix<spl_real,3,1>& org_vel){
  Eigen::Matrix<spl_real,4,1> theta;
  // first, speed scaling with bias
  spl_real kappa;
  kappa = act_vel.norm()/org_vel.norm() - 1;
  // TODO: add something to deal with low org_vel here!
  Eigen::Quaternion<spl_real> q;
  q.setFromTwoVectors(org_vel,act_vel);
  Eigen::AngleAxis<spl_real> aa(q);
  for (size_t k=0; k < 3; ++k)
    theta(k) = aa.axis()(k)*aa.angle();
  theta(3) = kappa;
  return theta;
}


SplineModulatedDS::Mat SplineModulatedDS::ModulationFunction(const Vec& angle_axis, spl_real speed_scaling){
  auto angle = angle_axis.norm();
  Mat modulation_matrix;
  double MIN_ANGLE = 0.001*M_PI;
  if(angle < MIN_ANGLE)
    modulation_matrix.setIdentity();
  else{
    auto axis = angle_axis/angle;
    Eigen::AngleAxis<spl_real> aa(angle,axis);
    modulation_matrix = aa.toRotationMatrix();
  }
  // We do not allow speed scaling to stop the motion.
  spl_real speed_scaling_final = speed_scaling + 1.0;
  if(speed_scaling_final < 0.1){
    speed_scaling_final = 0.1;
    std::cout<<"thresholding! before: "<<speed_scaling+1.0<<" after: "<<speed_scaling_final<<std::endl;
  }
  modulation_matrix *= speed_scaling_final;
  return modulation_matrix;
}

SplineModulatedDS::Mat SplineModulatedDS::ModulationFunction(const Vec& in){
  // 1) find closest point p on spline
  // 2) get velocity v on point p
  // 3) compute reshaping paramters to reshape original_dynamics(p) to velocity v
  // 4) compute generalization, gpr-equation with a single data point, this reuslts in reshaping paramters to apply 
  Eigen::Matrix<spl_real,3,1> angle_axis_hat;
  spl_real speed_scaling;
  // fill correct values for rotation and speed scaling, you can use the function ComputeReshpingParamters
  return ModulationFunction(angle_axis_hat, speed_scaling);
}
