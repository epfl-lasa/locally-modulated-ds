#include "locally_modulated_ds/gp_modulated_ds.h"

// define the constants
template <typename R>
const R GaussianProcessModulatedDS<R>::MIN_ANGLE = 0.001;


template <typename R>
Eigen::Matrix<R,4,1> ComputeReshapingParameters(const Eigen::Matrix<R,3,1>& act_vel, const Eigen::Matrix<R,3,1>& org_vel){
  Eigen::Matrix<R,4,1> theta;
  auto kappa = act_vel.norm()/org_vel.norm() - 1;
  if(org_vel.norm() > 1e-3)
    kappa = 1.0;

  Eigen::Quaternion<R> q;
  q.setFromTwoVectors(org_vel,act_vel);
  Eigen::AngleAxis<R> aa(q);
  for (size_t k=0; k < 3; ++k)
    theta(k) = aa.axis()(k)*aa.angle();
  theta(3) = kappa;
  return theta;
}

template <typename R>
typename GaussianProcessModulatedDS<R>::Mat GaussianProcessModulatedDS<R>::ModulationFunction(const Vec& in){
  Mat modulation_matrix;
  Eigen::Matrix<R,4,1> theta_hat;
  theta_hat = gpr_->DoRegression(in);
  Eigen::Matrix<R,3,1> aa_hat;
  for (size_t k=0; k<3; ++k)
    {
      aa_hat(k) = theta_hat(k);
    }
  auto angle = aa_hat.norm();
  if(angle < MIN_ANGLE)
    modulation_matrix.setIdentity();
  else{
    auto axis = aa_hat/angle;
    Eigen::AngleAxis<R> aa(angle,axis);
    modulation_matrix = aa.toRotationMatrix();
  }
  modulation_matrix *= 

  return modulation_matrix;
}


template <typename R>
void GaussianProcessModulatedDS<R>::AddData(const Vec& new_pos, const Vec& new_vel){
  auto reshaping_params = ComputeReshapingParameters(new_vel, this->original_dynamics_(new_pos));
  gpr_->AddTrainingData(new_pos, reshaping_params);
}

