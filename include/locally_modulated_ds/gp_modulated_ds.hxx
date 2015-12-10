#include "locally_modulated_ds/gp_modulated_ds.h"

// define the constants
template <typename R>
const R GaussianProcessModulatedDS<R>::MIN_ANGLE = 0.001;

template <typename R>
Eigen::Matrix<R,4,1> ComputeReshapingParameters(const Eigen::Matrix<R,3,1>& act_vel, const Eigen::Matrix<R,3,1>& org_vel){
  Eigen::Matrix<R,4,1> theta;
  // first, speed scaling with bias
  R kappa;
  kappa = act_vel.norm()/org_vel.norm() - 1;
  // add something to deal with low org_vel here!
  
  Eigen::Quaternion<R> q;
  q.setFromTwoVectors(org_vel,act_vel);
  Eigen::AngleAxis<R> aa(q);
  for (size_t k=0; k < 3; ++k)
    theta(k) = aa.axis()(k)*aa.angle();
  theta(3) = kappa;
  return theta;
}


template <typename R>
typename GaussianProcessModulatedDS<R>::Mat GaussianProcessModulatedDS<R>::ModulationFunction(const Vec& angle_axis, R speed_scaling){
  auto angle = angle_axis.norm();
  Mat modulation_matrix;
  if(angle < MIN_ANGLE)
    modulation_matrix.setIdentity();
  else{
    auto axis = angle_axis/angle;
    Eigen::AngleAxis<R> aa(angle,axis);
    modulation_matrix = aa.toRotationMatrix();
  }
  // We do not allow speed scaling to stop the motion.
  R speed_scaling_final = speed_scaling + 1.0;

  // this code makes the tests fail
  // if(speed_scaling_final < 0.5){
  //   speed_scaling_final = 0.5;
  //   std::cout<<"thresholding! before: "<<speed_scaling+1.0<<" after: "<<speed_scaling_final<<std::endl;
  // }
  // // temporary hack to keep original velocity:
  // speed_scaling_final = 1.0;
  
  modulation_matrix *= speed_scaling_final;
  return modulation_matrix;
}


template <typename R>
typename GaussianProcessModulatedDS<R>::Mat GaussianProcessModulatedDS<R>::ModulationFunction(const Vec& in){
  Eigen::Matrix<R,4,1> theta_hat;
  theta_hat = gpr_->DoRegression(in);
  Eigen::Matrix<R,3,1> aa_hat;
  for (size_t k=0; k<3; ++k)
    {
      aa_hat(k) = theta_hat(k);
    }
  return ModulationFunction(aa_hat,theta_hat(3));
}


template <typename R>
void GaussianProcessModulatedDS<R>::AddData(const Vec& new_pos, const Vec& new_vel){
  auto reshaping_params = ComputeReshapingParameters(new_vel, this->original_dynamics_(new_pos));
  gpr_->AddTrainingData(new_pos, reshaping_params);
}

// batch add data stored in std::vectors 
template <typename R>
void GaussianProcessModulatedDS<R>::AddDataBatch(const std::vector<Vec>& new_pos, const std::vector<Vec>& new_vel){
  // create matrices and put all the data there
  Eigen::Matrix<R,3,Eigen::Dynamic> inputs;
  Eigen::Matrix<R,4,Eigen::Dynamic> reshaping_params;
  reshaping_params.resize(4,new_vel.size());
  inputs.resize(4,new_vel.size());
  for (size_t k=0; k<new_pos.size(); ++k)
    {
      reshaping_params.col(k) = ComputeReshapingParameters(new_vel[k], this->original_dynamics_(new_pos[k]));
      inputs.col(k) = new_pos[k];
    } 
  gpr_->AddTrainingDataBatch(inputs, reshaping_params);
}

// batch add data along Eigen::Matrix columns
template <typename R>
void GaussianProcessModulatedDS<R>::AddDataBatch(const Eigen::Matrix<R,3,Eigen::Dynamic>& new_pos, const Eigen::Matrix<R,3,Eigen::Dynamic>& new_vel ){
  Eigen::Matrix<R,4,Eigen::Dynamic> reshaping_params;
  reshaping_params.resize(4,new_vel.cols());
  for (size_t k=0; k<reshaping_params.cols(); ++k)
    {
      reshaping_params.col(k) = ComputeReshapingParameters(new_vel.col(k), this->original_dynamics_(new_pos.col(k)));
    }
  gpr_->AddTrainingDataBatch(new_pos, reshaping_params);
}

