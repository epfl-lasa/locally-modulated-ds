#ifndef _GP_MODULATED_DS_H_
#define _GP_MODULATED_DS_H_

#include "eigen3/Eigen/Dense"
#include "locally_modulated_ds/locally_modulated_ds.h"
#include "gaussian_process_regression/gaussian_process_regression.h"
#include <memory>
/** 
    \brief Implementation of GP-MDS. 
 */
template<typename realtype>
class GaussianProcessModulatedDS
  : public LocallyModulatedDS<Eigen::Matrix<realtype, 3, 1>,
			      Eigen::Matrix<realtype, 3, 3>> {

 private:
  static const realtype MIN_ANGLE;
  std::shared_ptr<GaussianProcessRegression<realtype> > gpr_;

 public:

  // Use Eigen for the locally-modulated DS types.
  typedef Eigen::Matrix<realtype, 3, 1> Vec;
  typedef Eigen::Matrix<realtype, 3, 3> Mat;

  // A Dynamical System is a function that maps position to velocity.
  typedef std::function<Vec(Vec)> DynamicalSystem;
  /** 
      \brief Constructor.

      @param original_dynamics a function handle for evaluating the original dynamics

  */

  GaussianProcessModulatedDS(DynamicalSystem original_dynamics)
    : LocallyModulatedDS<Vec, Mat>(original_dynamics),
    gpr_(new GaussianProcessRegression<realtype>(3, 4)) {

    gpr_->SetHyperParams(3.2,1.0,0.02);
  };

  virtual ~GaussianProcessModulatedDS() { };

  virtual Mat ModulationFunction(const Vec &in);
  Mat ModulationFunction(const Vec& aa,realtype speed_scaling);
    
  /** 
      \brief Add a single trianing pointa

  */


  void AddData(const Vec &new_pos, const Vec &new_vel);
  /** 

      \brief Batch add training data

  */

  void AddDataBatch(const std::vector<Vec>& new_pos, const std::vector<Vec>& new_vel);

    /** 

      \brief Batch add training data

  */

  void AddDataBatch(const Eigen::Matrix<realtype,3,Eigen::Dynamic>& new_pos, const Eigen::Matrix<realtype,3,Eigen::Dynamic>& new_vel );

  /** 

      \brief Clear all training data

  */
  void ClearData(){
    gpr_->ClearTrainingData();
  };
  /** 

      \brief Get a pointer to GPR

  */

  std::shared_ptr<GaussianProcessRegression<realtype> > get_gpr(){return gpr_;};
    

};

#include "locally_modulated_ds/gp_modulated_ds.hxx"

#endif // _GP_MODULATED_DS_H_
