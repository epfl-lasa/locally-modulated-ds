#ifndef _SPLINE_MODULATED_DS_H_
#define _SPLINE_MODULATED_DS_H_

#include "eigen3/Eigen/Dense"
#include "locally_modulated_ds/locally_modulated_ds.h"
#include "locally_modulated_ds/spline.h"
#include <memory>

typedef double spl_real;

class SplineModulatedDS
  : public LocallyModulatedDS<Eigen::Matrix<spl_real, 3, 1>,
			      Eigen::Matrix<spl_real, 3, 3>> {

 private:
  std::vector<CubicSpline> splines_;
  
 public:
  // Use Eigen for the locally-modulated DS types.
  typedef Eigen::Matrix<spl_real, 3, 1> Vec;
  typedef Eigen::Matrix<spl_real, 3, 3> Mat;

  // A Dynamical System is a function that maps position to velocity.
  typedef std::function<Vec(Vec)> DynamicalSystem;

  SplineModulatedDS(DynamicalSystem original_dynamics)
    : LocallyModulatedDS<Vec, Mat>(original_dynamics){
    // if you need to do something in the constructor put it here
    // or move the implementation to the cpp file.
  };
  // a destructor, empty for now
  virtual ~SplineModulatedDS() { };
  // this function is necessary to define, it determines the behavior of lmds
  virtual Mat ModulationFunction(const Vec &in);
  // this is a helper function that I copied from the gp_mds class
  Mat ModulationFunction(const Vec& aa,spl_real speed_scaling);

  // Add a new spline to the system
  void AddSpline(const CubicSpline& spl);
    
  void Clear(){
    // remove any splines that we have added 
    splines_.clear();
  };

};

#endif // _SPLINE_MODULATED_DS_H_
