#ifndef LINEAR_VELOCITY_FIELDS_H
#define LINEAR_VELOCITY_FIELDS_H

#include "eigen3/Eigen/Dense"
/** 
    \brief Class rerpresenting linear dynamical systems, useful to use as orginal dynamics for locally modulated dynamical systems

 */
class LinearVelocityField{
public:
  typedef double realtype;
  typedef Eigen::Matrix<realtype,Eigen::Dynamic,Eigen::Dynamic> Mat;
  typedef Eigen::Matrix<realtype,Eigen::Dynamic,1> Vec;

private:
  Vec target_;
  Mat A_;
  realtype speedcap_;
public:
  /** 
      \brief Constructor.

      @param target a vector for setting the equilibrium point of the DS
      @param A the system matrix defining the DS
      @param speedcap for saturating the output rate

  */

  LinearVelocityField(Vec target,Mat A,realtype speedcap){
  target_ = target;
  A_ = A;
  speedcap_=speedcap;
}

  /** 
      \brief Evaluate the DS at the provided location.

      @param pos a vector where you want to get the corresponding velocity

  */

  Vec ComputeVelocity(const Vec& pos){
  Vec vel = A_*(pos-target_);
  if(vel.norm()>speedcap_){
  vel /= vel.norm();
  vel *= speedcap_;
}
  return vel;
}
  /** 
      \brief Evaluate as a function call. 

      @param pos a vector where you want to get the corresponding velocity

  */

  Vec operator()(const Vec& pos){
  return this->ComputeVelocity(pos);
}
  Vec target() const{
  return target_;
}
void set_target(const Vec &target){
  target_ = target;
}
};

  /**
   //How to use:
   int D = 3;
   Mat A = -0.4*Mat::Identity(D,D)
   Vec target(D);

   LinearVelocityField straightline_field(target,A,0.3);
   // to evaluate the system:
   Vec currpos; // set its value from somewhere
   Vec vel = straightline_field(currpos);

   another example:
   Mat A(D,D); A.setZero();
   A(0,1) = 1;
   A(1,0) = -1;
   A(2,2) = -3;
   LinearVelocityField circular_path(target,A,0.3);

  **/



#endif // LINEAR_VELOCITY_FIELDS_H

