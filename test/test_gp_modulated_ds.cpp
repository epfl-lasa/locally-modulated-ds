#include <gtest/gtest.h>
#include "locally_modulated_ds/gp_modulated_ds.h"
#include <iostream>

namespace{

typedef Eigen::Matrix<float,3,1> Vec3;

 Vec3 original_dynamics(Vec3 pos){
   return -1.0*pos;
 }

TEST(Basic,CreateObject){
  GaussianProcessModulatedDS<float> gpmds(original_dynamics);
}

 



};

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
