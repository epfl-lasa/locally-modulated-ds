//
//
// Author: Felix Duvallet <felix.duvallet@epfl.ch>


#include <iostream>
#include "snippet_gpmds.h"
#include <locally_modulated_ds/gp_modulated_ds.h>

using GPMDS = GaussianProcessModulatedDS<double>;
using Vec3 = GPMDS::Vec;

/*
 * Vec3 GPD
        GaussianProcessModulatedDS::DynamicalSystem original_dynamics(
        GaussianProcessModulatedDS::Vec) {
  GaussianProcessModulatedDS::Vec ret;
  return ret;
}
 */

//Vec3



int main() {
  printf("hello world");

  //std::function<Vec3(Vec3)> dynamics_fun = original_dynamics;

  //SnippetGPMDS snippet_gpmds(dynamics_fun);

  return 0;
}