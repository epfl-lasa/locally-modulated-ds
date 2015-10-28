//
//
// Author: Felix Duvallet <felix.duvallet@epfl.ch>


#include <iostream>
#include <locally_modulated_ds/gp_modulated_ds.h>
#include "snippet_gpmds/snippet_gpmds.h"

using GPMDS = GaussianProcessModulatedDS<double>;
using Vec3 = GPMDS::Vec;

Vec3 original_dynamics(Vec3 pose) {
  return -1.0 * pose;
}

int main() {
  printf("Snippet GPMDS");

  std::function<Vec3(Vec3)> dynamics_fun = original_dynamics;

  SnippetGPMDS snippet_gpmds(dynamics_fun);

  return 0;
}