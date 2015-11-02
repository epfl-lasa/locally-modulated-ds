#include <gtest/gtest.h>
#include "locally_modulated_ds/locally_modulated_ds.h"
#include <iostream>
#include "eigen3/Eigen/Dense"

namespace {

// Define types used in the class.
using Vec3 = Eigen::Matrix<float, 3, 1>;
using Mat3 = Eigen::Matrix<float, 3, 3>;

// Define a class just for the test; this class implements the (virtual)
// modulation function.
class DebuggingLocallyMDS : public LocallyModulatedDS<Vec3, Mat3> {
    Mat3 ModulationFunction(const Vec3 &input) {
      return Mat3();
    }
};

// Simple dynamics.
Vec3 basic_dynamics(Vec3 pose) {
  return -1.0 * pose;
}

// Test fixtures.
class TestLocallyMDS : public ::testing::Test {
    void setUp() {

    }

 public:
    DebuggingLocallyMDS lmds_;

};


}  // namespace

TEST_F(TestLocallyMDS, StartWithNoDynamics) {
  // LMDS starts with no original dynamics. Cast to bool = false means the
  // function is not callable.
  ASSERT_EQ(nullptr, lmds_.get_original_dynamics());
  ASSERT_FALSE((bool) lmds_.get_original_dynamics());

}

TEST_F(TestLocallyMDS, SetDynamics) {
  // Set the dynamics, it should no longer be null.
  lmds_.set_original_dynamics(basic_dynamics);
  ASSERT_NE(nullptr, lmds_.get_original_dynamics());
  ASSERT_TRUE((bool) lmds_.get_original_dynamics());
}

TEST_F(TestLocallyMDS, IdiotTest){
  ASSERT_EQ(1, 1);
}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
