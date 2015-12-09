#include <gtest/gtest.h>
#include "locally_modulated_ds/spline_modulated_ds.h"
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;

namespace {

typedef Eigen::Matrix<float, 3, 1> Vec3;
typedef Eigen::Matrix<float, 3, 3> Mat3;
static double FLOAT_COMPARISON_THRESHOLD = 1.0e-5;

template<typename R>
R deg_to_rad(R deg) {
  return deg / 180.0 * M_PI;
}

Mat3 load_matrix_3x3(const char *fname) {
  ifstream myfile(fname);
  Mat3 mat;
  for (size_t row = 0; row < 3; row++) {
    string line;
    getline(myfile, line);
    istringstream line_stream(line);
    for (size_t col = 0; col < 3; col++) {
      line_stream >> mat(row, col);
    }
  }
  return mat;
}


Vec3 basic_dynamics(Vec3 pos) {
  return -1.0 * pos;
}


template<typename M>
void compare_matrices(M m1, M m2) {
  ASSERT_EQ(m1.rows(), m2.rows());
  ASSERT_EQ(m1.cols(), m2.cols());
  for (size_t row = 0; row < m1.rows(); row++) {
    for (size_t col = 0; col < m1.cols(); col++) {
      ASSERT_NEAR(m1(row, col), m2(row, col), FLOAT_COMPARISON_THRESHOLD);
    }
  }
}


  TEST(SplineTest,BasicTest){
    ASSERT_EQ(1,1);
    // put your tests here
  }

  TEST(SplineTest,OtherTest){
    ASSERT_EQ(1,1);
    // or here
  }

  // etc....

} // end of namespace
int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

