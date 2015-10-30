#include <gtest/gtest.h>
#include "locally_modulated_ds/gp_modulated_ds.h"
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

// void load_data(const char *fname, vector<input_type> &inputs, vector<output_type> &outputs, int input_dim, int output_dim) {
//   input_type inp,tinp;
//   output_type outp,toutp;
//   ifstream myfile(fname);
//   ASSERT_TRUE(myfile);
//   string line;
//   while(getline(myfile,line)){
//     istringstream line_stream(line);
//     for(size_t k = 0; k < input_dim; k++)
//       line_stream>>inp(k);
//     for(size_t k = 0; k < output_dim; k++)
//       line_stream>>outp(k);
//     inputs.push_back(inp);
//     outputs.push_back(outp);
//   }
// }


Vec3 basic_dynamics(Vec3 pos) {
  return -1.0 * pos;
}

template<typename M>
void compare_matrices(M m1, M m2) {
  ASSERT_EQ(m1.rows(), m2.rows());
  ASSERT_EQ(m1.cols(), m2.cols());
  for (size_t row = 0; row < m1.rows(); row++) {
    for (size_t col = 0; col < m1.rows(); col++) {
      ASSERT_NEAR(m1(row, col), m2(row, col), FLOAT_COMPARISON_THRESHOLD);
    }
  }
}

TEST(Basic, DynamicsTestZero) {
  Vec3 vector = Vec3::Zero();
  EXPECT_NEAR(0, vector[0], FLOAT_COMPARISON_THRESHOLD);
  EXPECT_NEAR(0, vector[1], FLOAT_COMPARISON_THRESHOLD);
  EXPECT_NEAR(0, vector[2], FLOAT_COMPARISON_THRESHOLD);

  Vec3 output = basic_dynamics(vector);
  EXPECT_NEAR(0, output[0], FLOAT_COMPARISON_THRESHOLD);
  EXPECT_NEAR(0, output[1], FLOAT_COMPARISON_THRESHOLD);
  EXPECT_NEAR(0, output[2], FLOAT_COMPARISON_THRESHOLD);
}

TEST(Basic, DynamicsTestStuff) {
  Vec3 vector;
  vector << 1, 5, 10;
  EXPECT_NEAR(1, vector[0], FLOAT_COMPARISON_THRESHOLD);
  EXPECT_NEAR(5, vector[1], FLOAT_COMPARISON_THRESHOLD);
  EXPECT_NEAR(10, vector[2], FLOAT_COMPARISON_THRESHOLD);

  Vec3 output = basic_dynamics(vector);
  EXPECT_NEAR(-1, output[0], FLOAT_COMPARISON_THRESHOLD);
  EXPECT_NEAR(-5, output[1], FLOAT_COMPARISON_THRESHOLD);
  EXPECT_NEAR(-10, output[2], FLOAT_COMPARISON_THRESHOLD);
}


TEST(Basic, CreateObject) {
  GaussianProcessModulatedDS<float> gpmds(basic_dynamics);
  ASSERT_NE(nullptr, gpmds.getOriginalDynamics());
  ASSERT_TRUE((bool) gpmds.getOriginalDynamics());
}

TEST(Basic, GetOutputZeroTest) {
  // Call getOutput with zero input.
  GaussianProcessModulatedDS<float> gpmds(basic_dynamics);
  Vec3 x = gpmds.GetOutput(Vec3::Zero());
  ASSERT_NEAR(0, x[0], FLOAT_COMPARISON_THRESHOLD);
  ASSERT_NEAR(0, x[1], FLOAT_COMPARISON_THRESHOLD);
  ASSERT_NEAR(0, x[2], FLOAT_COMPARISON_THRESHOLD);
}

TEST(Basic, GetOutputStuffTest) {
  // Call getOutput with some input stuff.
  GaussianProcessModulatedDS<float> gpmds(basic_dynamics);
  Vec3 vector;
  vector << 1, 2, 3;
  auto x = snippet_gpmds_->gp_mds_->GetOutput(vector);
  EXPECT_NEAR(-1, x[0], EPS_THRESH);
  EXPECT_NEAR(-2, x[1], EPS_THRESH);
  EXPECT_NEAR(-3, x[2], EPS_THRESH);
}

TEST(Basic, FindFiles) {
  std::vector<string> file_names;
  file_names.push_back("R45z.txt");
  file_names.push_back("R162z.txt");
  for (string f:file_names) {
    ifstream ff(f);
    ASSERT_TRUE(ff);
  }
}

TEST(ModulationFunction, PureRotation45) {
  GaussianProcessModulatedDS<float> gpmds(basic_dynamics);
  auto correct_result = load_matrix_3x3("R45z.txt");
  Vec3 z_axis_45;
  z_axis_45.setZero();
  z_axis_45(2) = deg_to_rad(45.0);
  auto gpmds_result = gpmds.ModulationFunction(z_axis_45, 0.0);
  compare_matrices(gpmds_result, correct_result);
}

TEST(ModulationFunction, PureRotation162) {
  GaussianProcessModulatedDS<float> gpmds(basic_dynamics);
  auto correct_result = load_matrix_3x3("R162z.txt");
  Vec3 z_axis_162;
  z_axis_162.setZero();
  z_axis_162(2) = deg_to_rad(162.0);
  auto gpmds_result = gpmds.ModulationFunction(z_axis_162, 0.0);
  compare_matrices(gpmds_result, correct_result);
}


};

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
