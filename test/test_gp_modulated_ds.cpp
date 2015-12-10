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

class GPMDSTest : public ::testing::Test {

    void SetUp() {
      gp_mds_.reset(new GaussianProcessModulatedDS<float>(basic_dynamics));
    }

 public:
    std::unique_ptr<GaussianProcessModulatedDS<float>> gp_mds_;
    
};

}  // namespace


TEST_F(GPMDSTest, DynamicsTestZero) {
  Vec3 vector = Vec3::Zero();
  EXPECT_NEAR(0, vector[0], FLOAT_COMPARISON_THRESHOLD);
  EXPECT_NEAR(0, vector[1], FLOAT_COMPARISON_THRESHOLD);
  EXPECT_NEAR(0, vector[2], FLOAT_COMPARISON_THRESHOLD);

  Vec3 output = basic_dynamics(vector);
  EXPECT_NEAR(0, output[0], FLOAT_COMPARISON_THRESHOLD);
  EXPECT_NEAR(0, output[1], FLOAT_COMPARISON_THRESHOLD);
  EXPECT_NEAR(0, output[2], FLOAT_COMPARISON_THRESHOLD);
}

TEST_F(GPMDSTest, DynamicsTestStuff) {
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

TEST_F(GPMDSTest, CreateObject) {
  GaussianProcessModulatedDS<float> gpmds(basic_dynamics);
  ASSERT_NE(nullptr, gpmds.get_original_dynamics());
  ASSERT_TRUE((bool) gpmds.get_original_dynamics());
}

TEST_F(GPMDSTest, GetOutputZeroTest) {
  // Call getOutput with zero input.
  Vec3 x = gp_mds_->GetOutput(Vec3::Zero());
  ASSERT_NEAR(0, x[0], FLOAT_COMPARISON_THRESHOLD);
  ASSERT_NEAR(0, x[1], FLOAT_COMPARISON_THRESHOLD);
  ASSERT_NEAR(0, x[2], FLOAT_COMPARISON_THRESHOLD);
}

TEST_F(GPMDSTest, GetOutputStuffTest) {
  // Call getOutput with some input stuff.
  Vec3 vector;
  vector << 1, 2, 3;
  auto x =gp_mds_->GetOutput(vector);
  EXPECT_NEAR(-1, x[0], FLOAT_COMPARISON_THRESHOLD);
  EXPECT_NEAR(-2, x[1], FLOAT_COMPARISON_THRESHOLD);
  EXPECT_NEAR(-3, x[2], FLOAT_COMPARISON_THRESHOLD);
}

TEST_F(GPMDSTest, FindFiles) {
  std::vector<string> file_names;
  file_names.push_back("R45z.txt");
  file_names.push_back("R162z.txt");
  for (string f:file_names) {
    ifstream ff(f);
    ASSERT_TRUE(ff);
  }
}

TEST_F(GPMDSTest, PureRotation45) {
  auto correct_result = load_matrix_3x3("R45z.txt");
  Vec3 z_axis_45;
  z_axis_45.setZero();
  z_axis_45(2) = deg_to_rad(45.0);
  auto gpmds_result = gp_mds_->ModulationFunction(z_axis_45, 0.0);
  compare_matrices(gpmds_result, correct_result);
}

TEST_F(GPMDSTest, PureRotation162) {
  auto correct_result = load_matrix_3x3("R162z.txt");
  Vec3 z_axis_162;
  z_axis_162.setZero();
  z_axis_162(2) = deg_to_rad(162.0);
  auto gpmds_result = gp_mds_->ModulationFunction(z_axis_162, 0.0);
  compare_matrices(gpmds_result, correct_result);
}

TEST_F(GPMDSTest, PureScale) {
  Vec3 no_rotation = Vec3::Zero();
  float scale = 0.22;
  auto gpmds_result = gp_mds_->ModulationFunction(no_rotation, scale);
  Mat3 correct_result = Mat3::Identity();
  correct_result *= scale+1.0;
  compare_matrices(gpmds_result, correct_result);
}

TEST_F(GPMDSTest, Rotate162AndScale) {
  auto correct_result = load_matrix_3x3("R162z.txt");
  Vec3 z_axis_162;
  z_axis_162.setZero();
  z_axis_162(2) = deg_to_rad(162.0);
  float scale = 0.22;
  correct_result *= scale + 1.0;
  auto gpmds_result = gp_mds_->ModulationFunction(z_axis_162, scale);
  compare_matrices(gpmds_result, correct_result);
}

TEST_F(GPMDSTest, RotateAxisAndScale) {
  auto correct_result = load_matrix_3x3("R_test_case_234.txt");
  Vec3 aa = Vec3::Ones();
  aa *= 1.0/sqrt(3.0);
  float angle = 0.234;
  float scale = 0.22;
  correct_result *= scale + 1.0;
  auto gpmds_result = gp_mds_->ModulationFunction(aa*angle, scale);
  compare_matrices(gpmds_result, correct_result);
}

TEST_F(GPMDSTest, ComputeReshapingParams){
  Vec3 act_vel,org_vel;
  org_vel << 1.0, 0.0, 0.0;
  act_vel << 0.0, 2.0, 0.0;
  auto res = ComputeReshapingParameters(act_vel, org_vel);
  auto scale = res(3);
  Vec3 aa;
  for (int i = 0; i < 3; ++i)
    {
      aa(i) = res(i);
    }
  auto angle = aa.norm();
  EXPECT_NEAR(scale,1.0,1e-4);
  EXPECT_NEAR(angle,M_PI/2.0,1e-4);
  auto ax = aa;
  ax /= angle;
  Vec3 z;
  z<<0.,0.,1.;
  compare_matrices<Vec3>(ax,z);
}

TEST_F(GPMDSTest,ParamModulationPipeline){
  Vec3 org_vel, act_vel;
  org_vel<<1.0,1.0,1.0;
  act_vel<<-2.0,0.0,3.5;
  auto params = ComputeReshapingParameters(act_vel, org_vel);
  Vec3 aa;
  for (int i = 0; i < 3; ++i)
      aa(i) = params(i);
  auto re_vel = gp_mds_->ModulationFunction(aa, params(3))*org_vel;
  compare_matrices<Vec3>(act_vel,re_vel);
}

TEST_F(GPMDSTest, OneTrainingPoint){
  Vec3 training_pos;
  training_pos << 1,2,3;
  Vec3 training_vel;
  training_vel << -1.0,-2.0,0.0;
  // need to set a low sigma_n for this test to pass
  // the other paramters do not matter
  // SetHyperParams(length_scale, sigma_f, simga_n)
  gp_mds_->get_gpr()->SetHyperParams(0.2,1.0,0.0000001);
  gp_mds_->AddData(training_pos, training_vel);
  auto re_vel = gp_mds_->GetOutput(training_pos);
  compare_matrices<Vec3>(training_vel,re_vel);
}

TEST_F(GPMDSTest, AccessToGPR){
  Vec3 training_pos;
  training_pos << 1,2,3;
  Vec3 training_vel;
  training_vel << -1.0,-2.0,0.0;
  gp_mds_->AddData(training_pos, training_vel);
  Vec3 test_pos;
  test_pos << 1,2.4,3.3;
  auto re_vel = gp_mds_->GetOutput(test_pos);
  auto gpr = gp_mds_->get_gpr();
  gpr->SetHyperParams(1.2,0.3,0.02);
  auto re_vel2 = gp_mds_->GetOutput(test_pos);
  for (int i = 0; i < 3; ++i)
    {
      EXPECT_TRUE( fabs(re_vel(i) - re_vel2(i)) > 1e-2 );
    }
}


int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

