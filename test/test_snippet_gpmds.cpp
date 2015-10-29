// Author: Felix Duvallet <felix.duvallet@epfl.ch>

#include "gtest/gtest.h"
#include "snippet_gpmds/snippet_gpmds.h"
#include <memory>

#define EPS_THRESH 1e-12


namespace {

using GPMDS = GaussianProcessModulatedDS<double>;
GPMDS::Vec original_dynamics(GPMDS::Vec pose) {
  return -1.0 * pose;
}
std::function<GPMDS::Vec(GPMDS::Vec)> dynamics_fun = original_dynamics;

class SnippetTest : public ::testing::Test {

    void SetUp() {
      snippet_gpmds_.reset(new SnippetGPMDS(dynamics_fun));
    }

 public:
    std::unique_ptr<SnippetGPMDS> snippet_gpmds_;
};

}  // namespace

TEST_F(SnippetTest, SetupTest) {
  ASSERT_NE(nullptr, snippet_gpmds_);
}


TEST_F(SnippetTest, EmptyConstructorTest) {
  // Passing null dynamics results in null original dynamics.
  snippet_gpmds_.reset(new SnippetGPMDS());
  ASSERT_NE(nullptr, snippet_gpmds_->gp_mds_);  // not null
  ASSERT_EQ(nullptr, snippet_gpmds_->gp_mds_->getOriginalDynamics());
}

TEST_F(SnippetTest, ConstructorTest) {
  // Ensure providing the dynamics sets them correctly.
  ASSERT_NE(nullptr, snippet_gpmds_->gp_mds_);
  EXPECT_NE(nullptr, snippet_gpmds_->gp_mds_->getOriginalDynamics());
  EXPECT_TRUE((bool)snippet_gpmds_->gp_mds_->getOriginalDynamics());
}

TEST_F(SnippetTest, GetOutputTest) {
  // Without any modulation, the result of GetOutput is simply the original
  // dynamics.
  auto x = snippet_gpmds_->gp_mds_->GetOutput(GPMDS::Vec());
  ASSERT_NEAR(0, x[0], EPS_THRESH);
  ASSERT_NEAR(0, x[1], EPS_THRESH);
  ASSERT_NEAR(0, x[2], EPS_THRESH);
}

TEST_F(SnippetTest, TestOriginalDynamics) {
  // Make sure the original dynamics are valid.
  ASSERT_NE(nullptr, original_dynamics);
  ASSERT_TRUE((bool)original_dynamics);
  auto x = dynamics_fun(GPMDS::Vec());
  ASSERT_NEAR(0, x[0], EPS_THRESH);
  ASSERT_NEAR(0, x[1], EPS_THRESH);
  ASSERT_NEAR(0, x[2], EPS_THRESH);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
