// Author: Felix Duvallet <felix.duvallet@epfl.ch>

#include "gtest/gtest.h"
#include "snippet_gpmds/snippet_gpmds.h"
#include <memory>


namespace {

using GPMDS = GaussianProcessModulatedDS<double>;
GPMDS::Vec original_dynamics(GPMDS::Vec pose) {
  return -1.0 * pose;
}
std::function<GPMDS::Vec(GPMDS::Vec)> dynamics_fun = original_dynamics;

class SnippetTest : public ::testing::Test {

    void setUp() {

    }

 public:
    std::unique_ptr<SnippetGPMDS> snippet_gpmds_;

};

}  // namespace

TEST_F(SnippetTest, EmptyConstructorTest) {
  snippet_gpmds_.reset(new SnippetGPMDS());
  ASSERT_NE(nullptr, snippet_gpmds_->gp_mds_);
  ASSERT_EQ(nullptr, snippet_gpmds_->gp_mds_->getOriginalDynamics());
}

TEST_F(SnippetTest, ConstructorTest) {
  snippet_gpmds_.reset(new SnippetGPMDS(original_dynamics));
  ASSERT_NE(nullptr, snippet_gpmds_->gp_mds_);
  //ASSERT_NE(nullptr, snippet_gpmds_->gp_mds_->getOriginalDynamics());
  // FIXME
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
