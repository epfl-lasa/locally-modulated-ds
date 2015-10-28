// Author: Felix Duvallet <felix.duvallet@epfl.ch>

#include "gtest/gtest.h"
#include "snippet_gpmds/snippet_gpmds.h"
#include <memory>


namespace {

class SnippetTest : public ::testing::Test {

    void setUp() {

    }

 public:
    std::unique_ptr<SnippetGPMDS> snippet_gpmds_;
};

}  // namespace

TEST_F(SnippetTest, NullConstructorTest) {
  snippet_gpmds_.reset(new SnippetGPMDS());
  ASSERT_EQ(nullptr, snippet_gpmds_->gp_mds_);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
