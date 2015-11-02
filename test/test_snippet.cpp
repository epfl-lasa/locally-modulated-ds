// Author: Felix Duvallet <felix.duvallet@epfl.ch>

#include "gtest/gtest.h"
#include "snippet_gpmds/snippet.h"
#include <memory>
#include <vector>

using namespace std;

namespace {

// Create a 1D snippet using floats.
using SnippetFloat = Snippet<float>;

class SnippetTest : public ::testing::Test {

    void SetUp() {

    }

 public:
    std::unique_ptr<SnippetFloat> snippet_;
};

}  // namespace

TEST_F(SnippetTest, TestConstructorNoPoints) {
  SnippetFloat snippet(5);

  EXPECT_EQ(5, snippet.get_anchor());
  EXPECT_EQ(0, snippet.num_points());

  vector<float> vec;
  snippet.RelativePoints(&vec);
  ASSERT_EQ(0, vec.size());
}

TEST_F(SnippetTest, TestConstructorWithPoints) {
  vector<float> points {5, 10, 15};
  SnippetFloat snippet(5, points);

  EXPECT_EQ(5, snippet.get_anchor());
  EXPECT_EQ(3, snippet.num_points());

  vector<float> vec;
  snippet.RelativePoints(&vec);
  ASSERT_EQ(3, vec.size());
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
