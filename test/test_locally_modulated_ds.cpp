#include <gtest/gtest.h>
#include "locally_modulated_ds/locally_modulated_ds.h"
#include <iostream>


TEST(IdiotTest,IdiotDescription){
  ASSERT_EQ(1,1);
}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
