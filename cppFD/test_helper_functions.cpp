#include <gtest/gtest.h>

#include "header.hpp"

class testHelperFunctions : public testing::Test
{
protected:
  int a, b;

  void SetUp() {};

  void TearDown() {};
};

TEST_F(testHelperFunctions, testName1)
{
  EXPECT_EQ(1.0, 1.0);
  EXPECT_TRUE(true);
}

int main(int argc, char ** argv)
{
  // Start testing
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
