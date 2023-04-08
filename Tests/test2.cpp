#include <gtest/gtest.h>

// Demonstrate some basic assertions.
TEST(HelloTest2, BasicAssertions) {
  // Expect two strings not to be equal.
  EXPECT_STRNE("hello", "world");
  // Expect equality.
  EXPECT_EQ(7 * 6, 42);
}

class TestFw2 : public testing::Test {};

TEST_F(TestFw2, Test1) {
  EXPECT_EQ(1,1);
}

TEST_F(TestFw2, Test2) {
  EXPECT_LE(1,2);
}