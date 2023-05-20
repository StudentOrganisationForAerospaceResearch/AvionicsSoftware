#include <gtest/gtest.h>

// Demonstrate some basic assertions.
TEST(HelloTest, BasicAssertions) {
  // Expect two strings not to be equal.
  EXPECT_STRNE("hello", "world");
  // Expect equality.
  EXPECT_EQ(7 * 6, 42);
}

class TestFw : public testing::Test {};

TEST_F(TestFw, Test1) {
  EXPECT_EQ(1,1);
}

TEST_F(TestFw, Test2) {
  EXPECT_NE(2,1);
}

TEST_F(TestFw, Thingy) {
  int a = 0;
  for (int i = 0; i < 0xFFFFFF; i++) {
    EXPECT_GE(i, a);
  }
  EXPECT_EQ(a, 1);
}
