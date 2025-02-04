// Test aimGoAim()
// Note range checks done elsewhere so not tested here

#include "aim_go_aim.h"
#include <gtest/gtest.h>

// Test nothing to do
TEST(TestSuite, testCase1)
{
  turtlesim::Pose start, end;
  auto output = aimGoAim(start, end, 0.1, 0.1);
  EXPECT_EQ(output.linear.x, 0);
  EXPECT_EQ(output.linear.y, 0);
  EXPECT_EQ(output.angular.z, 0);
}

// Test: At position, yaw low
TEST(TestSuite, testCase2)
{
  turtlesim::Pose start, end;
  start.x = 5.0;
  start.y = 3.0;
  start.theta = 1.1;
  end.x = 5.0;
  end.y = 3.0;
  end.theta = 1.5;
  auto output = aimGoAim(start, end, 0.1, 0.1);
  EXPECT_EQ(output.linear.x, 0);
  EXPECT_EQ(output.linear.y, 0);
  EXPECT_NEAR(output.angular.z, 0.4, 0.001);
}

// Test: At position, yaw high
TEST(TestSuite, testCase3)
{
  turtlesim::Pose start, end;
  start.x = 5.0;
  start.y = 3.0;
  start.theta = 1.5;
  end.x = 5.0;
  end.y = 3.0;
  end.theta = 1.1;
  auto output = aimGoAim(start, end, 0.1, 0.1);
  EXPECT_EQ(output.linear.x, 0);
  EXPECT_EQ(output.linear.y, 0);
  EXPECT_NEAR(output.angular.z, -0.4, 0.001);
}

// Test: Off position, facing target
TEST(TestSuite, testCase4)
{
  turtlesim::Pose start, end;
  start.x = 1.0;
  start.y = 1.0;
  start.theta = 0.785;
  end.x = 11.0;
  end.y = 11.0;
  end.theta = 1.1;
  auto output = aimGoAim(start, end, 0.1, 0.1);
  EXPECT_NEAR(output.linear.x, 14.142, 0.01);
  EXPECT_EQ(output.linear.y, 0);
  EXPECT_NEAR(output.angular.z, 0.0, 0.001);
}

// Test: Off position, not facing target CW
TEST(TestSuite, testCase5)
{
  turtlesim::Pose start, end;
  start.x = 1.0;
  start.y = 1.0;
  start.theta = 0.5;
  end.x = 11.0;
  end.y = 11.0;
  end.theta = 1.1;
  auto output = aimGoAim(start, end, 0.1, 0.1);
  EXPECT_EQ(output.linear.y, 0);
  EXPECT_EQ(output.linear.y, 0);
  EXPECT_NEAR(output.angular.z, 0.285, 0.001);
}

// TODO: Test: Off position, not facing target CCW
// TODO:.... corners, edges ....

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
