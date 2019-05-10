#include "FakeHeadingSensor.h"

#include "gtest/gtest.h"

#include <fstream>

TEST(FakeHeadingSensorTest, CalculateHeadingTest) {
  std::set<int> anchors{0,2};
  std::set<int> robots{1,4};
  double baseLineLen = 2.0478;
  FakeHeadingSensor s(anchors, robots, baseLineLen);

  Eigen::Matrix<double, 2,2> range;
  range << 1.0171, 2.0679, 0.9767, 2.3819;
  auto xyphi = s.CalculateHeading(range);
  ASSERT_NEAR(xyphi[2], -3.0807, 1e-4);
}

TEST(FakeHeadingSensorTest, LinearFitTest) {
  std::set<int> anchors{0,2};
  std::set<int> robots{1,4};
  double baseLineLen = 2.0478;
  FakeHeadingSensor s(anchors, robots, baseLineLen);

  std::deque<double> data;
  for (int i = 1; i <= 10; ++i) {
    data.push_back(2*i+2);
  }
  auto coeff = s.LinearFit(data);
  ASSERT_EQ(coeff[0], 2);
  ASSERT_EQ(coeff[1], 2);
}