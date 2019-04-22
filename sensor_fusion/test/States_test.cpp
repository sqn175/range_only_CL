#include "States.h"

#include "gtest/gtest.h"

#include <iostream>

TEST(StatesTest, propagate) {
  States ini(1.300834753504995, 1.640666898835621, 0.754904384307536);
  double v0 = 0.091845169663429;
  double omega0 = -0.299327298998833;
  double v1 = 0.066344467923045;
  double omega1 = -0.299301259219646;
  double deltaSec = 0.05;
  ini.propagate(v0, omega0, v1, omega1, deltaSec, false);

  States expectedStates(1.303715137910676, 1.643376762591485, 0.739938670352074);
  EXPECT_TRUE(ini.serialize().isApprox(expectedStates.serialize(), 1e-9));
}

TEST(StatesTest, correct) {
  Eigen::Vector3d delta(-0.001, 0.00075, 0);
  States ini(1.3037,1.6434,0.73994);
  ini.correct(delta);

  States expectedStates(1.3027,1.64415,0.73994);
  EXPECT_TRUE(ini.serialize().isApprox(expectedStates.serialize(), 1e-5));
}