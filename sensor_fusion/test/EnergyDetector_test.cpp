#include "EnergyDetector.h"

#include "gtest/gtest.h"

TEST(EnergyTest, push) {
  Energy<double> e(20);
  e.push_back(2.2);
  e.push_back(2);
  ASSERT_EQ(e.N(), 2);

  for (int i = 0; i < 20; ++i) {
    e.push_back(0.1);
  }
  ASSERT_EQ(e.N(), 20);
}

TEST(EnergyTest, energy) {
  Energy<double> e(2);
  e.push_back(1.0);
  e.push_back(1.0);
  ASSERT_EQ(e.energy(), 1);

  e.push_back(1.0);
  ASSERT_EQ(e.energy(), 1);
}