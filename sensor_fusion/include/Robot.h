#pragma once

#include "States.h"
#include "EnergyDetector.h"

class Robot {
  public: 
    Robot();
    Robot(int id, double x, double y);
    Robot(int id, double x, double y, double phi);

    void setOrientation(double phi);
    int id() const;

  public:
    States state_;

  private:
    int id_;
    Energy<double> vEnergy_;
    Energy<double> omegaEnergy_;
};

inline bool operator < (const Robot& lhs, const Robot& rhs) {
  return lhs.id() < rhs.id();
}
