#pragma once

#include "States.h"
#include "EnergyDetector.h"

class Robot {
  public: 
    Robot();
    Robot(int id, double x, double y);
    Robot(int id, double x, double y, double phi);

    void SetState(double x, double y, double phi);
    void SetOrientation(double phi);
    void SetTimestamp(double t);
    int id() const;
    double t() const;

  public:
    States state_;

  private:
    int id_;  // Robot id
    double timeStamp_;  // timestamp of the incoming wheel encoder measurement
    // Energy<double> vEnergy_;
    // Energy<double> omegaEnergy_;
};

inline bool operator < (const Robot& lhs, const Robot& rhs) {
  return lhs.id() < rhs.id();
}
