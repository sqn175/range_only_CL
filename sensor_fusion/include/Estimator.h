#pragma once

#include "Robot.h"
#include "Measurement.h"

#include "Eigen/Dense"

#include <map>
#include <memory>
#include <set>
#include <utility>

using NoiseParams = struct {
  double sigmaVel;
  double sigmaOmega;
  double sigmaRange;
  double sigmaHeading;
};

class Estimator {
  public:
    Estimator();

    void process(const measBasePtr& m);
    void init();

  private:
    std::map<int, Robot> robots_; 
    std::map<int, Eigen::Vector2d> anchorPositions_;

    // Error state Kalman filter (ESKF) stuff
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd Phi_;
    Eigen::MatrixXd R_;
    std::map<int, WheelMeasPtr> lastWheelMeas_;
    std::map<std::pair<int, int>, std::vector<UwbMeasPtr>> uwbMeasBuffer_;
    std::map<int, bool> statesPropagated_;  // States of Kalman filters
};