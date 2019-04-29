#pragma once

#include "Robot.h"
#include "Measurement.h"

#include "Eigen/Dense"

#include <map>
#include <memory>
#include <set>
#include <utility>

// Standard deviation of noises
using NoiseParams = struct {
  double sigmaV;
  double sigmaOmega;
  double sigmaRange;
  double sigmaHeading;
};

class Estimator {
  public:
    Estimator();

    void process(const measBasePtr& m);
    void init(const NoiseParams& noises, std::map<int, Robot> iniRobots, 
              std::map<int, Eigen::Vector2d> anchorPositions, double deltaSec);

  private:
    std::map<int, Robot> robots_; 
    std::map<int, Eigen::Vector2d> anchorPositions_;
    int nRobot_;
    int nAnchor_;
    int nUwb_;
    int nRange_;
    double deltaSec_;

    // Error state Kalman filter (ESKF) stuff
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd Phi_;
    Eigen::MatrixXd R_;
    Eigen::MatrixXd P_;
    std::map<int, WheelMeasPtr> lastWheelMeas_;
    std::map<std::pair<int, int>, std::vector<UwbMeasPtr>> uwbMeasBuffer_;
    std::map<int, bool> statesPropagated_;  // States of Kalman filters
};