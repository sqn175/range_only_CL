#pragma once

#include "Robot.h"
#include "Measurement.h"

#include "Eigen/Dense"

#include <map>
#include <list>
#include <memory>
#include <set>
#include <queue>
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
    std::set<Robot> robots_;
    int nRobot_;

    // Error state Kalman filter stuff
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd Phi_;
    Eigen::MatrixXd R_;
    std::map<int, double> lastV_;
    std::map<int, double> lastOmega_;
    std::map<std::pair<int, int>, std::queue<double>> pastRanges_;
};