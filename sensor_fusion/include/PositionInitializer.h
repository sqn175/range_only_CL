#pragma once

#include "Measurement.h"
#include "EnergyDetector.h"

#include "Eigen/Dense"

#include <map>
#include <list>
#include <set>

class PositionInitializer {
  public:
    PositionInitializer() {
    }
    
    bool process(const measBasePtr& measurement);
    std::map<int, Eigen::Vector2d> GetUwbPositions() const;
    std::map<int, Eigen::Vector2d> GetAnchorPositions() const;
    std::map<int, Eigen::Vector2d> GetRobotPositions() const;

    std::set<int> GetRobotIds() const;
    std::set<int> GetAnchorIds() const;

  private:
    void AddMeasurement(const measBasePtr& measurement);
    bool TryToInitialize();

  private:
    std::map<int, Energy<double>> velEnergy_;
    // The range between each pair of anchors, used for initial position estimation
    std::map<std::pair<int, int>, Accumulator<double>> meanRange_;

    // If a new wheel measurement is received, we then have a chance to 
    // trigger the MDS_Adam function to locate the position of anchors.
    bool wheelMeasReceived_;

    // The initialized uwb positions
    // example: [x0 y0 z0
    //           x2 y2 z2]
    // is the position of uwbIds = 0,2
    Eigen::MatrixXd uwbPositions_;
    // uwbIds_ = robotIds_ + anchorIds_;
    std::set<int> uwbIds_;  
    // The ids of the moving UWBs equipped on a Robot
    std::set<int> robotIds_;
    std::set<int> anchorIds_;
};

