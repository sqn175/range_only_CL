#pragma once

#include "States.h"
#include "EnergyDetector.h"

#include <Eigen/Dense>

#include <map>
#include <list>
#include <set>
#include <memory>

class Estimator {
  public:
    Estimator();

    void AddMeasurement(const measBasePtr& measurement);
    void Estimate();

    bool PositionInitializer(const measBasePtr& measurement);

  private:
    bool positionInitialized_; // The position of the UWB anchors are initialized.
    bool poseInitialized_;     // The pose of the moving UWB anchors are initialized.
    
    std::set<EnergyDetector> energyDetectors_; 

    std::map<int, States> states_;


}