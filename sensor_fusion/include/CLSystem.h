#pragma once

#include "Initializer.h"
#include "Robot.h"
#include "Estimator.h"

class CLSystem {
  public:
    CLSystem();

    void process(const measBasePtr& m);
    
  private:
    PositionInitializer positionIni_;
    Estimator estimator_;
    bool positionInitialized_; // The position of the UWB anchors are initialized.
    bool poseInitialized_;     // The pose of the moving UWB anchors are initialized.
    std::map<int, Eigen::Vector2d> anchorPositions_;
    std::set<Robot> robots_;
};