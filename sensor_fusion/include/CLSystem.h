#pragma once

#include "PositionInitializer.h"
#include "FakeHeadingSensor.h"
#include "Robot.h"
#include "Estimator.h"

class CLSystem {
  public:
    CLSystem(const NoiseParams& noise, double deltaSec);

    void process(const measBasePtr& m);

    // Callbacks are not thread-safe
    void SetIniStatesCallback(const std::function<void (std::map<int, Robot>, 
                                                        std::map<int, Eigen::Vector2d> 
                                                       )>& callback);

    void SetRobotStatesCallback(const std::function<void (std::map<int, Robot> robots)>& callback);
    
  private:
    void OnHeading(int robotId, std::vector<double> xyphi);
  private:
    PositionInitializer positionIni_;
    FakeHeadingSensor fakeHeadingSensor_;
    Estimator estimator_;
    bool positionInitialized_; // The position of the UWB anchors are initialized.
    std::map<int, bool> poseInitialized_;     // The pose of the moving UWB anchors are initialized.
    std::map<int, Eigen::Vector2d> anchorPositions_;
    std::map<int, Robot> robots_;
    NoiseParams noise_;
    double deltaSec_;

    std::function<
      void (std::map<int, Robot>, std::map<int, Eigen::Vector2d>)> iniStatesCallback_;
};