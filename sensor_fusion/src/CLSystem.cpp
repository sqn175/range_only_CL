#include "CLSystem.h"

#include "glog/logging.h"

CLSystem::CLSystem(const NoiseParams& noise, double deltaSec)
  : positionInitialized_(false)
  , noise_(noise)
  , deltaSec_(deltaSec){
}

void CLSystem::process(const measBasePtr& m) {
  // Check robot status
  // If not initialized
  // TODO: use callback for position initialization completion

  if ( !positionInitialized_ ) {
    if ( positionIni_.process(m) ) {
      positionInitialized_ = true;
      anchorPositions_ = positionIni_.GetAnchorPositions();
      auto robotPositions = positionIni_.GetRobotPositions();
      for (auto& r : robotPositions) {
        auto xy = r.second;
        robots_[r.first] = Robot(r.first, xy(0), xy(1));
        poseInitialized_[r.first] = false;
      }
      LOG(INFO) << "Anchor position initialization completed!";

      if (iniStatesCallback_)
        iniStatesCallback_(robots_, anchorPositions_);

      positionInitialized_ = true;
      // Calculate baseline length
      auto it = anchorPositions_.begin();
      Eigen::Vector2d firstAnc = it->second;
      it++;
      Eigen::Vector2d secondAnc = it->second;

      double baseLineLength = (firstAnc - secondAnc).norm();

      // Initialize the FakeHeadingSensor
      fakeHeadingSensor_.init(positionIni_.GetAnchorIds(), positionIni_.GetRobotIds(), baseLineLength);
      // Initialize estimator
      estimator_.init(anchorPositions_, noise_, deltaSec_);
    }
  } else {
    // ESKF update and correct
    estimator_.process(m);
  }

}

void CLSystem::SetRobotStatesCallback(const std::function<void (std::map<int, Robot> robots)>& callback) {
  estimator_.SetRobotStatesCallback(callback);
}
    
void CLSystem::SetIniStatesCallback(const std::function<void (std::map<int, Robot>, 
                                    std::map<int, Eigen::Vector2d> )>& callback) {
  iniStatesCallback_ = callback;
}

void CLSystem::OnHeading(int robotId, std::vector<double> xyphi) {
  if (!poseInitialized_[robotId]) {
    poseInitialized_[robotId] = true;
    robots_[robotId].SetState(xyphi[0], xyphi[1], xyphi[2]);
    estimator_.AddRobot(robots_[robotId]);
  } else {
    auto m = std::make_shared<HeadingMeasurement>(0.0, robotId, xyphi[2]);
    estimator_.process(m);
  }
}