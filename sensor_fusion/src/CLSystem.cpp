#include "CLSystem.h"

#include "glog/logging.h"

CLSystem::CLSystem(const NoiseParams& noise, double deltaSec)
  : positionInitialized_(false)
  , poseInitialized_(false)
  , noise_(noise)
  , deltaSec_(deltaSec){
}

void CLSystem::process(const measBasePtr& m) {
  // Check robot status
  // If not initialized
  if ( !positionInitialized_ ) {
    if ( positionIni_.process(m) ) {
      positionInitialized_ = true;
      anchorPositions_ = positionIni_.GetAnchorPositions();
      auto robotPositions = positionIni_.GetRobotPositions();
      for (auto& r : robotPositions) {
        auto xy = r.second;
        robots_[r.first] = Robot(r.first, xy(0), xy(1));
      }
      LOG(INFO) << "Anchor position initialization completed!";

      if (iniStatesCallback_)
        iniStatesCallback_(robots_, anchorPositions_);
    }
  } else if (!poseInitialized_) {
     // If position initialized, but orientation not initialized
     // Fake pose initializer
     if (positionInitialized_) {
       poseInitialized_ = true;
       std::map<int, double> fakePhi;
       fakePhi[1] = -2.971;
       fakePhi[4] = -3.079;
       for (auto& r : robots_) {
         r.second.setOrientation(fakePhi[r.first]);
       }

       // Initialize estimator
       estimator_.init(noise_, robots_, anchorPositions_, deltaSec_);
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
