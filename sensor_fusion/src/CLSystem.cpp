#include "CLSystem.h"

#include "glog/logging.h"

CLSystem::CLSystem()
  : positionInitialized_(false)
  , poseInitialized_(false){
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
        robots_.insert(std::move(Robot(r.first, xy(0), xy(1))));
      }
      LOG(INFO) << "Anchor position initialization completed!";
    }
  } else if (!poseInitialized_) {
     // If position initialized, but orientation not initialized
     estimator_.init();
  } else {
    // ESKF update and correct
    estimator_.process(m);
  }
}