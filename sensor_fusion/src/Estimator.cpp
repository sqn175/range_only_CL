#include "Estimator.h"

#include <memory>
#include <assert.h>

Estimator::Estimator() 
  : positionInitialized_(false)
  , poseInitialized_(false){
}

void Estimator::Estimate() {
  // Retrive measurements

  // Check robot status
  // If not initialized
  if (!positionInitialized_) {
    for (auto& uwbList : uwbMeasurements_) {
      for (auto& uwbMeas : uwbList.second) {
        meanRange_[uwbList.first].push_back(uwbMeas->range_);
      }
    }
    // Check for movement
  }



  // If position initialized, but orientation not initialized

  // Check for linear movement

  // Initialized

  // Kalman filter
  // Update
  // Correct
}

bool Estimator::PositionInitializer(const measBasePtr& measurement) {

}