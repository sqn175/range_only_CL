#include "Initializer.h"
#include "AncLocalization.h"

#include <iostream>

void PositionInitializer::AddMeasurement(const measBasePtr& m) {
  switch (m->type_) {
    case MeasurementType::IMU : {
      auto imuMeasPtr = std::dynamic_pointer_cast<ImuMeasurement>(m);
      // We do not process IMU data
      break;
    }
    case MeasurementType::WHEEL : {
      auto wheelMeasPtr = std::dynamic_pointer_cast<WheelMeasurement>(m);
      auto it = velEnergy_.find(wheelMeasPtr->anchorId_);
      if ( it == velEnergy_.end()) {
        // We insert a new Energy object which do not use default constructor
        velEnergy_[wheelMeasPtr->anchorId_] = Energy<double>(2);
      }
      velEnergy_[wheelMeasPtr->anchorId_].push_back(wheelMeasPtr->v_);
      break;
    }
    case MeasurementType::UWB : {
      auto uwbMeasPtr = std::dynamic_pointer_cast<UwbMeasurement>(m);
      meanRange_[uwbMeasPtr->anchorPair_].push_back(uwbMeasPtr->range_);
    }
    default: {
      // TODO: add log
      // std::cout << "Unkonwn measurement type: " << m->type_ << std::endl;
      break;
    }
  }
}

bool PositionInitializer::TryToInitialize() {
  // We must have enough ranging measurements for any pair of anchors
  bool rangeAllReady = true;  
  for (auto& item : meanRange_) {
    rangeAllReady &= item.second.N <= 100; // TODO: parameters
  }

  // Check if any anchor is moving
  bool anyMoving = false;
  for (auto& item : velEnergy_) {
    anyMoving |= item.second.energy() >= 0.1;
  }

  // If we have enough range measurements and any a anchor is moving,
  // We try to localize the moving anchors and the static anchors.
  if (rangeAllReady && !anyMoving) {
    // TODO: add log
    std::cout << "Time for the anchors to move straightforward!" << std::endl;
  } else if (!rangeAllReady && anyMoving) {
    // TODO: add log
    std::cout << "Anchors need to remain static before moving. Please RESTART the program!" << std::endl;
  } else if (rangeAllReady && anyMoving) {
    // We use the MDS to initialize the position of anchors and then optimize
    // using the ranging measurements.

    // The key of the meanRange_ appears int the order like:
    //  [01, 02, 03, 12, 13, 23], if we have 4 anchors whose ids are 0,1,2,3

    // The number of anchors are then determined by the number of pairs whose first 
    // element is 0, obviously the number of anchors is 4.
    int anchorCnt = 0;
    auto it = meanRange_.begin();
    int firstAnchorId = it->first.first;
    for (auto& item : meanRange_) {
      if (item.first.first == firstAnchorId)
        ++anchorCnt;
    }

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(anchorCnt, anchorCnt);
    for (int i = 0; i < anchorCnt; ++i) {
      for (int j = i+1; j < anchorCnt; ++j) {
        M(i,j) = it->second.mean();
        M(j,i) = M(i,j);
        it++;
      }
    }

    anchorPositions_ = MDS_Adam(M, 1e-5, 500);
    return true;
  }

  return false;
}

Eigen::MatrixXd PositionInitializer::GetPositions() {
  return anchorPositions_;
}