#include "Initializer.h"
#include "AncLocalization.h"

#include "glog/logging.h"

#include <string>

bool PositionInitializer::process(const measBasePtr& m) {
  AddMeasurement(m);
  return TryToInitialize();
}

void PositionInitializer::AddMeasurement(const measBasePtr& m) {
  switch (m->type_) {
    case MeasurementType::IMU : {
      auto imuMeasPtr = std::dynamic_pointer_cast<ImuMeasurement>(m);
      // We do not process IMU data
      break;
    }
    case MeasurementType::WHEEL : {
      wheelMeasReceived_ = true;
      auto wheelMeasPtr = std::dynamic_pointer_cast<WheelMeasurement>(m);
      auto it = velEnergy_.find(wheelMeasPtr->uwbId_);
      if ( it == velEnergy_.end()) {
        // We insert a new Energy object which do not use default constructor
        velEnergy_[wheelMeasPtr->uwbId_] = Energy<double>(2);
        robotIds_.insert(wheelMeasPtr->uwbId_);
        LOG(INFO) << "Found a robot with id: " << wheelMeasPtr->uwbId_;
      }
      velEnergy_[wheelMeasPtr->uwbId_].push_back(wheelMeasPtr->v_);
      break;
    }
    case MeasurementType::UWB : {
      auto uwbMeasPtr = std::dynamic_pointer_cast<UwbMeasurement>(m);
      auto& idPair = uwbMeasPtr->uwbPair_;
      meanRange_[idPair].push_back(uwbMeasPtr->range_);
      if (uwbIds_.find(idPair.first) == uwbIds_.end()) {
        LOG(INFO) << "Found a UWB anchor with id: " << std::to_string(idPair.first);
        uwbIds_.insert(idPair.first);
      } 
      if (uwbIds_.find(idPair.second) == uwbIds_.end()) {
        LOG(INFO) << "Found a UWB anchor with id: " << std::to_string(idPair.second);
        uwbIds_.insert(idPair.second); 
      }

      LOG_EVERY_N(INFO, 1) << "[" << std::to_string(idPair.first) << "," 
            << std::to_string(idPair.second) << "]: " << uwbMeasPtr->range_ << "m";
      break;
    }
    default: {
      LOG(WARNING) << "Unknown measurement type: " << m->type_;
      break;
    }
  }
}

bool PositionInitializer::TryToInitialize() {
  if (!wheelMeasReceived_)
    return false;
  // We must have enough ranging measurements for any pair of anchors
  bool rangeAllReady = true;  
  for (auto& item : meanRange_) {
    rangeAllReady &= item.second.N >= 100; // TODO: parameters
  }

  // Check if any anchor is moving
  bool anyMoving = false;
  for (auto& item : velEnergy_) {
    anyMoving |= item.second.energy() >= 0.01*0.01; // TODO: parameters
  }

  // If we have enough range measurements and any a anchor is moving,
  // We try to localize the moving anchors and the static anchors.
  if (rangeAllReady && !anyMoving) {
    LOG_EVERY_N(INFO, 500) << "Wait for anchors moving in a straight line...";
  } else if (!rangeAllReady && anyMoving) {
    LOG(FATAL) << "Anchors need to remain static during initialization. Please RESTART the program!";
  } else if (rangeAllReady && anyMoving) {
    // Time to initialize the position of anchors and establish a coordinate

    // The key of the initial meanRange_ appears in the order like: [01, 02, 03, 12, 13, 23]
    // If we use the order to establish a coordinate using MDS_Adam method, 
    // the origin of the coordinate is set at 0, and x-axis along 1.
    // However, we need the origin to be at the first anchor, and x-axis along
    // the second anchor. Therefore, we need to rearrange the meanRange_ according 
    // to the ids of anchors. 
    // e.g. anchorIds_ = [1,4], uwbIds = [0,1,2,4], robotIds are then [0,2]
    // we need to rearrange meanRange_ in the order like: [14, 01, 12, 04, 24, 02],
    // which still follows the rule of the first id is less.
    std::set_difference(uwbIds_.begin(), uwbIds_.end(), 
                        robotIds_.begin(), robotIds_.end(), 
                        std::inserter(anchorIds_, anchorIds_.begin()));

    std::vector<std::pair<int, int>> idPairs;
    std::vector<double> ranges;

    for (auto i = anchorIds_.begin(); i != anchorIds_.end(); ++i) {
      for (auto j = std::next(i,1); j != anchorIds_.end(); ++j) {
        auto idPair = (*i < *j) ? std::make_pair(*i, *j) : std::make_pair(*j, *i);
        idPairs.push_back(idPair);
        ranges.push_back(meanRange_[idPair].mean());
      }
      for (auto k = robotIds_.begin(); k != robotIds_.end(); ++k) {
        auto idPair = (*i < *k) ? std::make_pair(*i, *k) : std::make_pair(*k, *i);
        idPairs.push_back(idPair);
        ranges.push_back(meanRange_[idPair].mean());
      }
    }
    for (auto i = robotIds_.begin(); i != robotIds_.end(); ++i) {
      for (auto j = std::next(i,1); j != robotIds_.end(); ++j) {
        auto idPair = (*i < *j) ? std::make_pair(*i, *j) : std::make_pair(*j, *i);
        idPairs.push_back(idPair);
        ranges.push_back(meanRange_[idPair].mean());
      }
    }

    // Construct the adjacent matrix of range
    int nUwb = uwbIds_.size();
    auto it = ranges.begin();
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(nUwb, nUwb);
    for (int i = 0; i < nUwb-1; ++i) {
      for (int j = i+1; j < nUwb; ++j) {
        M(i,j) = *it++;
        M(j,i) = M(i,j);
      }
    }

    LOG(INFO) << "Adjacent matrix of range: \n" << M;
    uwbPositions_ = MDS_Adam(M, 2, 1e-5);
    
    // TODO: check why we get negative y
    for (int i = 0; i < nUwb; ++i) {
      uwbPositions_(i,1) = uwbPositions_(i,1) < 0 ? -uwbPositions_(i,1) : uwbPositions_(i,1);
    }

    wheelMeasReceived_ = false;
    return true;
  }
  wheelMeasReceived_ = false;
  return false;
}

std::map<int, Eigen::Vector2d> PositionInitializer::GetUwbPositions() {
  std::map<int, Eigen::Vector2d> res;
  if (uwbPositions_.data()) {
    assert(uwbIds_.size() == uwbPositions_.rows());
    auto idIter = uwbIds_.begin();
    int i = 0;
    for ( ; i < uwbIds_.size(); ++i) {
      Eigen::Vector2d xy;
      xy << uwbPositions_(i,0), uwbPositions_(i,1);
      res[*idIter++] = xy; 
    }
  }
  return res;
}

std::map<int, Eigen::Vector2d> PositionInitializer::GetAnchorPositions() {
  std::map<int, Eigen::Vector2d> res;
  if (uwbPositions_.data()) {
    assert(uwbIds_.size() == uwbPositions_.rows());
    auto idIter = anchorIds_.begin();
    int i = 0;
    for ( ; i < anchorIds_.size(); ++i) {
      Eigen::Vector2d xy;
      xy << uwbPositions_(i,0), uwbPositions_(i,1);
      res[*idIter++] = xy;
    }
  }
  return res;
}

std::map<int, Eigen::Vector2d> PositionInitializer::GetRobotPositions() {
  std::map<int, Eigen::Vector2d> res;
  if (uwbPositions_.data()) {
    assert(uwbIds_.size() == uwbPositions_.rows());
    auto idIter = anchorIds_.begin();
    int i = anchorIds_.size();
    for ( ; i < uwbIds_.size(); ++i) {
      Eigen::Vector2d xy;
      xy << uwbPositions_(i,0), uwbPositions_(i,1);
      res[*idIter++] = xy;
    }
  }
  return res;
}
