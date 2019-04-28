#include "Estimator.h"

#include "glog/logging.h"

#include <memory>
#include <assert.h>

Estimator::Estimator() {
}

void Estimator::process(const measBasePtr& m) {
 switch (m->type_) {
    case MeasurementType::IMU : {
      auto imuMeasPtr = std::dynamic_pointer_cast<ImuMeasurement>(m);
      // We do not process IMU data
      break;
    }
    case MeasurementType::WHEEL : {
      auto wheelMeasPtr = std::dynamic_pointer_cast<WheelMeasurement>(m);
      auto id = wheelMeasPtr->uwbId;
      double deltaSec = wheelMeasPtr->timeStamp - lastWheelMeas_[id]->timeStamp;
      // 1. ESKF predict 
      robots_[id].state_.propagate(lastWheelMeas_[id]->v, lastWheelMeas_[id]->omega,
                                    wheelMeasPtr->v, wheelMeasPtr->omega, deltaSec, false);

      statesPropagated_[id] = true;
      lastWheelMeas_[id] = wheelMeasPtr;
      break;
    }
    case MeasurementType::UWB : {
      auto uwbMeasPtr = std::dynamic_pointer_cast<UwbMeasurement>(m);

      // test
      int nRobot = robots_.size();
      int nAnchor = anchorPositions_.size();
      int nUwb = nRobot + anchorPositions_.size();
      int nRange = uwbMeasBuffer_.size() - nAnchor * (nAnchor - 1) / 2;
      assert(uwbMeasBuffer_.size() == nUwb * (nUwb - 1) / 2);
      // test end

      // Check if all robots' state propageted
      int nPropagated = 0;
      for (auto it = statesPropagated_.begin(); it != statesPropagated_.end(); ++it) {
        nPropagated += it->second;
      }
      if (nPropagated != nRobot) {
        uwbMeasBuffer_[uwbMeasPtr->uwbPair].push(uwbMeasPtr);
        break;
      }

      // All robots' states propageted, now it's time to update
      // 2. Construct observation stuff
      Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nRange, 3*nRobot);      
      Eigen::VectorXd y = Eigen::VectorXd::Zero(nRange);
      Eigen::VectorXd yPredict = Eigen::VectorXd::Zero(nRange);

      int indexH = 0;
      for (auto it = uwbMeasBuffer_.begin(); it != uwbMeasBuffer_.end(); ++it) {
        int uwbId1 = it->first.first;
        int uwbId2 = it->first.second;
        // This is a robot-robot range, robot-anchor range or anchor-anchor range?
        // e.g range pair [1,4], small index comes first
        auto itRobot1 = robots_.find(uwbId1);
        auto itRobot2 = robots_.find(uwbId2);
        if (itRobot1 == robots_.end() && itRobot2 == robots_.end()) 
          continue; // A anchor-anchor range, 4 is an anchor, we do not process
        
        double rangeSum = 0;
        for (auto& uwbMeasPtr : it->second) {
          rangeSum += uwbMeasPtr->range;
        }
        y(indexH) = rangeSum / it->second.size();
        it->second.clear();

        if ( itRobot1 != robots_.end()) { // Range is associated with a robot, 1 is a robot
          int index1 = std::distance(robots_.begin(), itRobot1); 
          if (itRobot2 != robots_.end()) { // A robot-robot range measurement, 4 is a robot
            int index2 = std::distance(robots_.begin(), itRobot2);
            Eigen::Vector2d dxy;
            dxy << robots_[uwbId1].state_.x_ - robots_[uwbId2].state_.x_, 
                   robots_[uwbId1].state_.y_ - robots_[uwbId2].state_.y_;
            yPredict(indexH) = dxy.norm();
            Eigen::Vector2d e = dxy.array() / yPredict(indexH);
            H.block<1,2>(indexH, 3*(index1-1)) = e;
            H.block<1,2>(indexH, 3*(index2-1)) = -e;
          } else { // A robot-anchor range measurement, 4 is an anchor
            auto itAnchor2 = anchorPositions_.find(uwbId2);
            assert(itAnchor2 != anchorPositions_.end());
 
            Eigen::Vector2d dxy;
            dxy << robots_[uwbId1].state_.x_ - anchorPositions_[uwbId2](0),
                   robots_[uwbId1].state_.x_ - anchorPositions_[uwbId2](1);
            yPredict(indexH) = dxy.norm();
            Eigen::Vector2d e = dxy.array() / yPredict(indexH);
            H.block<1,2>(indexH, 3*(index1-1)) = e;
          } 
        } else {
          // Range is associated with an anchor, 1 is an anchor
          auto itAnchor1 = anchorPositions_.find(uwbId1);
          assert(itAnchor1 != anchorPositions_.end());
          if (itRobot2 != robots_.end()) { // A anchor-robot range, 4 is a robot
            int index2 = std::distance(robots_.begin(), itRobot2);
            Eigen::Vector2d dxy;
            dxy << robots_[uwbId2].state_.x_ - anchorPositions_[uwbId1](0),
                   robots_[uwbId2].state_.y_ - anchorPositions_[uwbId1](1);
            yPredict(indexH) = dxy.norm();
            Eigen::Vector2d e = dxy.array() / yPredict(indexH);
            H.block<1,2>(indexH, 3*(index2-1)) = e;
          } 
        }

        ++indexH;
      }
      
      break;
    }
    default: {
      LOG(WARNING) << "Unknown measurement type: " << m->type_;
      break;
    }
  }
}
