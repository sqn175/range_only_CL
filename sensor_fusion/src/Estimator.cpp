#include "Estimator.h"

#include "glog/logging.h"

#include <memory>
#include <assert.h>

Estimator::Estimator() {
}

void Estimator::init(const NoiseParams& noises, std::map<int, Robot> iniRobots, 
              std::map<int, Eigen::Vector2d> anchorPositions, double deltaSec) {
  robots_ = std::move(iniRobots);
  anchorPositions_ = std::move(anchorPositions);

  nRobot_ = robots_.size();
  nAnchor_ = anchorPositions_.size();
  nUwb_ = nRobot_ + anchorPositions_.size();
  nRange_ = nUwb_ * (nUwb_ - 1) / 2 - nAnchor_ * (nAnchor_ - 1) / 2; // Ignore anchor-anchor range measurements

  P_ = Eigen::MatrixXd::Identity(3*nRobot_, 3*nRobot_);
  Eigen::Vector3d singleQ;
  singleQ << noises.sigmaV * noises.sigmaV,
             noises.sigmaV * noises.sigmaV,
             noises.sigmaOmega * noises.sigmaOmega;
  Q_ = (singleQ.replicate(nRobot_, 1)).asDiagonal();
  R_ = (Eigen::VectorXd::Ones(nRange_) * noises.sigmaRange * noises.sigmaRange).asDiagonal();
  Phi_ = Eigen::MatrixXd::Zero(3*nRobot_, 3*nRobot_);
  for (int i = 0; i < nRobot_; ++i) {
    Phi_.block<3,3>(3*i, 3*i) = Eigen::MatrixXd::Identity(3,3); // TODO: this is an eye matrix
  }

  deltaSec_ = deltaSec;
}

void Estimator::process(const measBasePtr& m) {
 switch (m->type_) {
    case MeasurementType::IMU : {
      // auto imuMeasPtr = std::dynamic_pointer_cast<ImuMeasurement>(m);
      // We do not process IMU data
      break;
    }
    case MeasurementType::WHEEL : {
      auto wheelMeasPtr = std::dynamic_pointer_cast<WheelMeasurement>(m);
      auto id = wheelMeasPtr->uwbId;

      if (lastWheelMeas_[id]) {// Not null 
        double deltaSec = wheelMeasPtr->timeStamp - lastWheelMeas_[id]->timeStamp;

        if (deltaSec > 30) // 30 seconds
          LOG(WARNING) << "Wheel encoder measurements lost for 30 seconds!";

        if (abs(wheelMeasPtr->v) > 1e6) { // Outlier
          LOG(WARNING) << "Outlier wheel encoder velocity measurement: " << wheelMeasPtr->v
                       << " from id = " << std::to_string(wheelMeasPtr->uwbId);
          wheelMeasPtr->v = lastWheelMeas_[id]->v;  // Simply set its value as last velocity
        }

        if (abs(wheelMeasPtr->omega) > 1e6 ) {
          LOG(WARNING) << "Outlier wheel encoder angular velocity measurement: " << wheelMeasPtr->omega
                       << " from id = " << std::to_string(wheelMeasPtr->uwbId);
          wheelMeasPtr->omega = lastWheelMeas_[id]->omega;
        }

        // 1. ESKF predict 
        // 1.1 State propagation
        robots_[id].state_.propagate(lastWheelMeas_[id]->v, lastWheelMeas_[id]->omega,
                                     wheelMeasPtr->v, wheelMeasPtr->omega, deltaSec, false);
        
        statesPropagated_[id] = true;
      }
      lastWheelMeas_[id] = wheelMeasPtr;

      break;
    }
    case MeasurementType::UWB : {
      auto uwbMeasPtr = std::dynamic_pointer_cast<UwbMeasurement>(m);

      double deltaSec = 0;

      if (!uwbMeasBuffer_[uwbMeasPtr->uwbPair].empty())
        deltaSec = uwbMeasPtr->timeStamp - uwbMeasBuffer_[uwbMeasPtr->uwbPair].back()->timeStamp;
      if (deltaSec > 10) // 10 seconds
        LOG(WARNING) << "UWB range measurements lost for 10 seconds!";

      if (uwbMeasPtr->range > 1e5) { // Outlier
        LOG(WARNING) << "Outlier UWB range measurement: " << uwbMeasPtr->range << " from pair: ["
                     << uwbMeasPtr->uwbPair.first << "," << uwbMeasPtr->uwbPair.second << "]";
        if (!uwbMeasBuffer_[uwbMeasPtr->uwbPair].empty())
          uwbMeasPtr->range = uwbMeasBuffer_[uwbMeasPtr->uwbPair].back()->range;
        else
          LOG(ERROR) << "The UWB range measurement is a outlier and cannot be rejected.";
      }

      // Append the buffer
      uwbMeasBuffer_[uwbMeasPtr->uwbPair].push_back(uwbMeasPtr);

      // Check if all robots' state propageted
      int nPropagated = 0;
      for (auto it = statesPropagated_.begin(); it != statesPropagated_.end(); ++it) {
        nPropagated += it->second;
      }
      if (nPropagated != nRobot_) {
        break; 
      }

      KalmanCorrect();

      // Clear the buffer
      for (auto& item : uwbMeasBuffer_) {
        item.second.clear();
      }

      if (robotStatesCallback_) {
        // TODO: Not thread-safe
        robotStatesCallback_(robots_);
      }
      break;
    }
    default: {
      LOG(WARNING) << "Unknown measurement type: " << m->type_;
      break;
    }
  }
}

void Estimator::SetRobotStatesCallback(const std::function<void (std::map<int, Robot> robots)>& callback) {
  robotStatesCallback_ = callback;
}

void Estimator::KalmanCorrect() {
  // TODO: how to calculate P_
  // 1.2 error covariance propagation
  P_ = Phi_ * P_ * Phi_.transpose() + deltaSec_*deltaSec_*Q_;

  assert(uwbMeasBuffer_.size() == nUwb_ * (nUwb_ - 1) / 2);
  // All robots' states propageted, now it's time to update
  // 2. ESKF update
  // 2.1 Construct observation stuff
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nRange_, 3*nRobot_);      
  Eigen::VectorXd y = Eigen::VectorXd::Zero(nRange_);
  Eigen::VectorXd yPredict = Eigen::VectorXd::Zero(nRange_);

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
    assert(rangeSum != 0);

    y(indexH) = rangeSum / it->second.size();

    if ( itRobot1 != robots_.end()) { // Range is associated with a robot, 1 is a robot
      int index1 = std::distance(robots_.begin(), itRobot1); 
      if (itRobot2 != robots_.end()) { // A robot-robot range measurement, 4 is a robot
        int index2 = std::distance(robots_.begin(), itRobot2);
        Eigen::Vector2d dxy;
        dxy << robots_[uwbId1].state_.x_ - robots_[uwbId2].state_.x_, 
                robots_[uwbId1].state_.y_ - robots_[uwbId2].state_.y_;
        yPredict(indexH) = dxy.norm();
        Eigen::Vector2d e = dxy.array() / yPredict(indexH);
        H.block<1,2>(indexH, 3*index1) = e.transpose();
        H.block<1,2>(indexH, 3*index2) = -e.transpose();
      } else { // A robot-anchor range measurement, 4 is an anchor
        auto itAnchor2 = anchorPositions_.find(uwbId2);
        assert(itAnchor2 != anchorPositions_.end());

        Eigen::Vector2d dxy;
        dxy << robots_[uwbId1].state_.x_ - anchorPositions_[uwbId2](0),
                robots_[uwbId1].state_.x_ - anchorPositions_[uwbId2](1);
        yPredict(indexH) = dxy.norm();
        Eigen::Vector2d e = dxy.array() / yPredict(indexH);
        H.block<1,2>(indexH, 3*index1) = e.transpose();
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
        H.block<1,2>(indexH, 3*index2) = e.transpose();
      } 
    }

    ++indexH;
  }

  // 2.2. Kalman gain
  Eigen::MatrixXd K = P_ * H.transpose() * (H*P_*H.transpose() + R_).inverse();

  Eigen::VectorXd deltaX = K * (y - yPredict);

  // 2.3 Correct
  int rindex = 0;
  for (auto& it : robots_) 
    it.second.state_.correct(deltaX.segment<3>(rindex++));

  P_ = (Eigen::MatrixXd::Identity(3*nRobot_, 3*nRobot_) - K * H) * P_ 
        * (Eigen::MatrixXd::Identity(3*nRobot_, 3*nRobot_) - K * H).transpose() 
        + K * R_ * K.transpose();

  // Reset for states update
  for (auto& s : statesPropagated_) {
    s.second = false;
  }
}