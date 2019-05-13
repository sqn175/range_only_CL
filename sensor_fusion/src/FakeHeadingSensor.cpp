#include "FakeHeadingSensor.h"

#include "glog/logging.h"

FakeHeadingSensor::FakeHeadingSensor(const std::set<int>& anchorIds,
                                    const std::set<int>& robotIds,
                                    double baseLineLength) {
  init(anchorIds, robotIds, baseLineLength);
}

void FakeHeadingSensor::init(const std::set<int>& anchorIds,
                              const std::set<int>& robotIds,
                              double baseLineLength) {
  anchorIds_ = anchorIds;
  robotIds_ = robotIds;
  baseLineLength_ = baseLineLength;

  windowLen_ = 10;
  uwbWindowLen_ = windowLen_ * 5; // TODO: parameters. 5 = uwb frequency / wheel encoder frequency
  for (auto& r : robotIds_) {
    velEnergy_[r] = Energy<double>(windowLen_);
    omegaEnergy_[r] = Energy<double>(windowLen_);

    rangeToAnchor0_[r] = std::deque<double>();
    isLinearMotion_[r] = false;
    isPrevLinearMotion_[r] = false;
  }
}

// If the robot is linearly moving, its angular speed is then
// near zero and its linear velocity is quite larger than zero.
// If they move linearly, they form a line which directions are 
// their orientations.
void FakeHeadingSensor::process(const measBasePtr& m) {
  switch(m->type) {
    case MeasurementType::WHEEL : {
      auto wheelMeasPtr = std::dynamic_pointer_cast<WheelMeasurement>(m);
      int id = wheelMeasPtr->uwbId;
      velEnergy_[id].push_back(wheelMeasPtr->v);
      omegaEnergy_[id].push_back(wheelMeasPtr->omega);

      if (omegaEnergy_[id].energy() < 1e-3 && velEnergy_[id].energy() > 0.03
          && omegaEnergy_[id].N() == windowLen_)  { // TODO: parameter
        isLinearMotion_[id] = true;
        // The robot is linearly moving
        if (!isPrevLinearMotion_[id]) { // The robot starts a linear motion
          LOG(INFO) << "Robot " << id << " detects a linear motion at " << m->timeStamp << "s.";
        } 
      } else {
        // if (isPrevLinearMotion_[id] || rangeToAnchor0_[id].size() >= 2000) {
        isLinearMotion_[id] = false;
        if (isPrevLinearMotion_[id]) {
          // The robots ends a linear motion, or lasts a linear motion for 20 seconds (20*100)
          // We now calculate its orientation using the ranges to the first two static anchors
          auto pose = EstimateHeading(id);

          LOG(INFO) << "Robot " << id << " ends linear motion at " << m->timeStamp << "s "
                    << "with orientation = " << pose[2]*180/M_PI << " degrees";

          // Erase the range buffer
          assert(rangeToAnchor0_[id].size() >= uwbWindowLen_);
          rangeToAnchor0_[id].erase(rangeToAnchor0_[id].begin(), 
                                    rangeToAnchor0_[id].begin() + rangeToAnchor0_[id].size() - uwbWindowLen_);
          rangeToAnchor1_[id].erase(rangeToAnchor1_[id].begin(), 
                                    rangeToAnchor1_[id].begin() + rangeToAnchor1_[id].size() - uwbWindowLen_);
          
          // Callback
          if (headingCallback_) {
            std::vector<double> res{m->timeStamp};
            res.insert(res.end(), 
                        std::make_move_iterator(pose.begin()), 
                        std::make_move_iterator(pose.end()));
            headingCallback_(id, res);
          }
        } 
      }
      isPrevLinearMotion_[id] = isLinearMotion_[id];
      break;
    }
    case MeasurementType::UWB : {
      auto uwbMeasPtr = std::dynamic_pointer_cast<UwbMeasurement>(m);
      // Find the robot-anchor range
      int uwbId1 = uwbMeasPtr->uwbPair.first;
      int uwbId2 = uwbMeasPtr->uwbPair.second;
      auto itRobot1 = robotIds_.find(uwbId1);
      auto itRobot2 = robotIds_.find(uwbId2);
      if (itRobot1 != robotIds_.end() && itRobot2 != robotIds_.end())
        break;  // ignore robot-robot range
      if (itRobot1 == robotIds_.end() && itRobot2 == robotIds_.end())
        break;  // ignore anchor-anchor range

      int indexAnchor;
      int robotId;
      if (itRobot1 != robotIds_.end()) { // uwbId1 is a robot, uwbId2 is an anchor
        auto itAnchor = anchorIds_.find(uwbId2);
        assert(itAnchor != anchorIds_.end());  // This is an anchor
        indexAnchor = std::distance(anchorIds_.begin(), itAnchor);
        robotId = uwbId1;
      } else { // uwbId2 is a robot, uwbId1 is an anchor
        auto itAnchor = anchorIds_.find(uwbId1);
        assert(itAnchor != anchorIds_.end());
        indexAnchor = std::distance(anchorIds_.begin(), itAnchor);
        robotId = uwbId2;
      }

      if (indexAnchor == 0) {
        rangeToAnchor0_[robotId].push_back(uwbMeasPtr->range);
        // The robot is not linearly moving, we then only keep the ranges in the window
        if (!isLinearMotion_[robotId] && rangeToAnchor0_.size() > uwbWindowLen_) {
          rangeToAnchor0_[robotId].pop_front();
        }
      } else if (indexAnchor == 1) {
        rangeToAnchor1_[robotId].push_back(uwbMeasPtr->range);
        if (!isLinearMotion_[robotId] && rangeToAnchor1_.size() > uwbWindowLen_) {
          rangeToAnchor1_[robotId].pop_front();
        }
      }
      break;
    }
    default: {
      break;
    }
  }
}


std::vector<double> FakeHeadingSensor::EstimateHeading(int robotId) {
  auto coeff = LinearFit(rangeToAnchor0_[robotId]);
  // x-axis is 1-based
  double range0Start = coeff[0] * 1 + coeff[1];
  double range0End = coeff[0] * rangeToAnchor0_[robotId].size() + coeff[1];

  coeff = LinearFit(rangeToAnchor1_[robotId]);
  double range1Start = coeff[0] * 1 + coeff[1];
  double range1End = coeff[0] * rangeToAnchor1_[robotId].size() + coeff[1];

  Eigen::Matrix<double, 2,2> r;
  r << range0Start, range1Start, range0End, range1End;

  return CalculateHeading(r);
}

std::vector<double> FakeHeadingSensor::LinearFit(const std::deque<double>& data) {
    double xSum = 0, ySum = 0, xxSum = 0, xySum = 0, slope, intercept;
    std::vector<double> xData;
    // x-axis is 1-based
    for (int i = 1; i < data.size()+1; ++i)
    {
        xData.push_back(static_cast<double>(i));
    }
    for (int i = 0; i < data.size(); ++i)
    {
        xSum += xData[i];
        ySum += data[i];
        xxSum += xData[i] * xData[i];
        xySum += xData[i] * data[i];
    }
    slope = (data.size() * xySum - xSum * ySum) / (data.size() * xxSum - xSum * xSum);
    intercept = (ySum - slope * xSum) / data.size();
    std::vector<double> res;
    res.push_back(slope);
    res.push_back(intercept);
    return res;
}

std::vector<double> FakeHeadingSensor::CalculateHeading(const Eigen::Matrix<double, 2,2>& d) {
  std::vector<double> res;
  res.reserve(3);

  // Start point position and end point position
  Eigen::Matrix<double, 2,2> pos;
  for (int i = 0; i < 2; ++i) {
    double cosAlpha = (d(i,0)*d(i,0) + baseLineLength_*baseLineLength_ - d(i,1)*d(i,1)) 
                      / (2 * baseLineLength_ * d(i,0));
    if (cosAlpha > 1) 
      cosAlpha = 1;
    else if (cosAlpha < -1) 
      cosAlpha = -1;
    
    double alpha = acos(cosAlpha);
    pos(i,0) = d(i,0)*cosAlpha;
    pos(i,1) = d(i,0)*sin(alpha);
  }
  double dy = pos(1,1) - pos(0,1);
  double dx = pos(1,0) - pos(0,0);
  double phi = atan2(dy, dx);
  res.push_back(pos(1,0));
  res.push_back(pos(1,1));
  res.push_back(phi);

  return res;
}

void FakeHeadingSensor::SetRobotHeadingCallback(const std::function<
          void (int robotId, std::vector<double> res)>& callback) {
  headingCallback_ = callback;
}