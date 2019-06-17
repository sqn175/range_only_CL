#include "CLSystem.h"

#include "glog/logging.h"

CLSystem::CLSystem(const NoiseParams& noise, double deltaSec)
  : positionInitialized_(false)
  , noise_(noise)
  , deltaSec_(deltaSec){

  estimator_.SetRobotStatesCallback(     
              std::bind(&CLSystem::OnStatesEstimated, this,
              std::placeholders::_1));

  // Starts internal threads
  consumerThread_ = std::thread(&CLSystem::ConsumerLoop, this);
  publisherThread_ = std::thread(&CLSystem::PublisherLoop, this);
}

CLSystem::~CLSystem() {
  // Shut down queues
  measurements_.ShutDown();
  estimationResults_.ShutDown();

  // Destroy threads
  consumerThread_.join();
  publisherThread_.join();

  // Clear callbacks
  std::function<
      void (std::map<int, Robot>, std::map<int, Eigen::Vector2d>)> empty;
  iniStatesCallback_.swap(empty);
}

bool CLSystem::AddMeasurement(const measBasePtr& m) {
  return measurements_.PushBlockingIfFull(m, 1);
}

void CLSystem::ConsumerLoop() {
  measBasePtr m;
  for (;;) {
    if (measurements_.PopBlocking(m) == false) {
      return;
    }
    process(m);
  }
}

void CLSystem::PublisherLoop() {
  std::vector<PoseResults> res;
  for (;;) {
    if (estimationResults_.PopBlocking(res) == false) {
      return;
    }
    if (statesCallback_) 
      statesCallback_(res);
  }
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
      LOG(INFO) << "Anchor position initialization completed at " << m->timeStamp << " s.";
      
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
      fakeHeadingSensor_.SetRobotHeadingCallback(
              std::bind(&CLSystem::OnHeading, this,
              std::placeholders::_1, std::placeholders::_2));
      // Initialize estimator
      estimator_.init(anchorPositions_, noise_, deltaSec_);
    }
  } else {
    fakeHeadingSensor_.process(m);
    estimator_.process(m);
  }
}

void CLSystem::OnHeading(int robotId, std::vector<double> res) {
  if (!poseInitialized_[robotId]) {
    poseInitialized_[robotId] = true;
    robots_[robotId].SetState(res[1], res[2], res[3]);
    if (iniStatesCallback_)
      iniStatesCallback_(robots_, anchorPositions_);

    estimator_.AddRobot(robots_[robotId]);
  } else {
    auto m = std::make_shared<HeadingMeasurement>(0.0, robotId, res[0]);
    //estimator_.process(m);
  }
}

void CLSystem::OnStatesEstimated(std::map<int, Robot> robots) {
  std::vector<PoseResults> res;
  for (auto& r : robots) {
    PoseResults pose;
    pose.timeStamp = r.second.t();
    pose.id = r.first;
    pose.x = r.second.state_.x_;
    pose.y = r.second.state_.y_;
    pose.yaw = r.second.state_.phi_;
    res.push_back(pose);
  }
  estimationResults_.PushNonBlocking(res);
}

void CLSystem::SetStatesCallback(const std::function<void (std::vector<PoseResults>)>& callback) {
  statesCallback_ = callback;
}
    
void CLSystem::SetIniStatesCallback(const std::function<void (std::map<int, Robot>, 
                                    std::map<int, Eigen::Vector2d> )>& callback) {
  iniStatesCallback_ = callback;
}
