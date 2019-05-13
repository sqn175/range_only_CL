#pragma once

#include "PositionInitializer.h"
#include "FakeHeadingSensor.h"
#include "Robot.h"
#include "Estimator.h"
#include "ThreadSafeQueue.h"

struct PoseResults {
  int id;
  float x;
  float y;
  float yaw;
};

class CLSystem {
  public:
    CLSystem(const NoiseParams& noise, double deltaSec);
    ~CLSystem();

    bool AddMeasurement(const measBasePtr& m);

    // Callbacks are not thread-safe
    void SetIniStatesCallback(const std::function<void (std::map<int, Robot>, 
                                                        std::map<int, Eigen::Vector2d> 
                                                       )>& callback);

    void SetStatesCallback(const std::function<void (std::vector<PoseResults>)>& callback);
    
    // noncopyable
    CLSystem(const CLSystem& that) = delete;
    CLSystem& operator=(const CLSystem& that) = delete;
  private:
    // Loop to process incoming measurements
    void ConsumerLoop();
    // Loop to publish estimation results
    void PublisherLoop();

  private:
    void process(const measBasePtr& m);
    // res: t, x, y, phi
    void OnHeading(int robotId, std::vector<double> res);
    void OnStatesEstimated(std::map<int, Robot> robots);
  private:
    PositionInitializer positionIni_;
    FakeHeadingSensor fakeHeadingSensor_;
    Estimator estimator_;
    
    bool positionInitialized_; // The position of the UWB anchors are initialized.
    std::map<int, bool> poseInitialized_;     // The pose of the moving UWB anchors are initialized.
    std::map<int, Eigen::Vector2d> anchorPositions_;
    std::map<int, Robot> robots_;
    NoiseParams noise_;
    double deltaSec_;

    // Initialization completion callback
    std::function<
      void (std::map<int, Robot>, std::map<int, Eigen::Vector2d>)> iniStatesCallback_;

    // Estimation completion callback
    std::function<void (std::vector<PoseResults>)> statesCallback_;

    // Localization results queue that is ready for publishing
    ThreadSafeQueue<std::vector<PoseResults>> estimationResults_;
    // Measurement input queue
    ThreadSafeQueue<measBasePtr> measurements_;


    // Estimation main thread
    std::thread consumerThread_;
    // Publisher thread
    std::thread publisherThread_;
};