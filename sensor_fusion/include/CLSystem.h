#pragma once

#include "PositionInitializer.h"
#include "FakeHeadingSensor.h"
#include "Robot.h"
#include "Estimator.h"
#include "ThreadSafeQueue.h"

// Estimation results for a robot
struct PoseResults {
  int id;    // uwb id
  float x;   // meter
  float y;   // meter
  float yaw; // radius
};

class CLSystem {
/*
  This class manages the complete data flow in and out of the algorithm.

  To ensure fast return from user callbacks, new data are collected in thread safe input queue
  and are processed in consumer thread. 
  The algorithm task are running in a individual thread with a thread safe queue for message passing.
  For sending back data to the user, a publisher thread are created which ensure that the algorithm 
  is not blocked by slow users.
*/

  public:
    CLSystem(const NoiseParams& noise, double deltaSec);
    ~CLSystem();

    // Add a new measurement
    bool AddMeasurement(const measBasePtr& m);

    // Set the iniStatesCallback_ to be called when the position of anchors and the initial 
    // positions of all robots are estimated.
    void SetIniStatesCallback(const std::function<void (std::map<int, Robot>, 
                                                        std::map<int, Eigen::Vector2d> 
                                                       )>& callback);

    // Set the statesCallback_ to be called every time the states of all robots updated.
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