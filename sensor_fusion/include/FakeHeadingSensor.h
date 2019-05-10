#pragma once

#include "Measurement.h"
#include "EnergyDetector.h"

#include <map>
#include <set>
#include <deque>

class FakeHeadingSensor {
  public:
    FakeHeadingSensor() {
    }
    FakeHeadingSensor(const std::set<int>& anchorIds,
                      const std::set<int>& robotIds,
                      double baseLineLength);
    void init(const std::set<int>& anchorIds,
              const std::set<int>& robotIds,
              double baseLineLength);
    void process(const measBasePtr& measurement);
    void SetRobotHeadingCallback(const std::function<void (int robotId, std::vector<double> xyphi)>& callback);
  public:
    std::vector<double> EstimateHeading(int robotId);
    std::vector<double> LinearFit(const std::deque<double>& data);
    // Return orientation in radius
    std::vector<double> CalculateHeading(const Eigen::Matrix<double, 2,2>& range);
    
    std::map<int, Energy<double>> velEnergy_;
    std::map<int, Energy<double>> omegaEnergy_;
    int windowLen_;

    std::map<int, std::deque<double>> rangeToAnchor0_;
    std::map<int, std::deque<double>> rangeToAnchor1_;

    std::set<int> anchorIds_;
    std::set<int> robotIds_;
    double baseLineLength_;

    std::map<int, bool> isLinearMotion_;
    std::map<int, bool> isPrevLinearMotion_;

    // Heading estimation callback
    std::function<void (int robotId, std::vector<double> xyphi)> headingCallback_;
};