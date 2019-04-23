#pragma once

#include "Measurement.h"
#include "EnergyDetector.h"

#include <Eigen/Dense>

#include <map>
#include <list>

class PositionInitializer {
  public:
    PositionInitializer() {
    }

    void AddMeasurement(const measBasePtr& measurement);
    bool TryToInitialize();
    Eigen::MatrixXd GetPositions();

  private:
    std::map<int, Energy<double>> velEnergy_;
    Eigen::MatrixXd anchorPositions_;
    // The range between each pair of anchors, used for initial position estimation
    std::map<std::pair<int, int>, Accumulator<double>> meanRange_;
};

// https://www.johndcook.com/blog/standard_deviation/
template <typename T, typename T2=T>
struct Accumulator
{
    T2 sum; // we could plug in a more accurate type for the sum
    T S;    
    T M;
    size_t N;  // The elements we have accumulated
    size_t maxN; // Maximum elements we can accumulate, if exceeds, we do not accumulate
    
    bool is_empty;
    T min;
    T max;

    // default constructor initializes all values
    Accumulator() : sum(0), S(0), M(0), N(0), maxN(2147483647), is_empty(true) { }
 
    // add another number
    T2 push_back(const T& x)
    {
      ++N;
      if (N >= maxN) {
        // TODO: add log
        std::cout << "Accumulator exceeds its limits, i.e. the robots need to move";
        return sum;
      }
      sum += x;
      T Mprev = M;
      M += (x - Mprev) / N;
      S += (x - Mprev) * (x - M);
      if (is_empty) {
        min = x;
        max = x;
        is_empty = false;
      } else {
        if (x < min) {
          min = x;
        } else if (x > max) {
          max = x;
        }
      }
      return sum;
    }
 
    T mean() const
    {
        return sum / N;
    }
 
    T variance() const
    {
        return S / (N - 1);
    }
 
    friend std::ostream& operator<<(std::ostream& out,
            const Accumulator& a)
    {
        out << " N         = " << a.N << std::endl
            << " sum       = " << a.sum << std::endl
            << " mean      = " << a.mean() << std::endl
            << " variance  = " << a.variance() << std::endl
			<< " min       = " << a.min << std::endl
			<< " max       = " << a.max << std::endl;
        return out;
    }
 
};
