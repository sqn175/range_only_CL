#pragma once

#include <queue>

template<typename T, typename T2=T>
class Energy {
  public:
    // Constructor
    Energy()
      : squareSum_(0), windowLen_(40), N_(0) {
    }
    Energy(size_t windowLen_) 
      : squareSum_(0), windowLen_(windowLen_), N_(0) {
    }

    T2 push_back(const T& x) {
      buf_.push(x);
      if (N_ < windowLen_) 
        ++N_;

      squareSum_ += x*x;
      T& first = buf_.front();
      squareSum_ -= first*first;
      buf_.pop();

      return squareSum_;
    }

    T energy() const {
      return squareSum_ / N_;
    }

    void clear() {
      squareSum_ = 0;
      N_ = 0;
      // Clear the queue
      std::queue<T> empty;
      std::swap(buf_, empty);
    }

  private:
    T2 squareSum_; // We can plugin a more accurate type for square sum
    size_t windowLen_; // The capability 
    size_t N_;     // THe number of elements
    std::queue<T> buf_;
};

