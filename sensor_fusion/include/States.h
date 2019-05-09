#pragma once

#include "Eigen/Dense"

#include <iostream>

// Navigation states
class States {
  public:
    States();
    States(const double& x, const double& y, const double& phi);
    States(const States& s);

    States& operator+=(const States& rhs);
    States& operator*=(const States& rhs);
    States& operator*=(const double& c);
    friend std::ostream& operator<<(std::ostream& out, const States& s);
    Eigen::Vector3d serialize();
  
    Eigen::Matrix3d propagate(double v0, double omega0, 
                              double v1, double omega1, 
                              double deltaSec);
    void correct(const Eigen::Vector3d delta);
    
  public:
    double x_;   // x (meter) coordinate in world reference
    double y_;   // y (meter) coordinate in world reference
    double phi_; // orientation (rad)
  private:
    States dot(const double& v, const double& omega);
    // Continuous time jacobian matrix of f() with respect to error-state
    Eigen::Matrix3d jacobian(const double& v, const double& omega);
};

inline States operator+(States lhs, const States& rhs) {
  lhs += rhs;
  return lhs;
}

inline States operator*(States lhs, const States& rhs) {
  lhs *= rhs;
  return lhs;
}


