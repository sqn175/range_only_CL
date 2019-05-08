#include "States.h"

#include <math.h>

States::States()
  : x_(0.0),y_(0.0),phi_(0.0) {
}

States::States(const double& x, const double& y, const double& phi)
  : x_(x), y_(y), phi_(phi){
}

States::States(const States& s)
  : x_(s.x_), y_(s.y_), phi_(s.phi_){
}

States& States::operator+=(const States& rhs) {
  x_ += rhs.x_;
  y_ += rhs.y_;
  phi_ += rhs.phi_;
  return *this;
}

States& States::operator*=(const States& rhs) {
  x_ *= rhs.x_;
  y_ *= rhs.y_;
  phi_ *= rhs.phi_;
  return *this;
}

States& States::operator*=(const double& c) {
  x_ *= c;
  y_ *= c;
  phi_ *= c;
  return *this;
}

Eigen::Vector3d States::serialize() {
  return Eigen::Vector3d(x_, y_, phi_);
}

States States::dot(const double& v, const double& omega) {
  States s;
  s.x_ = v * cos(phi_);
  s.y_ = v * sin(phi_);
  s.phi_ = omega;
  return s;
}

Eigen::Matrix3d& States::propagate(double v0, double omega0, 
                          double v1, double omega1, 
                          double deltaSec) {

  Eigen::Matrix3d matPhi;
  double vMid = (v0 + v1) / 2;
  double omegaMid = (omega0 + omega1) / 2;
  // The transition matrix
  matPhi = Eigen::Matrix3d::Identity() + deltaSec * jacobian(vMid, omegaMid);
  States tmp = dot(vMid, omegaMid); // The derivative
  tmp *= deltaSec;  // The derivative mtimes the delta t
  *this += tmp;

  return matPhi;
}

void States::correct(const Eigen::Vector3d& delta) {
  x_ += delta[0];
  y_ += delta[1];
  phi_ += delta[2];
  if (phi_ >  M_PI) {
    phi_ -= 2 * M_PI;
  } else if (phi_ < -M_PI) {
    phi_ += 2 * M_PI;
  }
}

Eigen::Matrix3d& States::jacobian(const double& v, const double& omega) {
  Eigen::Matrix3d j;
  j << 0, 0, -v * sin(phi_),
       0, 0, v * cos(phi_),
       0, 0, 0;
  return j;
}

std::ostream& operator<<(std::ostream& out, const States& s) {
  out << "[x,y,phi]: [" << s.x_ << ", " << s.y_ << ", " << s.phi_ <<"]" << std::endl;
  return out;
}





