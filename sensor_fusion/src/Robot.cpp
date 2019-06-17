#include "Robot.h"

Robot::Robot()
  : id_(-1) {
}

Robot::Robot(int id, double x, double y) 
  : id_(id)
  , state_(x, y, 0.0){
}

Robot::Robot(int id, double x, double y, double phi)
  : id_(id)
  , state_(x, y, phi) {
}

void Robot::SetState(double x, double y, double phi) {
  state_.x_ = x;
  state_.y_ = y;
  state_.phi_ = phi;
}

void Robot::SetOrientation(double phi) {
  state_.phi_ = phi;
}

int Robot::id() const {
  return id_;
}

void Robot::SetTimestamp(double t) {
  timeStamp_ = t;
}

double Robot::t() const {
  return timeStamp_;
}