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

void Robot::setOrientation(double phi) {
  state_.phi_ = phi;
}

int Robot::id() const {
  return id_;
}