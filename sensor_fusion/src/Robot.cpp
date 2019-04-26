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

int Robot::id() const {
  return id_;
}