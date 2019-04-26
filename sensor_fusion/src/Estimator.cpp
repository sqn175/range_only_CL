#include "Estimator.h"

#include "glog/logging.h"

#include <memory>
#include <assert.h>

Estimator::Estimator() {
}

void Estimator::process(const measBasePtr& m) {
 switch (m->type_) {
    case MeasurementType::IMU : {
      auto imuMeasPtr = std::dynamic_pointer_cast<ImuMeasurement>(m);
      // We do not process IMU data
      break;
    }
    case MeasurementType::WHEEL : {
      auto wheelMeasPtr = std::dynamic_pointer_cast<WheelMeasurement>(m);
      auto id = wheelMeasPtr->uwbId_;
      break;
    }
    case MeasurementType::UWB : {
      auto uwbMeasPtr = std::dynamic_pointer_cast<UwbMeasurement>(m);
      break;
    }
    default: {
      LOG(WARNING) << "Unknown measurement type: " << m->type_;
      break;
    }
  }
  double v1;
  double omega1;
  double deltaSec;

  // Classify the measurements
  for (auto it = robots_.begin(); it != robots_.end(); ++i) {
    // TODO: check if lastV exist
    *it->state_.propagate(lastV_[*it->id], lastOmega_[*it->id], 
                          v1, omega1, deltaSec, false);
  }
}
