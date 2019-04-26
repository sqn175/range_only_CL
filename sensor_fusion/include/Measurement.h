#pragma once

#include "Eigen/Dense"

#include <cstdint>
#include <utility>
#include <memory>
#include <iostream>

enum class MeasurementType{UNKNOWN, IMU, WHEEL, UWB};
inline std::ostream& operator<< (std::ostream& os, const MeasurementType& m)
{
    switch (m)
    {
        case MeasurementType::IMU :    return os << "IMU" ;
        case MeasurementType::WHEEL:   return os << "WHEEL";
        case MeasurementType::UWB:     return os << "UWB";
        case MeasurementType::UNKNOWN: return os << "UNKNOWN";
        // omit default case to trigger compiler warning for missing cases
    };
    return os << static_cast<std::uint16_t>(m);
}

struct MeasurementBase {
  MeasurementType type_;       
  double timeStamp_;  // second

  MeasurementBase()
    : timeStamp_(0)
    , type_(MeasurementType::UNKNOWN) {
  }
  MeasurementBase(MeasurementType type, double t)
    : type_(type)
    , timeStamp_(t) {
  }
  virtual ~MeasurementBase() {}
};

using measBasePtr = std::shared_ptr<MeasurementBase>; 

struct ImuMeasurement : public MeasurementBase {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int uwbId_;
  Eigen::Vector3d gyroscopes_;     // rad/s^2
  Eigen::Vector3d accelerometers_; // m/s^2
  
  ImuMeasurement() 
    : MeasurementBase() {
  }
  ImuMeasurement(int uwbId, double t, double acc[3], double gyro[3])
    : MeasurementBase(MeasurementType::IMU, t)
    , uwbId_(uwbId) {
    accelerometers_ << acc[0], acc[1], acc[2];
    gyroscopes_ << gyro[0], gyro[1], gyro[2];
  }
};

using ImuMeasPtr = std::shared_ptr<ImuMeasurement>;

struct WheelMeasurement : public MeasurementBase {
  int uwbId_;
  double v_;     // m/s
  double omega_; // rad/s

  WheelMeasurement()
    : MeasurementBase() {
  }

  WheelMeasurement(int uwbId, double t, double v, double omega) 
    : MeasurementBase(MeasurementType::WHEEL, t)
    , uwbId_(uwbId)
    , v_(v)
    , omega_(omega) {
  }
};

using WheelMeasPtr = std::shared_ptr<WheelMeasurement>;

struct UwbMeasurement : public MeasurementBase {
  std::pair<uint8_t, uint8_t> uwbPair_;
  double range_;   // meter

  UwbMeasurement()
    : MeasurementBase() {  
  }

  UwbMeasurement(double t, uint8_t uwbId1, uint8_t uwbId2, double range)
    : MeasurementBase(MeasurementType::UWB, t)
    , uwbPair_(std::make_pair(uwbId1, uwbId2))
    , range_(range) {
  }
};

using UwbMeasPtr = std::shared_ptr<UwbMeasurement>;