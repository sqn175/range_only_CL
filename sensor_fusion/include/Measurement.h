#include <cstdint>
#include <Eigen/Dense>
#include <utility>
#include <memory>

enum class MeasurementType{UNKNOWN, IMU, WHEEL, UWB};

struct MeasurementBase {
  // measurement type
  // 1, IMU; 2, Wheel encoder; 3, UWB ranging; -1: unknown
  MeasurementType type_;       
  double timeStamp_;

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
  int anchorId_;
  Eigen::Vector3d gyroscopes_;
  Eigen::Vector3d accelerometers_;
  
  ImuMeasurement() 
    : MeasurementBase() {
  }
  ImuMeasurement(int anchorId, double t, double acc[3], double gyro[3])
    : MeasurementBase(MeasurementType::IMU, t)
    , anchorId_(anchorId) {
    accelerometers_ << acc[0], acc[1], acc[2];
    gyroscopes_ << gyro[0], gyro[1], gyro[2];
  }
};

using ImuMeasPtr = std::shared_ptr<ImuMeasurement>;

struct WheelMeasurement : public MeasurementBase {
  int anchorId_;
  double v_;
  double omega_;

  WheelMeasurement()
    : MeasurementBase() {
  }

  WheelMeasurement(int anchorId, double t, double v, double omega) 
    : MeasurementBase(MeasurementType::WHEEL, t)
    , anchorId_(anchorId)
    , v_(v)
    , omega_(omega) {
  }
};

using WheelMeasPtr = std::shared_ptr<WheelMeasurement>;

struct UwbMeasurement : public MeasurementBase {
  std::pair<uint8_t, uint8_t> anchorPair_;
  double range_;

  UwbMeasurement()
    : MeasurementBase() {  
  }

  UwbMeasurement(double t, uint8_t anchor1, uint8_t anchor2, double range)
    : MeasurementBase(MeasurementType::UWB, t)
    , anchorPair_(std::make_pair(anchor1, anchor2))
    , range_(range) {
  }
};

using UwbMeasPtr = std::shared_ptr<UwbMeasurement>;