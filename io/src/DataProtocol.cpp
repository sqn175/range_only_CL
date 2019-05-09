#include "DataProtocol.h"

#include <iostream>

MessageParser::MessageParser()
    : cBuf_(boost::circular_buffer<char>(1024)) {
  }

  void MessageParser::pushData(const char* data, unsigned int len) {
    for (int i = 0; i < len; ++i) {
      cBuf_.push_back(*(data+i));
    }
  }

std::vector<measBasePtr> MessageParser::popMsgs() {
  std::vector<measBasePtr> measurements;
  while(popUntilHeader()) {
    // Incomplete message only containing a header
    if (cBuf_.size() <= MSG_HEAD_LEN) 
      return measurements;

    // If the header is found, we then extract the payload length
    int payloadLen = cBuf_.at(INDEX_MSG_PAYLOAD_LEN);

    // Not a complete message
    if (cBuf_.size() < INDEX_MSG_TYPE + payloadLen + MSG_TAIL_LEN)
      return measurements;

    // Bad tail
    if (cBuf_.at(payloadLen + INDEX_MSG_TYPE) != (char)0xDD) {
      // TODO: add log
      std::cout << "Wrong tail" << std::endl;
      pop_front(1);
      continue;
    }

    char msgType = cBuf_.at(INDEX_MSG_TYPE);
    switch (msgType) {
      case MSG_TYPE_IMU : {
        ImuData data = parseData<ImuData>(payloadLen);

        double t = data.timeStamp / 1000.0; // Convert ms to seconds
        double acc[3];
        double gyro[3];
        for (int i = 0; i < 3; ++i) {
          acc[i] = data.acc[i] / 8192.0;
          gyro[i] = data.gyro[i] / 131.072;
        }
        measBasePtr imuM = std::make_shared<ImuMeasurement>(data.uwbId, t, acc, gyro);
        // measurements.push_back(imuM);
        break;
      }
      case MSG_TYPE_WHEEL : {
        WheelData data = parseData<WheelData>(payloadLen);

        double t = data.timeStamp / 1000.0; 
        double v = (double)data.dx / (double)data.interval * 1000.0;
        double omega = (double)data.dphi / (double)data.interval * 1000.0;
        measBasePtr wheelM = std::make_shared<WheelMeasurement>(data.uwbId, t, v, omega);
        measurements.push_back(wheelM);
        break;
      }
      case MSG_TYPE_UWB : {
        UwbData data = parseData<UwbData>(payloadLen);

        double t = data.timeStamp / 1000.0;

        // Calibration of UWB broadcast two way ranging results.
        // Here we simply add a constant bias, a more complex calibration model will be utilized.
        double range = data.range + 0.5158; // TODO: range calibration parameters 

        // TODO: add simple outlier rejection
        measBasePtr uwbM = std::make_shared<UwbMeasurement>(t, data.uwbId1, data.uwbId2, range);
        measurements.push_back(uwbM);
        break;
      }
      default:
        // throw();TODO: add log
        std::cout << "Wrong message type: " << msgType << std::endl;
    }
  }

  return measurements;
}

bool MessageParser::popUntilHeader() {
  if (MSG_HEAD_LEN > cBuf_.size())
    return false;

  while (cBuf_.size() >= MSG_HEAD_LEN) {
    bool isHeadFound = true;
    for (int i = 0; i < MSG_HEAD_LEN; ++i) {
      auto it = cBuf_.begin() + i;
      isHeadFound &= (*it == MSG_HEAD[i]);
      if (!isHeadFound)
        break;
    }

    if (isHeadFound) {
        return true;
    } else {
        pop_front(1);
    }
  }
    return false;
}

template<typename T>
T MessageParser::parseData(int payloadLen) {
  T t;
  // Arrive at the data shipped in the binary message
  pop_front(INDEX_MSG_DATA);
  int dataLen = payloadLen - (INDEX_MSG_TYPE - INDEX_MSG_PAYLOAD_LEN);
  // Be sure this is a little endian system, otherwise we will get the wrong parsing results
  for (int i = 0; i < dataLen; ++i) {
    memcpy((char*)&t+i, &cBuf_.at(i), 1);
  }
  pop_front(dataLen + MSG_TAIL_LEN);
  return t;
}

void MessageParser::pop_front(unsigned int len) {
  for (int i = 0; i < len; ++i) {
    if (cBuf_.empty()) {
      std::cout<<"empty buf" << std::endl;
    }
    cBuf_.pop_front();
  }
}



