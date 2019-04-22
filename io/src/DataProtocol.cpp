#include "DataProtocol.h"

#include <iostream>

Measurements::Measurements() {

}
void Measurements::push_back_ImuData(const ImuData& data) {
  imuMeasurement_[data.anchorId].push_back(data);
}
void Measurements::push_back_WheelData(const WheelData& data) {
  wheelMeasurement_[data.anchorId].push_back(data);
}
void Measurements::push_back_UwbData(const UwbData& data) {
  std::pair<int,int> key = std::make_pair(data.anchorId1, data.anchorId2);
  uwbMeasurement_[key].push_back(data);
}


MessageParser::MessageParser()
    : cBuf_(boost::circular_buffer<char>(1024)) {
  }

  void MessageParser::pushData(const char* data, unsigned int len) {
    for (int i = 0; i < len; ++i) {
      cBuf_.push_back(*(data+i));
    }
  }

void MessageParser::popMsgs() {
  while(popUntilHeader()) {
    // Incomplete message only containing a header
    if (cBuf_.size() <= MSG_HEAD_LEN) 
      return;

    // If the header is found, we then extract the payload length
    int payloadLen = cBuf_.at(INDEX_MSG_PAYLOAD_LEN);

    // Not a complete message
    if (cBuf_.size() < INDEX_MSG_TYPE + payloadLen + MSG_TAIL_LEN)
      return;

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
        measurements_.push_back_ImuData(data);
        break;
      }
      case MSG_TYPE_WHEEL : {
        WheelData data = parseData<WheelData>(payloadLen);
        measurements_.push_back_WheelData(data);
        break;
      }
      case MSG_TYPE_UWB : {
        UwbData data = parseData<UwbData>(payloadLen);
        measurements_.push_back_UwbData(data);
        break;
      }
      default:
        // throw();TODO: add log
        std::cout << "Wrong message type: " << msgType << std::endl;
    }
  }
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



