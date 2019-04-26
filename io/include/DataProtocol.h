#pragma once

#include "Measurement.h"

#include <boost/circular_buffer.hpp>

#include <cstdint>
#include <ctime>
#include <vector>
#include <memory>

// The byte index of the binary message
#define INDEX_MSG_HEADER       0
#define INDEX_MSG_PAYLOAD_LEN  2
#define INDEX_MSG_TYPE         3
#define INDEX_MSG_DATA         4

static const char MSG_HEAD[INDEX_MSG_PAYLOAD_LEN] = {(char)0xA5, (char)0x5A};
static const char MSG_TAIL = (char)0xDD;

static const unsigned char MSG_LEN_IMU    = 0x12;
static const unsigned char MSG_LEN_WHEEL  = 0x13;
static const unsigned char MSG_LEN_UWB    = 0x0F;

static const char MSG_TYPE_IMU   = (char)0x01;
static const char MSG_TYPE_WHEEL = (char)0x02;
static const char MSG_TYPE_UWB   = (char)0x03;

static const unsigned int MSG_HEAD_LEN = sizeof(MSG_HEAD) / sizeof(char);
static const unsigned int MSG_TAIL_LEN = sizeof(MSG_TAIL) / sizeof(char);

#pragma pack(push,1)
// Little Endian data from serial port
// IMU data
using ImuData = struct {
  uint32_t timeStamp; // ms
  uint8_t uwbId;  // Which anchor send this message
  uint16_t acc[3];   // accelerometers, [x,y,z]: real = acc/8192*m/s^2
  uint16_t gyro[3];  // gyroscopes, [x,y,z]: real = gyro/131.072*rad/s
};

// Wheel encoder data
using WheelData = struct {
  uint32_t timeStamp;
  uint8_t uwbId;
  uint8_t interval;
  float dx;         // m
  float dy;         // m
  float dphi;       // rad
};

// UWB ranging data
using UwbData = struct {
  uint32_t timeStamp;
  uint8_t uwbId1;
  uint8_t uwbId2;
  double range;       // m
};
#pragma pack(pop)

class MessageParser {
  public:
    MessageParser();
    void pushData(const char* data, unsigned int len);

    std::vector<measBasePtr> popMsgs();

  private:
    bool popUntilHeader();

    template<typename T>
    T parseData(int payloadLen);

    void pop_front(unsigned int len);
    // Inner circular buffer to store the incoming binary messages
    boost::circular_buffer<char> cBuf_;
};


