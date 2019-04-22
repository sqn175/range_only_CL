#include "DataProtocol.h"

#include "gtest/gtest.h"

#include <iostream>
#include <fstream>
#include <iterator>

TEST(DataProtocolTest, parser) {
  const unsigned char s[] = { 0xA5, 0x5A, 0x12, 0x01, 0xA4, 0x4F, 0x34, 
                              0x00, 0x04, 0xF1, 0xDF, 0xAF, 0xFF, 0xAA, 
                              0x00, 0x0A, 0x00, 0x8D, 0x00, 0xFA, 0xF7, 0xDD};
  MessageParser parser;
  parser.pushData((char*)s, 23);
  parser.popMsgs();
}

TEST(DataProtocolTest, parserFile) {
  std::ifstream input("/home/qin/Documents/range_only_CL/io/test/testdata.txt", std::ios::binary);
  char data[64];
  MessageParser parser;
  
  if (!input.good()) {
    std::cout << "testdata.txt not found" << std::endl;
  }

  ASSERT_TRUE(input.good());

  while(input.read(data, 1)) {
    parser.pushData(data, input.gcount());
    parser.popMsgs();
  }

  size_t imuCnt = parser.measurements_.imuMeasurement_.size();
  size_t wheelCnt = parser.measurements_.wheelMeasurement_.size();
  size_t uwbCnt = parser.measurements_.uwbMeasurement_.size();
  ASSERT_EQ(imuCnt, 4);
  ASSERT_EQ(wheelCnt, 2);
  ASSERT_EQ(uwbCnt, 6);

  size_t imuLen0 = parser.measurements_.imuMeasurement_[0].size();
  size_t wheelLen1 = parser.measurements_.wheelMeasurement_[1].size();
  std::pair<int, int> pair = std::make_pair<int, int>(2,4);
  size_t uwbLen5 = parser.measurements_.uwbMeasurement_[pair].size();

  ImuData imufront = parser.measurements_.imuMeasurement_[0].front();
  ImuData imuback = parser.measurements_.imuMeasurement_[0].back();
  std::cout << imufront.timeStamp << "," << imuback.timeStamp << std::endl;

  WheelData wheelfront = parser.measurements_.wheelMeasurement_[1].front();
  WheelData wheelback = parser.measurements_.wheelMeasurement_[1].back();
  std::cout << wheelfront.timeStamp << "," << wheelback.timeStamp << std::endl;

  UwbData uwbfront = parser.measurements_.uwbMeasurement_[pair].front();
  UwbData uwbback = parser.measurements_.uwbMeasurement_[pair].back();
  std::cout << uwbfront.timeStamp << "," << uwbback.timeStamp << std::endl;

  ASSERT_EQ(imuLen0, 20944);    
  ASSERT_EQ(wheelLen1, 4187);
  ASSERT_EQ(uwbLen5, 20938);

}