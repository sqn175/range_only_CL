#include "DataProtocol.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

#include <iostream>
#include <fstream>
#include <iterator>
#include <map>
#include <vector>

TEST(DataProtocolTest, parser) {
  const unsigned char s[] = { 0xA5, 0x5A, 0x12, 0x01, 0xA4, 0x4F, 0x34, 
                              0x00, 0x04, 0xF1, 0xDF, 0xAF, 0xFF, 0xAA, 
                              0x00, 0x0A, 0x00, 0x8D, 0x00, 0xFA, 0xF7, 0xDD};
  MessageParser parser;
  parser.pushData((char*)s, 23);
  auto i = parser.popMsgs();
}

TEST(DataProtocolTest, parserFile) {
  std::ifstream input("/home/qin/Documents/range_only_CL/io/test/testdata.txt", std::ios::binary);
  char data[64];
  MessageParser parser;
  
  if (!input.good()) {
    std::cout << "testdata.txt not found" << std::endl;
  }

  ASSERT_TRUE(input.good());

  std::map<int, std::vector<ImuMeasPtr>> imuMeas;
  std::map<int, std::vector<WheelMeasPtr>> wheelMeas;
  std::map<std::pair<int, int>, std::vector<UwbMeasPtr>> uwbMeas;

  while(input.read(data, 1)) {
    parser.pushData(data, input.gcount());
    auto tmp = parser.popMsgs();
    for (auto& m: tmp) {
        switch (m->type_) {
          case MeasurementType::IMU : {
            auto imuMeasPtr = std::dynamic_pointer_cast<ImuMeasurement>(m);
            imuMeas[imuMeasPtr->uwbId_].push_back(imuMeasPtr);
            break;
          }
          case MeasurementType::WHEEL : {
            auto wheelMeasPtr = std::dynamic_pointer_cast<WheelMeasurement>(m);
            wheelMeas[wheelMeasPtr->uwbId_].push_back(wheelMeasPtr);
            break;
          }
          case MeasurementType::UWB : {
            auto uwbMeasPtr = std::dynamic_pointer_cast<UwbMeasurement>(m);
            uwbMeas[uwbMeasPtr->uwbPair_].push_back(uwbMeasPtr);
            break;
          }
          default: {
            LOG(WARNING) << "Unknown measurement type: " << m->type_;
            break;
          }
        } 
    }
  }

  ASSERT_EQ(uwbMeas.size(), 6);
  ASSERT_EQ(imuMeas.size(), 4);
  ASSERT_EQ(wheelMeas.size(), 2);

  ASSERT_EQ(uwbMeas.begin()->second.size(), 20935);
  ASSERT_NEAR(uwbMeas.begin()->second[0]->range_, 0.405035292508921, 1e-10);
  ASSERT_NEAR(uwbMeas.begin()->second[10268]->range_, 0.779440403886972, 1e-10);
}