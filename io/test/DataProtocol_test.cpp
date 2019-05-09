#include "DataProtocol.h"

#include "gtest/gtest.h"
#include "glog/logging.h"

#include <iostream>
#include <fstream>
#include <iterator>
#include <map>
#include <vector>

TEST(DataProtocolTest, wheelDataParser) {
  const unsigned char s[] = {165,90,19,2,106,173,56,0,1,50,151,144,175,59,0,0,0,0,59,87,76,60,221};
  MessageParser parser;
  parser.pushData((char*)s, 23);
  auto m = parser.popMsgs();
  auto wheelMeasPtr = std::dynamic_pointer_cast<WheelMeasurement>(m.front());

  ASSERT_EQ(wheelMeasPtr->timeStamp, 3.71441e3);
  ASSERT_EQ(wheelMeasPtr->type_, MeasurementType::WHEEL);
  ASSERT_EQ(wheelMeasPtr->uwbId, 1);
  ASSERT_NEAR(wheelMeasPtr->omega, 0.249439384788275, 1e-8);
  ASSERT_NEAR(wheelMeasPtr->v, 0.107156252488494, 1e-8);
}

TEST(DataProtocolTest, parserFile) {

  ASSERT_EQ(sizeof(float), 4);  // float32 for dx,dy,dphi in WheelData 

  std::ifstream input("/home/qin/Documents/range_only_CL/io/test/20190418.txt", std::ios::binary);
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
            imuMeas[imuMeasPtr->uwbId].push_back(imuMeasPtr);
            break;
          }
          case MeasurementType::WHEEL : {
            auto wheelMeasPtr = std::dynamic_pointer_cast<WheelMeasurement>(m);
            wheelMeas[wheelMeasPtr->uwbId].push_back(wheelMeasPtr);
            break;
          }
          case MeasurementType::UWB : {
            auto uwbMeasPtr = std::dynamic_pointer_cast<UwbMeasurement>(m);
            uwbMeas[uwbMeasPtr->uwbPair].push_back(uwbMeasPtr);
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

  ASSERT_EQ(uwbMeas.begin()->second.size(), 34662);
  ASSERT_NEAR(uwbMeas.begin()->second[0]->range, 1.017652, 1e-6);
  ASSERT_NEAR(uwbMeas.begin()->second[26340]->range, 1.605612, 1e-6);
  // An outlier
  ASSERT_NEAR(uwbMeas.begin()->second[22551]->range, 2.509796148336626e+09, 1);

  ASSERT_EQ(wheelMeas.begin()->first, 1);
  ASSERT_EQ(wheelMeas[1].size(), 6932);
  ASSERT_NEAR(wheelMeas[1][1847]->v, 0.1071562, 1e-7);
  ASSERT_NEAR(wheelMeas[1][1847]->omega, 0.2494393, 1e-7);

  auto it = wheelMeas.find(4);
  ASSERT_TRUE(it != wheelMeas.end());
  ASSERT_EQ(wheelMeas[4].size(), 6924);
  ASSERT_NEAR(wheelMeas[4][4194]->v, 5.32716512e-06, 1e-7);
  ASSERT_NEAR(wheelMeas[4][4194]->omega, 0.4929537698, 1e-7);
}