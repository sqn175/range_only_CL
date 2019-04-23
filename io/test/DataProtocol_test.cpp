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

  while(input.read(data, 1)) {
    parser.pushData(data, input.gcount());
    auto i = parser.popMsgs();
  }


}