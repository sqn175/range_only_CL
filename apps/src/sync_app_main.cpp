#include "DataProtocol.h"
#include "Estimator.h"

#include "glog/logging.h"

#include <fstream>

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  MessageParser parser;
  Estimator estimator;

  std::string fileName = "/home/qin/Documents/range_only_CL/datasets/20190418/pc.txt";
  std::ifstream dataFile(fileName, std::ios::binary);
  char data[64];  // File read buffer
  if (!dataFile.good()) {
    LOG(ERROR) << "Data file not found: " << fileName;
    return -1;
  }

  while(dataFile.read(data, 64)) {
    parser.pushData(data, dataFile.gcount());
    auto m = parser.popMsgs();
    for (auto& i : m) {
      VLOG(4) << "Receive measurement: " << i->type_ << ", t: " << i->timeStamp; 
      estimator.Estimate(i);
    }
    
  }

}