#include "DataProtocol.h"
#include "CLSystem.h"
#include "Viewer.h"

#include "glog/logging.h"

#include <fstream>
#include <mutex>
#include <thread>
#include <iostream>

void measurementFeedingLoop(CLSystem* rclSystem, Viewer* viewer) {
  MessageParser parser;
  std::string fileName = "/home/qin/Documents/range_only_CL/datasets/20190418/pc.txt";
  std::ifstream dataFile(fileName, std::ios::binary);
  char data[64];  // File read buffer
  if (!dataFile.good()) {
    LOG(ERROR) << "Data file not found: " << fileName;
    return;
  }

  while(dataFile.read(data, 836)) {
    // Parse data from binary data stream
    parser.pushData(data, dataFile.gcount());
    auto m = parser.popMsgs();

    // Process the parsed measurements
    for (auto& i : m) {
      VLOG(4) << "Receive measurement: " << i->type << ", t: " << i->timeStamp; 
      rclSystem->AddMeasurement(i);
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  std::cout << "Press Enter to Quit";
  getchar(); 

  viewer->quit();
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  NoiseParams noises;
  noises.sigmaV = 0.05;
  noises.sigmaOmega = 0.05;
  noises.sigmaRange = 0.09;
  noises.sigmaHeading = 1; // TODO: tune this parameter
  double deltaSec = 0.05; // 1/frequency of wheel encoder

  CLSystem rclSystem(noises, deltaSec);
  Viewer viewer;

  rclSystem.SetIniStatesCallback(
    std::bind(&Viewer::InitAsCallBack, &viewer,
              std::placeholders::_1, std::placeholders::_2));

  rclSystem.SetStatesCallback(
    std::bind(&Viewer::UpdateAsCallBack, &viewer,
              std::placeholders::_1));

  std::thread measurementFeedingThread(measurementFeedingLoop, &rclSystem, &viewer);

  // Main loop
  viewer.PlottingLoop();

  measurementFeedingThread.join();
  return 0;
}