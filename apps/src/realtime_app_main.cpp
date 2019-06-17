#include "AsyncSerial.h"
#include "DataProtocol.h"
#include "CLSystem.h"
#include "Viewer.h"

#include <fstream>

using namespace std;

class MainSystem {
  public:
    MainSystem(CLSystem* rcl)
      : rclSystem(rcl) {
      std::string fileName = "/home/qin/Documents/range_only_CL/datasets/rawMeasurements.txt";
      file_ = std::move(std::ofstream(fileName, std::ios::binary));
      if (!file_.good()) {
        LOG(ERROR) << "Open file failed" << fileName;
      }
    }
    void OnSerialReceived(const char* data, unsigned int len)
    {
      // Write to file
      file_.write(data, len);

      // Parse data from binary data stream
      parser.pushData(data, len);
      auto m = parser.popMsgs();

      // Process the parsed measurements
      for (auto& i : m) {
        VLOG(4) << "Receive measurement: " << i->type << ", t: " << i->timeStamp; 
        rclSystem->AddMeasurement(i);
      }
    }

    void SavingAndFeedingLoop() {
      std::string serialPortName = "/dev/ttyACM0";
      unsigned int baudRate = 115200;
      CallbackAsyncSerial serial(serialPortName, baudRate);

      serial.setCallback(
              std::bind(&MainSystem::OnSerialReceived, this,
              std::placeholders::_1, std::placeholders::_2));

      while (1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }

  private:
    CLSystem* rclSystem;
    MessageParser parser;

    std::ofstream file_;

    // TODO: add quit method
};

int main(int argc, char* argv[])
{
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  NoiseParams noises;
  noises.sigmaV = 0.05;
  noises.sigmaOmega = 0.05;
  noises.sigmaRange = 0.09;
  noises.sigmaHeading = 1; // TODO: tune this parameter
  double deltaSec = 0.05; // 1/frequency of wheel encoder

  Viewer viewer;

  CLSystem rclSystem(noises, deltaSec);
  rclSystem.SetIniStatesCallback(
    std::bind(&Viewer::InitAsCallBack, &viewer,
              std::placeholders::_1, std::placeholders::_2));
  rclSystem.SetStatesCallback(
    std::bind(&Viewer::UpdateAsCallBack, &viewer,
              std::placeholders::_1));

  MainSystem mainsys(&rclSystem);

  std::thread serialFeedingThread(&MainSystem::SavingAndFeedingLoop, &mainsys);

  viewer.PlottingLoop();

  return 0;
}
