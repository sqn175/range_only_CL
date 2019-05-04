#include "DataProtocol.h"
#include "CLSystem.h"

#include "glog/logging.h"
#include "matplotlibcpp.h"

#include <fstream>

namespace plt = matplotlibcpp;
class Viewer {
  public:
    Viewer() {

    }
    void InitAsCallBack(std::map<int, Robot> iniRobots, 
                std::map<int, Eigen::Vector2d> anchorPositions) {
      nRobot_ = iniRobots.size();
      nAnchor_ = anchorPositions.size();
      arrowX_.reserve(nRobot_);
      arrowY_.reserve(nRobot_);
      arrowU_.reserve(nRobot_);
      arrowV_.reserve(nRobot_);
      ancX_.reserve(nAnchor_);
      ancY_.reserve(nAnchor_);
      for (auto& anc : anchorPositions) {
        ancX_.push_back(anc.second(0));
        ancY_.push_back(anc.second(1));
        plt::text(anc.second(0), anc.second(1), "Anchor " + std::to_string(anc.first));
      }

      plt::title("Collaborative Localization");
      plt::axis("equal");

      // Plot anchors
      plt::plot(ancX_, ancY_);
    }

    void UpdateAsCallBack(std::map<int, Robot> robots) {
      // Clear previous plot
      plt::clf();

      // Plot anchors
      plt::plot(ancX_, ancY_);

      for (auto&r : robots) {
        int rId = r.first;
        if (x_[rId].size() >= 10) {
          x_[rId].erase(x_[rId].begin());
          y_[rId].erase(y_[rId].begin());
          phi_[rId].erase(phi_[rId].begin());
        }
        x_[rId].push_back(r.second.state_.x_);
        y_[rId].push_back(r.second.state_.y_);
        phi_[rId].push_back(r.second.state_.phi_);
        // Plot robot trajectories
        plt::plot(x_[rId], y_[rId]);

        // Orientation arrow
        // u and v are respectively the x and y components of the arrows we're plotting
        arrowX_.push_back(r.second.state_.x_);
        arrowY_.push_back(r.second.state_.y_);
        double u = r.second.state_.x_ + cos(r.second.state_.phi_) * 0.2;
        double v = r.second.state_.y_ + sin(r.second.state_.phi_) * 0.2;
        arrowU_.push_back(u);
        arrowV_.push_back(v);
      }
      // plot arrows
      plt::quiver(arrowX_, arrowY_, arrowU_, arrowV_);
      plt::show();
    }

  private:
    int nRobot_;
    int nAnchor_;
    std::map<int, std::vector<double>> x_;
    std::map<int, std::vector<double>> y_;
    std::map<int, std::vector<double>> phi_;
    std::vector<double> ancX_;
    std::vector<double> ancY_;
    std::vector<double> arrowX_;   // arrow start point
    std::vector<double> arrowY_;
    std::vector<double> arrowU_;   // arrow end point
    std::vector<double> arrowV_;
};


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

  MessageParser parser;
  CLSystem rclSystem(noises, deltaSec);
  Viewer viewer;

  std::string fileName = "/home/qin/Documents/range_only_CL/datasets/20190418/pc.txt";
  std::ifstream dataFile(fileName, std::ios::binary);
  char data[64];  // File read buffer
  if (!dataFile.good()) {
    LOG(ERROR) << "Data file not found: " << fileName;
    return -1;
  }

  while(dataFile.read(data, 64)) {
    // Parse data from binary data stream
    parser.pushData(data, dataFile.gcount());
    auto m = parser.popMsgs();

    // Process the parsed measurements
    for (auto& i : m) {
      VLOG(4) << "Receive measurement: " << i->type_ << ", t: " << i->timeStamp; 
      rclSystem.process(i);
    }
    
  }

}