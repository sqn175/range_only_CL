#include "DataProtocol.h"
#include "Estimator.h"

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
      nRobot_ = robots_.size();
      nAnchor_ = anchorPositions_.size();
      arrowX_.reserve(nRobot_);
      arrowY_.reserve(nRobot_);
      arrowU_.reserve(nRobot_);
      arrowV_.reserve(nRobot_);
      std::vector<double> ancX(nAnchor_);
      std::vector<double> ancY(nAnchor_);
      for (auto& anc : anchorPositions) {
        ancX.push_back(anc.second(0));
        ancY.push_back(anc.second(1));
        text(anc.second(0), anc.second(1), "Anchor " + std::to_string(anc.first));
      }

      plt::title("Collaborative Localization");
      plt::axis("equal");

      // Plot anchors
      plt::plot(ancX, ancY);
    }

    void UpdateAsCallBack(std::map<int, Robot> robots) {
      for (auto& r : robots) {
        arrowX_.clear();
        arrowY_.clear();
        arrowU_.clear();
        arrowV_.clear();
        
      }
    }

  private:
    int nRobot_;
    int nAnchor_;
    std::map<int, std::vector<double>> x_;
    std::map<int, std::vector<double>> y_;
    std::map<int, std::vector<double>> phi_;
    std::vector<double> arrowX_;   // arrow start point
    std::vector<double> arrowY_;
    std::vector<double> arrowU_;   // arrow end point
    std::vector<double> arrowV_;
};
void update_window(const double x, const double y, const double t,
                   std::vector<double> &xt, std::vector<double> &yt)
{
    const double target_length = 300;
    const double half_win = (target_length/(2.*sqrt(1.+t*t)));

    xt[0] = x - half_win;
    xt[1] = x + half_win;
    yt[0] = y - half_win*t;
    yt[1] = y + half_win*t;
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  // test
   size_t n = 1000;
    std::vector<double> x, y;

    const double w = 0.05;
    const double a = n/2;

    for (size_t i=0; i<n; i++) {
        x.push_back(i);
        y.push_back(a*sin(w*i));
    }

    std::vector<double> xt(2), yt(2);

    plt::title("Tangent of a sine curve");
    plt::xlim(x.front(), x.back());
    plt::ylim(-a, a);
    plt::axis("equal");

    // Plot sin once and for all.
    plt::named_plot("sin", x, y);

    // Prepare plotting the tangent.
    plt::Plot plot("tangent");

    plt::legend();

    for (size_t i=0; i<n; i++) {
        if (i % 10 == 0) {
            update_window(x[i], y[i], a*w*cos(w*x[i]), xt, yt);

            // Just update data for this plot.
            plot.update(xt, yt);

            // Small pause so the viewer has a chance to enjoy the animation.
            plt::pause(0.1);
        }
   }
  // test end

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

    }
    
  }

}