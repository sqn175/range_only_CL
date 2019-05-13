#include "DataProtocol.h"
#include "CLSystem.h"

#include "glog/logging.h"

#include <fstream>

#include "pangolin/pangolin.h"

static const std::string window_name = "HelloPangolinThreads";

void setup() {
    // create a window and bind its context to the main thread
    pangolin::CreateWindowAndBind(window_name, 640, 480);

    // enable depth
    glEnable(GL_DEPTH_TEST);

    // unset the current context from the main thread
    pangolin::GetBoundWindow()->RemoveCurrent();
}

void run() {
    // fetch the context and bind it to this thread
    pangolin::BindToContext(window_name);

    // we manually need to restore the properties of the context
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
        pangolin::ModelViewLookAt(-2,2,-2, 0,0,0, pangolin::AxisY)
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
            .SetHandler(&handler);

    while( !pangolin::ShouldQuit() )
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        // Render OpenGL Cube
        pangolin::glDrawColouredCube();

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    // unset the current context from the main thread
    pangolin::GetBoundWindow()->RemoveCurrent();
}

int main(int argc, char **argv) {

     // create window and context in the main thread
    setup();

    // use the context in a separate rendering thread
    std::thread render_loop;
    render_loop = std::thread(run);
    render_loop.join();

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

  // rclSystem.SetIniStatesCallback(
  //   std::bind(&Viewer::InitAsCallBack, &viewer,
  //             std::placeholders::_1, std::placeholders::_2));

  // rclSystem.SetStatesCallback(
  //   std::bind(&Viewer::UpdateAsCallBack, &viewer,
  //             std::placeholders::_1));

  std::string fileName = "/home/qin/Documents/range_only_CL/datasets/20190418/pc.txt";
  std::ifstream dataFile(fileName, std::ios::binary);
  char data[64];  // File read buffer
  if (!dataFile.good()) {
    LOG(ERROR) << "Data file not found: " << fileName;
    return -1;
  }

  while(dataFile.read(data, 64)) {
    // viewer.display();
    // Parse data from binary data stream
    parser.pushData(data, dataFile.gcount());
    auto m = parser.popMsgs();

    // Process the parsed measurements
    for (auto& i : m) {
      VLOG(4) << "Receive measurement: " << i->type << ", t: " << i->timeStamp; 
      rclSystem.AddMeasurement(i);
    }
    
  }

}