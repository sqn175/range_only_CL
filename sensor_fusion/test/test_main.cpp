#include "gtest/gtest.h"
#include "glog/logging.h"
#include <memory>
#include <vector>
#include <map>

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = 0;  // INFO: 0, WARNING: 1, ERROR: 2, FATAL: 3
  FLAGS_colorlogtostderr = 1;

  std::shared_ptr<int> a = std::make_shared<int>(1231);
  std::vector<int> b;
  b.push_back(0);
  b.push_back(2);

  std::map<int, double> c;
  c[3] = 0.1;
  c[8] = 1;


  return RUN_ALL_TESTS();
}