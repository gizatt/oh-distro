
#include "IRB140Estimator.hpp"

using namespace std;
using namespace Eigen;

double getUnixTime(void)
{
    struct timespec tv;

    if(clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

    return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}

int main(int argc, char** argv) {
  const char* drc_path = std::getenv("DRC_BASE");
  if (!drc_path) {
    throw std::runtime_error("environment variable DRC_BASE is not set");
  }

  std::shared_ptr<RigidBodyTree> model(new RigidBodyTree(std::string(drc_path) + "/software/drake/drake/examples/IRB140/urdf/irb_140.urdf"));
  model->compile();

  std::unique_ptr<IRB140Estimator> estimator(new IRB140Estimator(model));
  std::cout << "IRB140 Estimator Listening" << std::endl;
  estimator->run();
  return 0;
}