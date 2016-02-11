
#include "IRB140Estimator.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  const char* drc_path = std::getenv("DRC_BASE");
  if (!drc_path) {
    throw std::runtime_error("environment variable DRC_BASE is not set");
  }

  std::shared_ptr<RigidBodyTree> model(new RigidBodyTree(std::string(drc_path) + "/software/drake/drake/examples/IRB140/urdf/irb_140.urdf"));
  model->compile();

  std::unique_ptr<IRB140Estimator> estimator(new IRB140Estimator(model,
    (std::string(drc_path) + "/software/config/irb140/multisense_05.cfg").c_str()));
  std::cout << "IRB140 Estimator Listening" << std::endl;
  estimator->run();
  return 0;
}