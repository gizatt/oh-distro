
#include "IRB140Estimator.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  const char* drc_path = std::getenv("DRC_BASE");
  if (!drc_path) {
    throw std::runtime_error("environment variable DRC_BASE is not set");
  }

  std::shared_ptr<RigidBodyTree> arm(new RigidBodyTree(std::string(drc_path) + "/software/drake/drake/examples/IRB140/urdf/irb_140.urdf"));
  arm->compile();
  std::shared_ptr<RigidBodyTree> manipuland(new RigidBodyTree(std::string(drc_path) + "/software/control/src/compressed_air_can.urdf"));
  manipuland->compile();

  VectorXd x0_arm(arm->num_positions + arm->num_velocities);
  x0_arm*=0;

  VectorXd x0_manipuland(manipuland->num_positions + manipuland->num_velocities);
  x0_manipuland << 0.4, 0.0, 1.0, 0.0, 0.0, 0.0, 
                   0.0, 0.0, 0.7, 0.0, 0.0, 0.0 ;

  std::unique_ptr<IRB140Estimator> estimator(new IRB140Estimator(arm, manipuland, x0_arm, x0_manipuland,
    (std::string(drc_path) + "/software/config/irb140/irb140.cfg").c_str()));
  std::cout << "IRB140 Estimator Listening" << std::endl;
  estimator->run();
  return 0;
}