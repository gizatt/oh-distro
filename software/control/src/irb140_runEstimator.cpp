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

  //std::shared_ptr<RigidBodyTree> manipuland(new RigidBodyTree(std::string(drc_path) + "/software/control/src/jasmine_tea_box.urdf"));
  std::shared_ptr<RigidBodyTree> manipuland(new RigidBodyTree(std::string(drc_path) + "/software/control/src/urdf/robotiq_simple_collision.urdf"));
  //manipuland->addRobotFromURDF(std::string(drc_path) + "/software/drake/drake/examples/Atlas/urdf/robotiq_simple.urdf", DrakeJoint::ROLLPITCHYAW);
  manipuland->compile();

  VectorXd x0_arm(arm->num_positions + arm->num_velocities);
  x0_arm*=0;

  VectorXd x0_manipuland = VectorXd::Zero(manipuland->num_positions + manipuland->num_velocities);
  //x0_manipuland.block<6, 1>(0, 0) << 0.5, 0.0, 0.88, 0.0, 0.0, 1.5;
  //x0_manipuland.block<6, 1>(6, 0) << 0.5, 0.0, 0.7, 0.0, 0.0, 0.0;
  //x0_manipuland.block<6, 1>(12, 0) << 0.5, 0.0, 0.9, 0.0, 0.0, 0.0;
  x0_manipuland.block<6, 1>(0, 0) << 0.5, 0.0, 1.21, 1.6, -1.14, -3.36;
 //0.495786 -0.0261564    1.21738    2.61125   -1.36793   -4.20813    0.80034  -0.454942   0.127296          0          0          0   0.571175   0.969762   0.458476          0          0          0          0          0          0          0          0          0          0          0          0          0          0          0


  std::unique_ptr<IRB140Estimator> estimator(new IRB140Estimator(arm, manipuland, x0_arm, x0_manipuland,
    (std::string(drc_path) + "/software/config/irb140/irb140.cfg").c_str()));
  std::cout << "IRB140 Estimator Listening" << std::endl;
  estimator->run();
  return 0;
}