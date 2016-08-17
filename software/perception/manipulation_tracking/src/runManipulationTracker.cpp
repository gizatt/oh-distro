
#include "ManipulationTracker.hpp"
#include "RobotStateCost.hpp"
#include "JointStateCost.hpp"
#include "KinectFrameCost.hpp"
#include "DynamicsCost.hpp"
#include "GelsightCost.hpp"
#include "AttachedApriltagCost.hpp"
#include "OptotrakMarkerCost.hpp"
#include "yaml-cpp/yaml.h"
#include "common.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  const char* drc_path = std::getenv("DRC_BASE");
  if (!drc_path) {
    throw std::runtime_error("environment variable DRC_BASE is not set");
  }

  if (argc != 2){
    printf("Use: runManipulationTrackerIRB140 <path to yaml config file>\n");
    return 0;
  }


  std::shared_ptr<lcm::LCM> lcm(new lcm::LCM());
  if (!lcm->good()) {
    throw std::runtime_error("LCM is not good");
  }

  string configFile(argv[1]);
  YAML::Node config = YAML::LoadFile(configFile);

  VectorXd x0_robot;
  std::shared_ptr<const RigidBodyTree> robot = setupRobotFromConfig(config, x0_robot, string(drc_path), true, false);

  // initialize tracker itself
  ManipulationTracker estimator(robot, x0_robot, lcm, config, true);

  // and register all of the costs that we know how to handle
  if (config["costs"]){
    for (auto iter = config["costs"].begin(); iter != config["costs"].end(); iter++){
      std::string cost_type = (*iter)["type"].as<string>();
      if (cost_type == "RobotStateCost"){
        std::shared_ptr<RobotStateCost> cost(new RobotStateCost(robot, lcm, *iter));
        estimator.addCost(dynamic_pointer_cast<ManipulationTrackerCost, RobotStateCost>(cost));
      } else if (cost_type == "KinectFrameCost") {
        // demands a modifiable copy of the robot to do collision calls
        std::shared_ptr<KinectFrameCost> cost(new KinectFrameCost(setupRobotFromConfig(config, x0_robot, string(drc_path), true, false), lcm, *iter));
        estimator.addCost(dynamic_pointer_cast<ManipulationTrackerCost, KinectFrameCost>(cost));
      } else if (cost_type == "DynamicsCost") { 
        std::shared_ptr<DynamicsCost> cost(new DynamicsCost(robot, lcm, *iter));
        estimator.addCost(dynamic_pointer_cast<ManipulationTrackerCost, DynamicsCost>(cost));
      } else if (cost_type == "JointStateCost") { 
        std::shared_ptr<JointStateCost> cost(new JointStateCost(robot, lcm, *iter));
        estimator.addCost(dynamic_pointer_cast<ManipulationTrackerCost, JointStateCost>(cost));
      } else if (cost_type == "GelsightCost") { 
        // demands a modifiable copy of the robot
        std::shared_ptr<GelsightCost> cost(new GelsightCost(setupRobotFromConfig(config, x0_robot, string(drc_path), true, true), lcm, *iter));
        estimator.addCost(dynamic_pointer_cast<ManipulationTrackerCost, GelsightCost>(cost));
      } else if (cost_type == "AttachedApriltagCost") { 
        std::shared_ptr<AttachedApriltagCost> cost(new AttachedApriltagCost(robot, lcm, *iter));
        estimator.addCost(dynamic_pointer_cast<ManipulationTrackerCost, AttachedApriltagCost>(cost));
      } else if (cost_type == "OptotrakMarkerCost") { 
        std::shared_ptr<OptotrakMarkerCost> cost(new OptotrakMarkerCost(robot, lcm, *iter));
        estimator.addCost(dynamic_pointer_cast<ManipulationTrackerCost, OptotrakMarkerCost>(cost));
      }else {
        cout << "Got cost type " << cost_type << " but I don't know what to do with it!" << endl;
      }
    }
  }

  std::cout << "Manipulation Tracker Listening" << std::endl;

  double last_update_time = getUnixTime();
  double timestep = 0.01;
  if (config["timestep"])
    timestep = config["timestep"].as<double>();

  while(1){
    for (int i=0; i < 100; i++)
      lcm->handleTimeout(0);

    double dt = getUnixTime() - last_update_time;
    if (dt > timestep){
      last_update_time = getUnixTime();
      estimator.update();
      estimator.publish();
    } else {
      usleep(1000);
    }
  }

  return 0;
}