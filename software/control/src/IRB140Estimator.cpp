#include "IRB140Estimator.hpp"
#include "drake/util/convexHull.h"

using namespace std;
using namespace Eigen;


IRB140Estimator::IRB140Estimator(std::shared_ptr<RigidBodyTree> model) :
    kinematics_cache(model->bodies)
{
  this->model = model;


  if (!this->lcm.good()) {
    throw std::runtime_error("LCM is not good");
  }
}


void IRB140Estimator::setupSubscriptions(){
  //lcm->subscribe("SCAN", &IRB140EstimatorSystem::handlePointCloud, this);
  lcm.subscribe("SCAN", &IRB140Estimator::handlePlanarLidarMsg, this);
  lcm.subscribe("PRE_SPINDLE_TO_POST_SPINDLE", &IRB140Estimator::handleSpindleFrameMsg, this);
}

void IRB140Estimator::handlePlanarLidarMsg(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const bot_core::planar_lidar_t* msg){
  printf("Received scan on channel %s\n", chan.c_str());
  // transform according 
}

void IRB140Estimator::handlePointCloudMsg(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const drake::lcmt_point_cloud* msg){
  printf("Received scan pts on channel %s\n", chan.c_str());
  // todo: transform them all by the lidar frame
}

void IRB140Estimator::handleSpindleFrameMsg(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const bot_core::rigid_transform_t* msg){
  printf("Received transform on channel %s\n", chan.c_str());
  cout << msg->trans << "," << msg->quat << endl;
  // todo: transform them all by the lidar frame
}