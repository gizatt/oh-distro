#ifndef IRB140_ESTIMATOR_H
#define IRB140_ESTIMATOR_H

#include <stdexcept>
#include <iostream>
#include <lcm/lcm-cpp.hpp>

#include "drake/systems/plants/RigidBodyTree.h"
#include "lcmtypes/drc/utime_t.hpp"
#include "lcmtypes/drake/lcmt_robot_state.hpp"
#include "bot_core/planar_lidar_t.hpp"
#include "bot_core/rigid_transform_t.hpp"
#include "lcmtypes/drake/lcmt_point_cloud.hpp"

class IRB140Estimator {
public:
  ~IRB140Estimator() {}

  IRB140Estimator(std::shared_ptr<RigidBodyTree> model);
  void run() {
    while(0 == this->lcm.handle());
  }

  void setupSubscriptions();

  void handlePlanarLidarMsg(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const bot_core::planar_lidar_t* msg);

  void handlePointCloudMsg(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const drake::lcmt_point_cloud* msg);

  void handleSpindleFrameMsg(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const bot_core::rigid_transform_t* msg);

private:
  std::shared_ptr<RigidBodyTree> model;
  KinematicsCache<double> kinematics_cache;

  lcm::LCM lcm;
};

#endif