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
#include "lcmtypes/kinect/frame_msg_t.hpp"
#include <kinect/kinect-utils.h>
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include <mutex>
#include <bot_lcmgl_client/lcmgl.h>
#include <bot_frames/bot_frames.h>
#include <bot_param/param_client.h>

static double getUnixTime(void)
{
    struct timespec tv;

    if(clock_gettime(CLOCK_REALTIME, &tv) != 0) return 0;

    return (tv.tv_sec + (tv.tv_nsec / 1000000000.0));
}

class IRB140Estimator {
public:
  ~IRB140Estimator() {}

  IRB140Estimator(std::shared_ptr<RigidBodyTree> model, const char* filename);
  void run() {
    while(1){
      this->lcm.handleTimeout(0);
      double dt = getUnixTime() - last_update_time;
      if (dt > timestep){
        last_update_time = getUnixTime();
        this->update(dt);
      }
    }
  }

  void update(double dt);

  void setupSubscriptions();
  void initBotConfig(const char* filename);
  int get_trans_with_utime(std::string from_frame, std::string to_frame,
                               long long utime, Eigen::Isometry3d & mat);

  void handlePlanarLidarMsg(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const bot_core::planar_lidar_t* msg);

  void handlePointCloudMsg(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const drake::lcmt_point_cloud* msg);

  void handleSpindleFrameMsg(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const bot_core::rigid_transform_t* msg);

  void handleKinectFrameMsg(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const kinect::frame_msg_t* msg);

private:
  std::shared_ptr<RigidBodyTree> model;
  KinematicsCache<double> kinematics_cache;

  std::mutex latest_cloud_mutex;
  KinectCalibration* kcal;
  pcl::PointCloud<pcl::PointXYZRGB> latest_cloud;

  double last_update_time;
  double timestep = 0.01;

  lcm::LCM lcm;
  bot_lcmgl_t* lcmgl_ = NULL;
  BotParam* botparam_ = NULL;
  BotFrames* botframes_ = NULL;
};

#endif