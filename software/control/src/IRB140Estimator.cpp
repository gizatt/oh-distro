#include "IRB140Estimator.hpp"
#include "drake/util/convexHull.h"

using namespace std;
using namespace Eigen;



IRB140Estimator::IRB140Estimator(std::shared_ptr<RigidBodyTree> model, const char* filename) :
    kinematics_cache(model->bodies)
{
  this->model = model;

  last_update_time = getUnixTime();

  if (!this->lcm.good()) {
    throw std::runtime_error("LCM is not good");
  }

  lcmgl_= bot_lcmgl_init(lcm.getUnderlyingLCM(), "trimmed_lidar");

  this->initBotConfig(filename);

  // if we're using a kinect... (to be refactored)
  // This is in full agreement with Kintinuous: (calibrationAsus.yml)
  // NB: if changing this, it should be kept in sync
  kcal = kinect_calib_new();
  kcal->intrinsics_depth.fx = 528.01442863461716;//was 576.09757860;
  kcal->intrinsics_depth.cx = 320;
  kcal->intrinsics_depth.cy = 267.0;
  kcal->intrinsics_rgb.fx = 528.01442863461716;//576.09757860; ... 528 seems to be better, emperically, march 2015
  kcal->intrinsics_rgb.cx = 320;
  kcal->intrinsics_rgb.cy = 267.0;
  kcal->intrinsics_rgb.k1 = 0; // none given so far
  kcal->intrinsics_rgb.k2 = 0; // none given so far
  kcal->shift_offset = 1090.0;
  kcal->projector_depth_baseline = 0.075;
  //double rotation[9];
  double rotation[]={0.999999, -0.000796, 0.001256, 0.000739, 0.998970, 0.045368, -0.001291, -0.045367, 0.998970};
  double depth_to_rgb_translation[] ={ -0.015756, -0.000923, 0.002316};
  memcpy(kcal->depth_to_rgb_rot, rotation, 9*sizeof(double));
  memcpy(kcal->depth_to_rgb_translation, depth_to_rgb_translation  , 3*sizeof(double));

  this->setupSubscriptions();
}

void IRB140Estimator::initBotConfig(const char* filename)
{
  if (filename && filename[0])
    {
      botparam_ = bot_param_new_from_file(filename);
    }
  else
    {
    while (!botparam_)
      {
        botparam_ = bot_param_new_from_server(this->lcm.getUnderlyingLCM(), 0);
      }
    }
  botframes_ = bot_frames_get_global(this->lcm.getUnderlyingLCM(), botparam_);
}

int IRB140Estimator::get_trans_with_utime(std::string from_frame, std::string to_frame,
                               long long utime, Eigen::Isometry3d & mat)
{
  if (!botframes_)
  {
    std::cout << "botframe is not initialized" << std::endl;
    mat = mat.matrix().Identity();
    return 0;
  }

  int status;
  double matx[16];
  status = bot_frames_get_trans_mat_4x4_with_utime( botframes_, from_frame.c_str(),  to_frame.c_str(), utime, matx);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      mat(i,j) = matx[i*4+j];
    }
  }
  return status;
}

void IRB140Estimator::update(double dt){
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  latest_cloud_mutex.lock();
  cloud = latest_cloud;
  latest_cloud_mutex.unlock();
  
  // the meat
  Eigen::Isometry3d to_kinect;
  long long utime;
  this->get_trans_with_utime("robot_yplus_tag", "KINECT_FROM_APRILTAG", utime, to_kinect);
  // visualize point cloud

  pcl::transformPointCloud(cloud, cloud, to_kinect.matrix());
  bot_lcmgl_point_size(lcmgl_, 4.5f);
  bot_lcmgl_color3f(lcmgl_, 0, 1, 0);
  int i = 0;
  bot_lcmgl_begin(lcmgl_, LCMGL_POINTS);
  for (auto pt = cloud.begin(); pt != cloud.end(); pt++){
    i = (i + 1) % 50;
    if (i == 0) {
      bot_lcmgl_vertex3f(lcmgl_, pt->x, pt->y, pt->z);
    }
  }
  bot_lcmgl_end(lcmgl_);

  bot_lcmgl_switch_buffer(lcmgl_);  

}


void IRB140Estimator::setupSubscriptions(){
  //lcm->subscribe("SCAN", &IRB140EstimatorSystem::handlePointlatest_cloud, this);
  //lcm.subscribe("SCAN", &IRB140Estimator::handlePlanarLidarMsg, this);
  //lcm.subscribe("PRE_SPINDLE_TO_POST_SPINDLE", &IRB140Estimator::handleSpindleFrameMsg, this);
  lcm.subscribe("KINECT_FRAME", &IRB140Estimator::handleKinectFrameMsg, this);
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

void IRB140Estimator::handleKinectFrameMsg(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const kinect::frame_msg_t* msg){
  printf("Received kinect frame on channel %s\n", chan.c_str());

  // only dealing with depth. Copied from ddKinectLCM... shouldn't 
  // this be in the Kinect driver or something?

  latest_cloud_mutex.lock();
  latest_cloud.clear();

  std::vector<uint16_t> depth_data;
  // 1.2.1 De-compress if necessary:
  if(msg->depth.compression != msg->depth.COMPRESSION_NONE) {
    printf("can't do this yet\n");
  }else{
    for (int i=0; i<msg->depth.depth_data.size()/2; i++)
      depth_data.push_back(  ((uint16_t)msg->depth.depth_data[2*i])+ (((uint16_t)msg->depth.depth_data[2*i+1])<<8) );
  }

  if(msg->depth.depth_data_format == msg->depth.DEPTH_MM  ){ 
    /////////////////////////////////////////////////////////////////////
    // Openni Data
    // 1.2.2 unpack raw byte data into float values in mm

    // NB: no depth return is given 0 range - and becomes 0,0,0 here
    latest_cloud.width    = msg->depth.width;
    latest_cloud.height   = msg->depth.height; 
    latest_cloud.is_dense = false;
    latest_cloud.points.resize (latest_cloud.width * latest_cloud.height);
    int j2=0;
    for(int v=0; v<msg->depth.height; v++) { // t2b self->height 480
      for(int u=0; u<msg->depth.width; u++ ) {  //l2r self->width 640
        // not dealing with color yet

        double constant = 1.0f / kcal->intrinsics_rgb.fx ;
        double disparity_d = depth_data[v*msg->depth.width+u]  / 1000.0; // convert to m

        if (disparity_d!=0){
          latest_cloud.points[j2].x = (((double) u)- kcal->intrinsics_depth.cx)*disparity_d*constant; //x right+
          latest_cloud.points[j2].y = (((double) v)- kcal->intrinsics_depth.cy)*disparity_d*constant; //y down+
          latest_cloud.points[j2].z = disparity_d;  //z forward+
          j2++;
        }

      }
    }
    latest_cloud.points.resize (j2);
  } else {
    printf("Can't unpack different Kinect data format yet.\n");
  }
  latest_cloud_mutex.unlock();

}