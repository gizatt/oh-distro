#include "IRB140Estimator.hpp"
#include "drake/util/convexHull.h"
#include "zlib.h"

using namespace std;
using namespace Eigen;

// from https://forum.kde.org/viewtopic.php?f=74&t=91514
template<typename Derived>
inline bool is_finite(const Eigen::MatrixBase<Derived>& x)
{
   return ( (x - x).array() == (x - x).array()).all();
}
template<typename Derived>
inline bool is_nan(const Eigen::MatrixBase<Derived>& x)
{
   return ((x.array() == x.array())).all();
}

IRB140Estimator::IRB140Estimator(std::shared_ptr<RigidBodyTree> arm, std::shared_ptr<RigidBodyTree> manipuland, 
      Eigen::Matrix<double, Eigen::Dynamic, 1> x0_arm, Eigen::Matrix<double, Eigen::Dynamic, 1> x0_manipuland,
    const char* filename) :
    x_arm(x0_arm), x_manipuland(x0_manipuland), manipuland_kinematics_cache(manipuland->bodies)
{
  this->arm = arm;
  this->manipuland = manipuland;

  last_update_time = getUnixTime();

  if (!this->lcm.good()) {
    throw std::runtime_error("LCM is not good");
  }

  lcmgl_lidar_= bot_lcmgl_init(lcm.getUnderlyingLCM(), "trimmed_lidar");
  lcmgl_manipuland_= bot_lcmgl_init(lcm.getUnderlyingLCM(), "manipuland_se");
  lcmgl_icp_= bot_lcmgl_init(lcm.getUnderlyingLCM(), "icp_p2pl");
  lcmgl_measurement_model_ = bot_lcmgl_init(lcm.getUnderlyingLCM(), "meas_model");

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

  // generate sample points for doing sensor simulation
  // todo: verify vals / figure out how to regenerate raycast endpoints when we 
  // receive depth images and know the resolution
  double min_pitch = -22.5*3.1415/180.;
  double max_pitch = 22.5*3.1415/180.;
  double min_yaw = -27.0*3.1415/180.;
  double max_yaw = 27.0*3.1415/180.; // todo: verify this FOV vals
  double max_range = 30.0;
  raycast_endpoints.resize(3,num_pixel_rows*num_pixel_cols);
  for (size_t i=0; i<num_pixel_rows; i++) {
    double pitch = min_pitch + (num_pixel_rows>1 ? static_cast<double>(i)/(num_pixel_rows-1) : 0.0)*(max_pitch-min_pitch);
    for (size_t j=0; j<num_pixel_cols; j++) {
      double yaw = min_yaw + (num_pixel_cols>1 ? static_cast<double>(j)/(num_pixel_cols-1) : 0.0)*(max_yaw-min_yaw);
      //raycast_endpoints.col(num_pixel_cols*i + j) = max_range*Vector3d(cos(yaw)*cos(pitch), sin(yaw), -cos(yaw)*sin(pitch)); // rolled out from roty(pitch)*rotz(yaw)*[1;0;0]
      // instead, +z is forward, +y is down, +x is right.
      raycast_endpoints.col(num_pixel_cols*i + j) = max_range*Vector3d(sin(yaw), -sin(pitch)*cos(yaw), cos(yaw)*cos(pitch)); // rolled out from rotx(pitch)*roty(yaw)*[0;0;1]
    }
  }

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
  pcl::PointCloud<pcl::PointXYZRGB> full_cloud;
  Eigen::MatrixXd depth_image;
  latest_cloud_mutex.lock();
  full_cloud = latest_cloud;
  depth_image= latest_depth_image;
  latest_cloud_mutex.unlock();

  VectorXd q_old = x_manipuland.block(0, 0, manipuland->num_positions, 1);
  manipuland_kinematics_cache.initialize(q_old);
  manipuland->doKinematics(manipuland_kinematics_cache);
  
  // transform into world frame
  Eigen::Isometry3d kinect2tag;
  long long utime = 0;
  this->get_trans_with_utime("KINECT_RGB", "KINECT_TO_APRILTAG", utime, kinect2tag);
  Eigen::Isometry3d world2tag;
  long long utime2 = 0;
  this->get_trans_with_utime("local", "robot_yplus_tag", utime2, world2tag);
  Eigen::Isometry3d kinect2world =  world2tag.inverse() * kinect2tag;
  pcl::transformPointCloud(full_cloud, full_cloud, kinect2world.matrix());

  // cut down to just point cloud in our manipulation space
  // (todo: bring in this info externally somehow)

  double manip_x_bounds[2] = {0.45, 0.75};
  double manip_y_bounds[2] = {-0.1, 0.2};
  double manip_z_bounds[2] = {0.7, 1.05};

  Matrix3Xd points;
  points.resize(3, full_cloud.size());
  int i = 0;
  for (auto pt = full_cloud.begin(); pt != full_cloud.end(); pt++){
    if (pt->x > manip_x_bounds[0] && pt->x < manip_x_bounds[1] && 
        pt->y > manip_y_bounds[0] && pt->y < manip_y_bounds[1] && 
        pt->z > manip_z_bounds[0] && pt->z < manip_z_bounds[1]){
      points(0, i) = pt->x;
      points(1, i) = pt->y;
      points(2, i) = pt->z;
      i++;
      
    }
  }
  // conservativeResize keeps old coefficients
  // (regular resize would clear them)
  points.conservativeResize(3, i);


  double now=getUnixTime();
  
  // calculate SDFs in the image plane (not voxel grid like DART... too expensive
  // since we're not on a GPU yet)
  Eigen::MatrixXd observation_sdf;

  // perform raycast to generate "expected" observation
  // (borrowing code from Matthew Woehlke's pull request for the moment here)
  VectorXd distances(raycast_endpoints.cols());
  Vector3d origin = kinect2world*Vector3d::Zero();
  Matrix3Xd origins(3, raycast_endpoints.cols());
  for (int i=0; i < raycast_endpoints.cols(); i++)
    origins.block<3, 1>(0, i) = origin;

  auto raycast_endpoints_world = kinect2world*raycast_endpoints;
  manipuland->collisionRaycast(manipuland_kinematics_cache,origins,raycast_endpoints_world,distances);
  // initialize observation SDF to Inf where the measurement return is in front of the real return
  // and 0 otherwise
  observation_sdf.resize(num_pixel_rows, num_pixel_cols);
  observation_sdf *= 0.;
  for (size_t i=0; i<num_pixel_rows; i++) {
    for (size_t j=0; j<num_pixel_cols; j++) {
      if (distances(i*num_pixel_cols + j) > 0. && distances(i*num_pixel_cols + j) < depth_image(i*num_pixel_cols + j)){
        observation_sdf(i, j) = 1E6;
      }
    }
  }

  printf("elapsed in SDF: %f\n", getUnixTime() - now);

  // visualize measurement simulation
  bot_lcmgl_point_size(lcmgl_measurement_model_, 4.5f);
  bot_lcmgl_color3f(lcmgl_measurement_model_, 0, 0, 1);  
  bot_lcmgl_begin(lcmgl_measurement_model_, LCMGL_POINTS);
  for (i = 0; i < distances.rows(); i++){
    Vector3d endpt = origin + distances(i) * (raycast_endpoints_world.block<3, 1>(0, i) - origin)/30.; //warning, hardcoded scan max dist
    if (endpt(0) > manip_x_bounds[0] && endpt(0) < manip_x_bounds[1] && 
        endpt(1) > manip_y_bounds[0] && endpt(1) < manip_y_bounds[1] && 
        endpt(2) > manip_z_bounds[0] && endpt(2) < manip_z_bounds[1]){
      bot_lcmgl_vertex3f(lcmgl_measurement_model_, endpt(0), endpt(1), endpt(2));
    }
  }
  bot_lcmgl_end(lcmgl_measurement_model_);
  bot_lcmgl_switch_buffer(lcmgl_measurement_model_);  
  




  now=getUnixTime();
  this->performICPStep(points);
  printf("elapsed in ICP: %f\n", getUnixTime() - now);


  // visualize point cloud
  bot_lcmgl_point_size(lcmgl_lidar_, 4.5f);
  bot_lcmgl_color3f(lcmgl_lidar_, 0, 1, 0);
  
  bot_lcmgl_begin(lcmgl_lidar_, LCMGL_POINTS);
  for (i = 0; i < points.cols(); i++){
    if (i % 10 == 0) {
      bot_lcmgl_vertex3f(lcmgl_lidar_, points(0, i), points(1, i), points(2, i));
    }
  }
  bot_lcmgl_end(lcmgl_lidar_);
  bot_lcmgl_switch_buffer(lcmgl_lidar_);  

  // visualize manipuland
  bot_lcmgl_point_size(lcmgl_manipuland_, 4.5f);
  bot_lcmgl_color3f(lcmgl_manipuland_, 1, 0, 1);
  bot_lcmgl_push_matrix(lcmgl_manipuland_);
  bot_lcmgl_translated(lcmgl_manipuland_, x_manipuland[0], x_manipuland[1], x_manipuland[2]);
  bot_lcmgl_rotated(lcmgl_manipuland_,180./3.1415* x_manipuland[5], 0.0, 0.0, 1.0);
  bot_lcmgl_rotated(lcmgl_manipuland_, 180./3.1415*x_manipuland[4], 0.0, 1.0, 0.0);
  bot_lcmgl_rotated(lcmgl_manipuland_, 180./3.1415*x_manipuland[3], 1.0, 0.0, 0.0);
  //double zeros[3] = {0,0,-0.105};
  //bot_lcmgl_cylinder(lcmgl_manipuland_, zeros, 0.0325, 0.0325, 0.21,
  //      10, 10);
  double zeros[3] = {0,0,0};
  float size[3] = {0.14, 0.07, 0.225};
  bot_lcmgl_box(lcmgl_manipuland_, zeros, size);
  bot_lcmgl_pop_matrix(lcmgl_manipuland_);
  bot_lcmgl_color3f(lcmgl_manipuland_, 0, 1, 1);
  bot_lcmgl_push_matrix(lcmgl_manipuland_);
  bot_lcmgl_translated(lcmgl_manipuland_, x_manipuland[6], x_manipuland[7], x_manipuland[8]);
  bot_lcmgl_rotated(lcmgl_manipuland_,180./3.1415* x_manipuland[11], 0.0, 0.0, 1.0);
  bot_lcmgl_rotated(lcmgl_manipuland_, 180./3.1415*x_manipuland[10], 0.0, 1.0, 0.0);
  bot_lcmgl_rotated(lcmgl_manipuland_, 180./3.1415*x_manipuland[9], 1.0, 0.0, 0.0);
  float size_table[3] = {0.3, 0.3, 0.05};
  double zeros_table[3] = {0,0,0.};
  bot_lcmgl_box(lcmgl_manipuland_, zeros_table, size_table);
  bot_lcmgl_pop_matrix(lcmgl_manipuland_);
  bot_lcmgl_switch_buffer(lcmgl_manipuland_);  
}

void IRB140Estimator::performICPStep(Matrix3Xd& points){
  VectorXd phi;
  Matrix3Xd normal, x, body_x;
  std::vector<int> body_idx;
  int nq = manipuland->num_positions;
  VectorXd q_old = manipuland_kinematics_cache.getQ();

  // project all cloud points onto the surface of the object positions
  // via the last state estimate
  manipuland->signedDistances(manipuland_kinematics_cache, points,
                       phi, normal, x, body_x, body_idx, false);

  // set up a quadratic program:
  // 0.5 * x.' Q x + f.' x
  // and since we're unconstrained then solve as linear system
  // Qx = -f
  VectorXd f(nq);
  f *= 0.;
  MatrixXd Q(nq, nq);
  Q *= 0.;
  double K = 0.;

  // for every unique body points have returned onto...
  std::vector<int> num_points_on_body(manipuland->bodies.size(), 0);

  for (int i=0; i < body_idx.size(); i++)
    num_points_on_body[body_idx[i]] += 1;

  // for every body...
  for (int i=0; i < manipuland->bodies.size(); i++){
    if (num_points_on_body[i] > 0){

      // collect results from raycast that correspond to this sensor
      Matrix3Xd z(3, num_points_on_body[i]); // points, in world frame, near this body
      Matrix3Xd z_prime(3, num_points_on_body[i]); // same points projected onto surface of body
      Matrix3Xd body_z_prime(3, num_points_on_body[i]); // projected points in body frame
      Matrix3Xd z_norms(3, num_points_on_body[i]); // normals corresponding to these points
      int k = 0;
      for (int j=0; j < body_idx.size(); j++){
        if (body_idx[j] == i){
          z.block<3, 1>(0, k) = points.block<3, 1>(0, j);
          z_prime.block<3, 1>(0, k) = x.block<3, 1>(0, j);
          body_z_prime.block<3, 1>(0, k) = body_x.block<3, 1>(0, j);
          z_norms.block<3, 1>(0, k) = normal.block<3, 1>(0, j);
          k++;
        }
      }

      // forwardkin to get our jacobians at the project points on the body
      auto J = manipuland->transformPointsJacobian(manipuland_kinematics_cache, body_z_prime, i, 0, false);

      // apply point-to-plane cost
      // we're minimizing point-to-plane projected distance after moving the body config by delta_q
      // i.e. (z - z_prime_new).' * n
      //   =  (z - (z_prime + J*(q_new - q_old))) .' * n
      //   =  (z - z_prime - J*(q_new - q_old))) .' * n
      // Which, if we penalize quadratically, and expand out, removing constant terms, we get
      // argmin_{qn}[ qn.' * (J.' * n * n.' * J) * qn +
      //              - 2 * (Ks.' * n * n.' * J) ]
      // for Ks = (z - z_prime + Jz*q_old)

      bool POINT_TO_PLANE = false;

      for (int j=0; j < num_points_on_body[i]; j++){
        MatrixXd Ks = z.col(j) - z_prime.col(j) + J.block(3*j, 0, 3, nq)*q_old;
        if (POINT_TO_PLANE){
          //cout << z_norms.col(j).transpose() << endl;
          //cout << "Together: " << (z_norms.col(j) * z_norms.col(j).transpose()) << endl;
          f = f - (2. * Ks.transpose() * (z_norms.col(j) * z_norms.col(j).transpose()) * J.block(3*j, 0, 3, nq)).transpose()  / points.cols();
          Q = Q + (2. *  J.block(3*j, 0, 3, nq).transpose() * (z_norms.col(j) * z_norms.col(j).transpose()) * J.block(3*j, 0, 3, nq)) / points.cols();
        } else {
          f = f - (2. * Ks.transpose() * J.block(3*j, 0, 3, nq)).transpose()  / points.cols();
          Q = Q + (2. *  J.block(3*j, 0, 3, nq).transpose() * J.block(3*j, 0, 3, nq)) / points.cols();
        }
        K += Ks.squaredNorm() / points.cols();

        if (j % 10 == 0){
          // visualize point correspondences and normals
          double dist_normalized = fmin(1.0, (z.col(j) - z_prime.col(j)).norm());
          bot_lcmgl_color3f(lcmgl_icp_, dist_normalized*dist_normalized, 0, (1.0-dist_normalized)*(1.0-dist_normalized));
          
          bot_lcmgl_begin(lcmgl_icp_, LCMGL_LINES);
          bot_lcmgl_line_width(lcmgl_icp_, 2.0f);
          bot_lcmgl_vertex3f(lcmgl_icp_, z(0, j), z(1, j), z(2, j));
          bot_lcmgl_vertex3f(lcmgl_icp_, z_prime(0, j), z_prime(1, j), z_prime(2, j));
          bot_lcmgl_end(lcmgl_icp_);  

/*
          bot_lcmgl_line_width(lcmgl_icp_, 1.0f);
          bot_lcmgl_color3f(lcmgl_icp_, 1.0, 0.0, 1.0);
          bot_lcmgl_begin(lcmgl_icp_, LCMGL_LINES);
          bot_lcmgl_vertex3f(lcmgl_icp_, z_prime(0, j)+z_norms(0, j)*0.01, z_prime(1, j)+z_norms(1, j)*0.01, z_prime(2, j)+z_norms(2, j)*0.01);
          bot_lcmgl_vertex3f(lcmgl_icp_, z_prime(0, j), z_prime(1, j), z_prime(2, j));
          bot_lcmgl_end(lcmgl_icp_);  
*/
        }
      }
    }
  }

  if (K > 0.0){
  //  cout << "f: " << f << endl;
  //  cout << "Q: " << Q << endl;
  //  cout << "K: " << K << endl;
    // Solve the unconstrained QP!
    VectorXd q_new = Q.colPivHouseholderQr().solve(-f);
  //  cout << "q_new: " << q_new.transpose() << endl;
    x_manipuland.block(0, 0, nq, 1) = q_new;
    bot_lcmgl_switch_buffer(lcmgl_icp_);
  }

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
    // ugh random C code
    uint8_t * uncompress_buffer = (uint8_t*) malloc(msg->depth.uncompressed_size);
    unsigned long dlen = msg->depth.uncompressed_size;
    int status = uncompress(uncompress_buffer, &dlen, 
        msg->depth.depth_data.data(), msg->depth.depth_data_nbytes);
    if(status != Z_OK) {
      printf("Problem in uncompression.\n");
      free(uncompress_buffer);
      latest_cloud_mutex.unlock();
      return;
    }
    for (int i=0; i<msg->depth.uncompressed_size/2; i++)
      depth_data.push_back(  ((uint16_t)uncompress_buffer[2*i])+ (((uint16_t)uncompress_buffer[2*i+1])<<8) );
    free(uncompress_buffer);

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
    if (latest_depth_image.cols() != msg->depth.width && latest_depth_image.rows() != msg->depth.height){
      latest_depth_image.resize(msg->depth.height, msg->depth.width);
    }
    latest_depth_image *= 0.0;
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
          latest_depth_image(v, u) = disparity_d;
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