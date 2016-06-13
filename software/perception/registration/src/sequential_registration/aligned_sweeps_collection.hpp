#include <vector>
#include <Eigen/Geometry>

#include "sweep_scan.hpp"

#include <icp-registration/icp_utils.h>

// Point cloud class
class AlignedSweepsCollection 
{
  public:
    AlignedSweepsCollection();
    ~AlignedSweepsCollection();

    long long int getUtimeStart(){ return utime_start; };
    long long int getUtimeEnd(){ return utime_end; };
    int getNbClouds(){ return aligned_clouds.size(); };

    bool isEmpty(){ return !initialized_; };

    std::vector<SweepScan>& getClouds(){ return aligned_clouds; };
    PM::TransformationParameters& getConstraintToReference(){ return reference_to_current_cloud; };
    SweepScan& getReference(){ return aligned_clouds.front(); };
    SweepScan& getCurrentCloud(){ return aligned_clouds.back(); };
    SweepScan& getCloud(int index){ return aligned_clouds.at(index); };

    void initializeCollection(SweepScan reference);
    void addSweep(SweepScan current_aligned, PM::TransformationParameters T_ref_curr);

  private:
    bool initialized_;

    long long int utime_start; // Time stamp of the reference cloud
    long long int utime_end; // Time stamp of last cloud aligned

    std::vector<SweepScan> aligned_clouds;  // The cloud (field dp_cloud) transformed using
                                            // reference_to_current_cloud

    PM::TransformationParameters reference_to_current_cloud; // Current pose with respect to reference cloud
                                                  // constraint after drift correction 
    //Eigen::Isometry3d previous_to_current_cloud; // Current pose with respect to previous cloud (cloud_id-1)
                                                 // without drift correction 
};