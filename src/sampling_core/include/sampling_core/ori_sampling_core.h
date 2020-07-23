#pragma once

#include <ros/ros.h>

#include "sampling_partition/weighted_voronoi_partition.h"
#include "sampling_utils/utils.h"
#include "sampling_visualization/"

namespace sampling {
namespace core {

class SamplingCore {
 public:
  SamplingCore() = delete;

  static std::unique_ptr<SamplingCore> MakeUniqueFromRos(os::NodeHandle &nh,
                                                         os::NodeHandle &ph);

  bool Init();

  bool AssignInterestPoint(sampling_msgs::SamplingGoal::Request &req,
                           sampling_msgs::SamplingGoal::Response &res);

  bool ParseFromRosParam();

  bool InitializeVisualization();

  void UpdateModel();

  void UpdateVisualization();

  void Update();

 private:
  SamplingCore();
};
}  // namespace core
}  // namespace sampling
