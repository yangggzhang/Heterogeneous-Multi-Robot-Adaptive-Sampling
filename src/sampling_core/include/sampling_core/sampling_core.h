#pragma once

#include <ros/ros.h>

#include "sampling_core/sampling_core_params.h"
#include "sampling_online_learning/online_learning_handler.h"
#include "sampling_partition/weighted_voronoi_partition.h"

namespace sampling {
namespace core {

// todo agent die

const std::string KModelingNamespace = "modeling/";

class SamplingCore {
 public:
  SamplingCore() = delete;

  static std::unique_ptr<SamplingCore> MakeUniqueFromRos(ros::NodeHandle &nh,
                                                         ros::NodeHandle &ph);

 private:
  SamplingCore(
      ros::NodeHandle &nh,
      std::unique_ptr<partition::WeightedVoronoiPartition> partition_handler,
      std::unique_ptr<learning::OnlineLearningHandler> learning_handler,
      const SamplingCoreParams &params);

  SamplingCoreParams params_;

  // Agent
  ros::Subscriber agent_location_subscriber_;

  ros::Subscriber sample_subscriber_;

  ros::ServiceClient modeling_add_sample_client_;

  ros::ServiceClient modeling_update_model_client_;

  ros::ServiceClient modeling_predict_client_;

  // Partition
  std::unique_ptr<partition::WeightedVoronoiPartition> partition_handler_;

  // Online Learning
  std::unique_ptr<learning::OnlineLearningHandler> learning_handler_;

  // Visualization
};
}  // namespace core
}  // namespace sampling
