/**
 * Utility functions for high dimensio voronoi construction
 * AUTHOR: Yang Zhang
 */

#pragma once
#include <geometry_msgs/Point.h>

#include <Eigen/Dense>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "sampling_msgs/AgentLocation.h"
#include "sampling_partition/heterogeneity.h"
#include "sampling_partition/weighted_voronoi_partition_params.h"

namespace sampling {
namespace partition {

class WeightedVoronoiPartition {
 public:
  WeightedVoronoiPartition() = delete;

  static std::unique_ptr<WeightedVoronoiPartition> MakeUniqueFromRosParam(
      const std::vector<std::string> &agent_ids, const Eigen::MatrixXd &map,
      ros::NodeHandle &ph);

  bool ComputePartitionForAgent(
      const std::string &agent_id,
      const std::vector<sampling_msgs::AgentLocation> &location,
      std::vector<int> &partition_index);

  bool ComputePartitionForMap(
      const std::vector<sampling_msgs::AgentLocation> &location,
      std::vector<int> &index_for_map);

 private:
  WeightedVoronoiPartition(
      const WeightedVoronoiPartitionParam &params,
      const std::unordered_map<std::string, std::vector<HeterogeneityParams>>
          &heterogeneity_param_map,
      const Eigen::MatrixXd &map);

  Eigen::VectorXd CalculateEuclideanDistance(const geometry_msgs::Point &point,
                                             const Eigen::MatrixXd &map);

  WeightedVoronoiPartitionParam params_;

  std::unordered_map<std::string, std::vector<std::unique_ptr<Heterogeneity>>>
      heterogeneity_map_;

  Eigen::MatrixXd map_;
};
}  // namespace partition
}  // namespace sampling