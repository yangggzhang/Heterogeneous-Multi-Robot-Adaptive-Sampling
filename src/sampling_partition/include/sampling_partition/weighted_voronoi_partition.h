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

#include "sampling_partition/heterogeneity.h"
#include "sampling_partition/weighted_voronoi_partition_params.h"
#include "sampling_utils/utils.h"

namespace sampling {
namespace partition {

struct AgentLocation {
  std::string agent_id;
  geometry_msgs::Point position;
};

class WeightedVoronoiPartition {
 public:
  WeightedVoronoiPartition() = delete;

  static std::unique_ptr<WeightedVoronoiPartition> MakeUniqueFromRosParam(
      const std::unordered_set<std::string> &agent_ids,
      const Eigen::MatrixXd &map, ros::NodeHandle &ph);

  bool ComputePartition(
      const std::vector<AgentLocation> &location,
      std::unordered_map<std::string, std::vector<int>> &partition_index);

 private:
  WeightedVoronoiPartition(
      const WeightedVoronoiPartitionParam &params,
      const std::unordered_map<std::string,
                               std::vector<std::unique_ptr<Heterogeneity>>>
          &heterogeneity_map,
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