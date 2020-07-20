/**
 * Utility functions for high dimensio voronoi construction
 * AUTHOR: Yang Zhang
 */

#pragma once
#include <geometry_msgs/Point.h>

#include <Eigen/Dense>
#include <boost/functional/hash.hpp>
#include <functional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "sampling/weighted_voronoi_partion_params.h"
#include "sampling_partition/heterogeneity.h"

namespace sampling {
namespace partition {

typedef std::pair<double, double> pair;

class WeightedVoronoiPartition {
 public:
  WeightedVoronoiPartition() = delete;

  static std::unique_ptr<WeightedVoronoiPartition> MakeUniqueFromRosParam(
      ros::NodeHandle &ph);

 private:
  std::unordered_map<std::string, std::vector<unique_ptr<Heterogeneity>>>
      heterogeneity_map_;

  Eigen::VectorXd CalculateEuclideanDistance(const geometry_msgs::Point &point,
                                             const Eigen::MatrixXd &map);

  //   Eigen::VectorXd Heterogeneity::CalculateEuclideanDistance(
  //       const geometry_msgs::Point &point, const Eigen::MatrixXd &map) {
  //     Eigen::MatrixXd distance_map(map.rows(), map.cols());
  //     distance_map.col(0).array() = map.col(0).array() - point.x;
  //     distance_map.col(1).array() = map.col(1).array() - point.y;
  //     return distance_map.rowwise().norm();
  //   }

  Eigen::MatrixXd map_;
};
}  // namespace partition
}  // namespace sampling