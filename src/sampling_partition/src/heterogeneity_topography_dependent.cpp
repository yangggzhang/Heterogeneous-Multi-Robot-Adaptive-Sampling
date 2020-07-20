#include "sampling_partition/heterogeneity_topography_dependent.h"

#include <math.h>

namespace sampling {
namespace partition {

Eigen::VectorXd HeterogeneityTopographyDepedent::CalculateCost(
    const geometry_msgs::Point &agent_position,
    const Eigen::VectorXd &distance) {
  return topography_cost_;
}

HeterogeneityTopographyDepedent::HeterogeneityTopographyDepedent(
    const HeterogeneityParams &params, const Eigen::MatrixXd &map)
    : Heterogeneity(params, map),
      topography_cost_(Eigen::VectorXd::Zero(map.rows())) {
  for (int i = 0; i < params_.control_area_center.size(); ++i) {
    geometry_msgs::Point center = params_.control_area_center[i];
    Eigen::MatrixXd distance_map(map_.rows(), map_.cols());
    distance_map.col(0).array() = map_.col(0).array() - center.x;
    distance_map.col(1).array() = map_.col(1).array() - center.y;
    Eigen::VectorXd distance = distance_map.rowwise().norm();
    for (int j = 0; j < distance.size(); ++j) {
      if (distance(j) <= params_.control_area_radius[j]) {
        topography_cost_(j) = params_.heterogeneity_primitive;
      }
    }
  }
}

}  // namespace partition
}  // namespace sampling