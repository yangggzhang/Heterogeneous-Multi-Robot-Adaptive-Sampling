#include "sampling_partition/heterogeneity_distance_dependent.h"

#include <math.h>

namespace sampling {
namespace partition {

Eigen::VectorXd HeterogeneityDistanceDepedent::CalculateCost(
    const geometry_msgs::Point &agent_position,
    const Eigen::VectorXd &distance) {
  Eigen::VectorXd cost = distance.array() * params_.heterogeneity_primitive;
  cost = cost.array().tanh();
  if (params_.heterogeneity_primitive >= 0) {
    return cost;
  } else {
    return cost.array() + 1.0;
  }
}

HeterogeneityDistanceDepedent::HeterogeneityDistanceDepedent(
    const HeterogeneityParams &params, const Eigen::MatrixXd &map)
    : Heterogeneity(params, map) {}

}  // namespace partition
}  // namespace sampling