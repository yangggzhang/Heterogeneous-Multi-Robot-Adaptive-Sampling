#include "sampling_partition/heterogeneity_distance.h"

#include <math.h>

namespace sampling {
namespace partition {

Eigen::VectorXd HeterogeneityDistance::CalculateCost(
    const geometry_msgs::Point &agent_position,
    const Eigen::VectorXd &distance) {
  Eigen::VectorXd cost = distance.array() * KDistancePrimitive;
  cost = cost.array().tanh();
  return cost;
}

HeterogeneityDistance::HeterogeneityDistance(const HeterogeneityParams &params,
                                             const Eigen::MatrixXd &map)
    : Heterogeneity(params, map) {}

}  // namespace partition
}  // namespace sampling