#include <math.h>

#include "sampling_partition/heterogeneity_distance_dependent.h"

namespace sampling {
namespace partition {

double HeterogeneityDistanceDepedent::CalculateCost(
    const geometry_msgs::Point &agent_position,
    const geometry_msgs::Point &cell_position) override {
  const double distance =
      CalculateEuclideanDistance(agent_position, cell_position);
  double cost =
      params_.weight_factor * tanh(distance * params_.heterogeneity_primitive);
  if (params_.heterogeneity_primitive >= 0) {
    return cost;
  } else {
    return cost + 1;
  }
}

HeterogeneityDistanceDepedent::HeterogeneityDistanceDepedent(
    const HeterogeneityParams &params)
    : Heterogeneity(param) {}

}  // namespace partition
}  // namespace sampling