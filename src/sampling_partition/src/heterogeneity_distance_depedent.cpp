#include "sampling_partition/heterogeneity_distance_dependent.h"

namespace sampling {
namespace partition {

double HeterogeneityDistanceDepedent::CalculateCost(
    const geometry_msgs::Point &agent_position,
    const geometry_msgs::Point &cell_position) override {
  return 0;  // for compilation
}

HeterogeneityDistanceDepedent::HeterogeneityDistanceDepedent(
    const HeterogeneityParams &params)
    : Heterogeneity(param) {}

}  // namespace partition
}  // namespace sampling