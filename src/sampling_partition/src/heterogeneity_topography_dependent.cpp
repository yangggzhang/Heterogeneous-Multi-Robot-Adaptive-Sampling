#include "sampling_partition/heterogeneity_topography_depedent.h"

#include <math.h>

namespace sampling {
namespace partition {

double HeterogeneityDistanceDepedent::CalculateCost(
    const geometry_msgs::Point &agent_position,
    const geometry_msgs::Point &cell_position) override {
  for (const geometry_msgs::Point &control_center :
       params_.control_area_center) {
    if (CalculateEuclideanDistance(cell_position, control_center) <=
        params_.control_area_radius)
      return params_.weight_factor * params_.heterogeneity_primitive;
  }
  return 0.0;
}

HeterogeneityDistanceDepedent::HeterogeneityDistanceDepedent(
    const HeterogeneityParams &params)
    : Heterogeneity(param) {}

}  // namespace partition
}  // namespace sampling