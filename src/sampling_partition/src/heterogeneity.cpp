#include "sampling_partition/heterogeneity.h"

#include <math.h> /* sqrt */

#include "sampling_partition/heterogeneity_distance_dependent.h"
#include "sampling_partition/heterogeneity_topography_dependent.h"

namespace sampling {
namespace partition {

std::unique_ptr<Heterogeneity> Heterogeneity::MakeUniqueFromParam(
    const HeterogeneityParams &params, const Eigen::MatrixXd &map) {
  if (KHeterogeneitySpeed.compare(params.heterogeneity_type) == 0)
    return std::unique_ptr<Heterogeneity>(
        new HeterogeneityDistanceDepedent(params, map));
  else if (KHeterogeneityBatteryLife.compare(params.heterogeneity_type) == 0)
    return std::unique_ptr<Heterogeneity>(
        new HeterogeneityDistanceDepedent(params, map));
  else if (KHeterogeneityTraversability.compare(params.heterogeneity_type) == 0)
    return std::unique_ptr<Heterogeneity>(
        new HeterogeneityTopographyDepedent(params, map));
  else
    return nullptr;
}

Eigen::VectorXd Heterogeneity::CalculateCost(
    const geometry_msgs::Point &agent_position,
    const Eigen::VectorXd &distance) {
  return Eigen::VectorXd::Zero(distance.size());
}

Heterogeneity::Heterogeneity(const HeterogeneityParams &params,
                             const Eigen::MatrixXd &map)
    : params_(params), map_(map) {}

}  // namespace partition
}  // namespace sampling