#include "sampling_partition/heterogeneity.h"

#include <math.h> /* sqrt */

#include "sampling_partition/heterogeneity_distance_dependent.h"
#include "sampling_partition/heterogeneity_topography_dependent.h"

namespace sampling {
namespace partition {

std::unique_ptr<Heterogeneity> Heterogeneity::MakeUniqueFromParam(
    const HeterogeneityParams &params) {
  if (KHeterogeneitySpeed.compare(params.heterogeneity_type) == 0)
    return std::unique_ptr<Heterogeneity>(
        new HeterogeneityDistanceDepedent(params));
  else if (KHeterogeneityBatteryLife.compare(params.heterogeneity_type) == 0)
    return std::unique_ptr<Heterogeneity>(
        new HeterogeneityDistanceDepedent(params));
  else if (KHeterogeneityTraversability.compare(params.heterogeneity_type) == 0)
    return std::unique_ptr<Heterogeneity>(
        new HeterogeneityTopographyDepedent(params));
  else
    return nullptr;
}

double Heterogeneity::CalculateCost(const geometry_msgs::Point &agent_position,
                                    const geometry_msgs::Point &cell_position) {
  return 0;  // for compilation
}

inline double Heterogeneity::CalculateEuclideanDistance(
    const geometry_msgs::Point &point1, const geometry_msgs::Point &point2) {
  const double dx = point1.x - point2.x;
  const double dy = point1.y - point2.y;
  return sqrt(dx * dx + dy * dy);
}

Heterogeneity::Heterogeneity(const HeterogeneityParams &params)
    : params_(params) {}

}  // namespace partition
}  // namespace sampling