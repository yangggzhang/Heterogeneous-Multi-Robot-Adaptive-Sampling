#include "sampling_partition/heterogeneity.h"

#include <math.h> /* sqrt */

namespace sampling {
namespace partition {

std::unique_ptr<Heterogeneity> Heterogeneity::MakeUniqueFromParam(
    const HeterogeneityParams &params) {
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