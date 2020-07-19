#include "sampling_partition/heterogeneity.h"

#include <math.h> /* sqrt */

namespace sampling {
namespace partition {

std::unique_ptr<SamplingAgent> Heterogeneity::MakeUniqueFromROS(
    ros::NodeHandle &nh, const std::string &heterogeneity_type) {
  return nullptr;
}

double Heterogeneity::CalculateCost(const geometry_msgs::Point &agent_position,
                                    const geometry_msgs::Point &cell_position) {
  return 0;  // for compilation
}

inline double Heterogeneity::CalculateEuclideanDistance(
    const geometry_msgs::Point &agent_position,
    const geometry_msgs::Point &cell_position) {
  const double dx = agent_position.x - cell_position.x;
  const double dy = agent_position.y - cell_position.y;
  return sqrt(dx * dx + dy * dy);
}

Heterogeneity::Heterogeneity(const HeterogeneityParams &params)
    : params_(param) {}

}  // namespace partition
}  // namespace sampling