#include "sampling_partition/heterogeneity.h"

namespace sampling {
namespace partition {

std::unique_ptr<SamplingAgent> Heterogeneity::MakeUniqueFromROS(
    ros::NodeHandle &nh, const std::string &heterogeneity_type) {
  return nullptr;
}

virtual double CalculateCost(const geometry_msgs::Point &agent_position,
                             const geometry_msgs::Point &cell_position);

protected:
Heterogeneity(ros::NodeHandle &nh, const std::string &heterogeneity_type);

std::string heterogeneity_type_;
};  // namespace partition
}  // namespace sampling
}  // namespace sampling