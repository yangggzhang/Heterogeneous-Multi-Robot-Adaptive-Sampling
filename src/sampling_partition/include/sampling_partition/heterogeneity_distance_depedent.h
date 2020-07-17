#pragma once

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include "heterogeneity.h"

namespace sampling {
namespace partition {

class Heterogeneity {
 public:
  Heterogeneity() = delete;

  static std::unique_ptr<SamplingAgent> MakeUniqueFromROS(
      ros::NodeHandle &nh, const std::string &heterogeneity_type);

  virtual double CalculateCost(const geometry_msgs::Point &agent_position,
                               const geometry_msgs::Point &cell_position);

 protected:
  Heterogeneity(ros::NodeHandle &nh, const std::string &heterogeneity_type);

  std::string heterogeneity_type_;
};
}  // namespace partition
}  // namespace sampling