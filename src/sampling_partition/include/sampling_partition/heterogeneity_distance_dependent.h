#pragma once

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include "heterogeneity.h"

namespace sampling {
namespace partition {

class HeterogeneityDistanceDepedent : public Heterogeneity {
 public:
  HeterogeneityDistanceDepedent() = delete;

  HeterogeneityDistanceDepedent(const HeterogeneityParams &params);

  double CalculateCost(const geometry_msgs::Point &agent_position,
                       const geometry_msgs::Point &cell_position) override;
};
}  // namespace partition
}  // namespace sampling