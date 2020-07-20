#pragma once

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include "heterogeneity.h"

namespace sampling {
namespace partition {

class HeterogeneityDistanceDepedent : public Heterogeneity {
 public:
  HeterogeneityDistanceDepedent() = delete;

  HeterogeneityDistanceDepedent(const HeterogeneityParams &params,
                                const Eigen::MatrixXd &map);

  Eigen::VectorXd CalculateCost(const geometry_msgs::Point &agent_position,
                                const Eigen::VectorXd &distance) override;
};
}  // namespace partition
}  // namespace sampling