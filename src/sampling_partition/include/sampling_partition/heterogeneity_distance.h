#pragma once

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include "sampling_partition/heterogeneity.h"

namespace sampling {
namespace partition {

class HeterogeneityDistance : public Heterogeneity {
 public:
  HeterogeneityDistance() = delete;

  HeterogeneityDistance(const HeterogeneityParams &params,
                        const Eigen::MatrixXd &map);

  Eigen::VectorXd CalculateCost(const geometry_msgs::Point &agent_position,
                                const Eigen::VectorXd &distance) override;
};
}  // namespace partition
}  // namespace sampling