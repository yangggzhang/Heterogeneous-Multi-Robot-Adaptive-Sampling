#pragma once

#include <geometry_msgs/Point.h>

#include <Eigen/Dense>

#include "sampling_partition/heterogeneity_params.h"

namespace sampling {
namespace partition {

const std::string KHomogeneityDistance = "DISTANCE";
const double KDistancePrimitive = 1.0;

const std::string KHeterogeneitySpeed = "SPEED";
const std::string KHeterogeneityBatteryLife = "BATTERY_LIFE";
const std::string KHeterogeneityTraversability = "TRAVERSABILITY";

class Heterogeneity {
 public:
  Heterogeneity() = delete;

  virtual Eigen::VectorXd CalculateCost(
      const geometry_msgs::Point &agent_position,
      const Eigen::VectorXd &distance) = 0;

  std::string GetType();

 protected:
  Heterogeneity(const HeterogeneityParams &params, const Eigen::MatrixXd &map);

  HeterogeneityParams params_;

  Eigen::MatrixXd map_;
};
}  // namespace partition
}  // namespace sampling