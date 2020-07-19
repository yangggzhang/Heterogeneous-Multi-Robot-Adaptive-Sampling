#pragma once

#include <geometry_msgs/Point.h>

#include "sampling_partition/heterogeneity_params.h"

namespace sampling {
namespace partition {

const std::string KHeterogeneitySpeed = "SPEED";
const std::string KHeterogeneityBatteryLife = "BATTERY_LIFE";
const std::string KHeterogeneityTraversability = "TRAVERSABILITY";

class Heterogeneity {
 public:
  Heterogeneity() = delete;

  static std::unique_ptr<Heterogeneity> MakeUniqueFromParam(
      const HeterogeneityParams &params);

  virtual double CalculateCost(const geometry_msgs::Point &agent_position,
                               const geometry_msgs::Point &cell_position);

 protected:
  Heterogeneity(const HeterogeneityParams &params);

  HeterogeneityParams params_;

  inline double CalculateEuclideanDistance(const geometry_msgs::Point &point1,
                                           const geometry_msgs::Point &point2);

  std::string heterogeneity_type_;
};
}  // namespace partition
}  // namespace sampling