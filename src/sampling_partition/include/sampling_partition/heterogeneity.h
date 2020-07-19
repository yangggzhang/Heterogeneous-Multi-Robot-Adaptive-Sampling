#pragma once

#include <geometry_msgs/Point.h>

#include "heterogeneity_params.h"

namespace sampling {
namespace partition {

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

  inline double CalculateEuclideanDistance(
      const geometry_msgs::Point &agent_position,
      const geometry_msgs::Point &cell_position);

  std::string heterogeneity_type_;
};
}  // namespace partition
}  // namespace sampling