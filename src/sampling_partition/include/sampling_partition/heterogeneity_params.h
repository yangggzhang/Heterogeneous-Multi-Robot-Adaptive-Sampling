#pragma once

#include <geometry_msgs/Point.h>

#include <string>
#include <vector>

namespace sampling {
namespace partition {

class HeterogeneityParams {
 public:
  HeterogeneityParams(){};

  std::string heterogeneity_type;

  double weight_factor;

  double heterogeneity_primitive;

  std::vector<geometry_msgs::Point> control_area_center;

  std::vector<double> control_area_radius;
};
}  // namespace partition
}  // namespace sampling