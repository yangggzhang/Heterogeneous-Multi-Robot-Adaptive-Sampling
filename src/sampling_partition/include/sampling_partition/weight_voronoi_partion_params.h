#pragma once

#include <ros/ros.h>

#include <string>

#include "sampling_utils/utils.h"

namespace sampling {
namespace partition {

enum Heterogenity { DISTANCE, SPEED, BATTERYLIFE, TRAVERSABILITY };

class WeightedVoronoiPartitionParam {
 public:
  WeightedVoronoiPartitionParam();

  bool LoadFromRosParams(ros::NodeHandle &ph);

  //   std::string navigation_frame;

  //   double navigation_height_m;

  //   double navigation_speed_ms;

  //   double takeoff_distance_m;
};
}  // namespace partition
}  // namespace sampling