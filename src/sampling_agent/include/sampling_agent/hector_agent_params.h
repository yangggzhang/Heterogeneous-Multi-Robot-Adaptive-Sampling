#pragma once

#include <ros/ros.h>

#include <string>

namespace sampling {
namespace agent {

const std::string KNavigationFrame = "world";
const double KNavigationHeight_m = 2.0;
const double KNavigationSpeed_ms = 4.0;
const double KTakeoffDistance_m = 0.5;

class HectorAgentParams {
 public:
  HectorAgentParams();

  bool LoadFromRosParams(ros::NodeHandle &ph);

  std::string navigation_frame;

  double navigation_height_m;

  double navigation_speed_ms;

  double takeoff_distance_m;

};  // namespace scene
}  // namespace agent
}  // namespace sampling