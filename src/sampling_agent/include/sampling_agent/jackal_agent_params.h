#pragma once

#include <ros/ros.h>

#include <string>

namespace sampling {
namespace agent {

enum JackalNavigationMode { GPS, ODOM };

const std::string KJackalWorldFrame = "map";
const std::string KJackalNavigationMode = "ODOM";
const double KJackalMaxSpeed_ms = 1.5;
const double KJackalExecuteTimeout_s = 30.0;
const double KJackalPreemptTimeout_s = 15.0;

class JackalAgentParams {
 public:
  JackalAgentParams();

  bool LoadFromRosParams(ros::NodeHandle &ph);

  std::string navigation_frame;

  JackalNavigationMode navigation_mode;

  double execute_timeout_s;

  double preempt_timeout_s;

};  // namespace scene
}  // namespace agent
}  // namespace sampling