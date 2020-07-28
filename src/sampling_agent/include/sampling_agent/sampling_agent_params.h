#pragma once

#include <ros/ros.h>

#include <string>

namespace sampling {
namespace agent {

const double KBatteryLife_ROS_sec = 5000.0;
const double KMaxSpeed_ms = 2;

class SamplingAgentParams {
 public:
  SamplingAgentParams();

  bool LoadFromRosParams(ros::NodeHandle &ph);

  std::string agent_id;

  double max_speed_ms;

  double batterylife_ros_sec;

};  // namespace scene
}  // namespace agent
}  // namespace sampling