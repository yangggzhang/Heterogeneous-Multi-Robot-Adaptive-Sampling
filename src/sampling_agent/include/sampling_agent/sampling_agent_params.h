#pragma once

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include <string>

namespace sampling {
namespace agent {

const double KBatteryLife_ROS_sec = 5000.0;
const double KMaxSpeed_ms = 2;
const double KRetreatPositionX_m = -1.0;
const double KRetreatPositionY_m = -1.0;

class SamplingAgentParams {
 public:
  SamplingAgentParams();

  bool LoadFromRosParams(ros::NodeHandle &ph);

  std::string agent_id;

  double max_speed_ms;

  double batterylife_ros_sec;

  geometry_msgs::Point retreat_position;

};  // namespace scene
}  // namespace agent
}  // namespace sampling