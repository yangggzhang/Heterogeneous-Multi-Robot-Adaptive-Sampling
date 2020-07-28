#include "sampling_agent/sampling_agent_params.h"

#include <ros/ros.h>

namespace sampling {
namespace agent {

SamplingAgentParams::SamplingAgentParams() {}

bool SamplingAgentParams::LoadFromRosParams(ros::NodeHandle& ph) {
  if (!ph.getParam("agent_id", agent_id)) {
    ROS_ERROR_STREAM("Missing agent id!");
    return false;
  }
  ph.param<double>("max_speed_ms", max_speed_ms, KMaxSpeed_ms);

  ph.param<double>("batterylife_ros_sec", batterylife_ros_sec,
                   KBatteryLife_ROS_sec);

  return true;
}

}  // namespace agent
}  // namespace sampling