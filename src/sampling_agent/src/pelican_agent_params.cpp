#include "sampling_agent/pelican_agent_params.h"

namespace sampling {
namespace agent {

PelicanAgentParams::PelicanAgentParams() {}

bool PelicanAgentParams::LoadFromRosParams(ros::NodeHandle &ph) {
  ph.param<double>("hover_time_s", hover_time_s, KHoverTime_s);

  ph.param<double>("navigate_wait_time_s", navigate_wait_time_s,
                   KNavigationWaitTime_s);

  ph.param<double>("navigate_timeout_s", navigate_timeout_s,
                   KNavigationTimeout_s);

  ph.param<double>("hover_height_mm", hover_height_mm, KHoverHeight_mm);

  ph.param<double>("measure_height_mm", measure_height_mm, KMeasureHeight_mm);

  ph.param<double>("gps_converge_threshold_mm", gps_converge_threshold_mm,
                   KGPSConvergeThreshold_mm);

  ph.param<int>("gps_buffer_size", gps_buffer_size, KGPSBufferSize);

  ph.param<int>("navigate_loop_rate_hz", navigate_loop_rate_hz,
                KNavigateLoopRate_hz);

  if (!ph.getParam("navigation_latitude_offset", navigation_latitude_offset)) {
    ROS_ERROR("Error! Missing pelican navigation latitude offset!");
    return false;
  }

  if (!ph.getParam("navigation_longitude_offset",
                   navigation_longitude_offset)) {
    ROS_ERROR("Error! Missing pelican navigation longitude offset!");
    return false;
  }

  if (!ph.getParam("rtk_latitude_offset", rtk_latitude_offset)) {
    ROS_ERROR("Error! Missing rtk latitude offset!");
    return false;
  }

  if (!ph.getParam("rtk_longitude_offset", rtk_longitude_offset)) {
    ROS_ERROR("Error! Missing rtk longitude offset!");
    return false;
  }

  if (!ph.getParam("latitude_offset", latitude_offset)) {
    ROS_ERROR("Error! Missing latitude offset!");
    return false;
  }

  if (!ph.getParam("longitude_offset", longitude_offset)) {
    ROS_ERROR("Error! Missing longitude offset!");
    return false;
  }

  return true;
}

}  // namespace agent
}  // namespace sampling