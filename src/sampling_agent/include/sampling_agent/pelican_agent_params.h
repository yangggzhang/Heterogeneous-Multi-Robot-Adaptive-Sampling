#pragma once

#include <ros/ros.h>

#include <string>
#include <vector>

namespace sampling {
namespace agent {

const double KHoverTime_s = 5.0;
const double KNavigationWaitTime_s = 5.0;
const double KNavigationTimeout_s = 20.0;
const double KHoverHeight_mm = 5000.0;
const double KMeasureHeight_mm = 3500.0;
const double KGPSConvergeThreshold_mm = 1500.0;
const int KGPSBufferSize = 5;
const int KNavigateLoopRate_hz = 10;
const double KGPSScaleFactor = 10e8;

class PelicanAgentParams {
 public:
  PelicanAgentParams();

  bool LoadFromRosParams(ros::NodeHandle &ph);

  double hover_time_s;

  double navigate_wait_time_s;  // waiting time before detecting convergence

  double navigate_timeout_s;

  double hover_height_mm;

  double measure_height_mm;

  double gps_converge_threshold_mm;

  int gps_buffer_size;

  int navigate_loop_rate_hz;

  double navigation_latitude_offset;

  double navigation_longitude_offset;

  double rtk_latitude_offset;

  double rtk_longitude_offset;

  double latitude_offset;

  double longitude_offset;

};  // namespace scene
}  // namespace agent
}  // namespace sampling