#pragma once

#include <std_msgs/String.h>

#include <Eigen/Dense>
#include <random>

#include "robot_agent/robot_agent.h"

/// todo \Paul \Yunfei
/// basic function of Pelican execution
/// reference robot_agent.h and jackal_agent.h

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

class PelicanAgent : public SamplingAgent {
 public:
  PelicanAgent() = delete;

  bool update_goal_from_gps();

  bool navigate();

  void update_GPS_location_callback(const sensor_msgs::NavSatFix &msg) override;

  bool gps_is_converged(const double &last_latitude,
                        const double &last_longitude,
                        const double &current_latitude,
                        const double &current_longitude,
                        const double &difference_threshold,
                        const int &buffer_size, int &count);

  bool waypoint_navigate(const double &latitude, const double &longitude,
                         const int &height, const double &converge_duration);

  bool ReportGPSService(sampling_msgs::RequestLocation::Request &req,
                        sampling_msgs::RequestLocation::Response &res) override;

  double getPoly(double x, double y);

  bool collect_temperature_sample() override;

  double getGroundTruth();

 private:
  PelicanAgent(const ros::NodeHandle &nh, const std::string &agent_id,
               const ros::NodeHandle &rh);

  ros::Publisher xb_command_publisher_;

  bool InitializePelican();

  double KHoverTime_s = 5.0;
  double KNavigationWaitTime_s = 5.0;
  double KNavigationTimeout_s = 20.0;
  double KHoverHeight_mm = 5000.0;
  double KMeasureHeight_mm = 3500.0;
  double KGPSConvergeThreshold_mm = 1500.0;
  int KGPSBufferSize = 5;
  int KNavigateLoopRate_hz = 10;

  double cmd_latitude_;
  double cmd_longitude_;
  double last_cmd_latitude_;
  double last_cmd_longitude_;
  double hover_height_mm_;
  double measure_height_mm_;
  double gps_converge_threshold_mm_;
  int converge_count_;
  int gps_buffer_size_;
  bool gps_converg_flag_;  // flag to detect the converge of GPS sensor
  int navigate_loop_rate_hz_;

  double hover_time_s_;
  double navigate_wait_time_s_;  // waiting time before detecting convergence
  double navigate_timeout_s_;
  double last_latitude_;
  double last_longitude_;

  double rtk_longitude_origin_;
  double rtk_latitude_origin_;

  double pelican_latitude_origin_;
  double pelican_longitude_origin_;

  bool get_ground_truth_;
  double observation_noise_std_;
  std::vector<double> poly_coeff_;
  std::default_random_engine generator;
  double lat_constant_;
  double lng_constant_;

  Eigen::Matrix2f calibration_matrix_, inverse_calibration_matrix_;
};
}  // namespace agent
}  // namespace sampling
