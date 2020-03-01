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

class PelicanNode : public AgentNode {
 public:
  PelicanNode(){};

  PelicanNode(const ros::NodeHandle &nh, const ros::NodeHandle &rh);

  bool update_goal_from_gps();

  bool navigate();

  void update_GPS_location_callback(const sensor_msgs::NavSatFix &msg) override;

  bool initialize_pelican();

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
  ros::Publisher xb_command_pub_;
  std::string xb_command_channel_;

  double cmd_latitude_;
  double cmd_longitude_;
  double last_cmd_latitude_;
  double last_cmd_longitude_;
  double hover_height_;
  double measure_height_;
  double gps_converge_threshold_;
  int converge_count_;
  int gps_converge_buffer_size_;
  bool gps_converg_flag_;  // flag to detect the converge of GPS sensor
  int nagivate_loop_rate_int_;

  double height_waiting_threshold_;
  double
      navigate_waiting_threshold_;  // waiting time before detecting convergence
  double maximum_navigation_time_;
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
