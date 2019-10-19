#pragma once

#include "robot_agent/robot_agent.h"
#include <std_msgs/String.h>

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
  bool gps_converg_flag_; // flag to detect the converge of GPS sensor
  int nagivate_loop_rate_int_;

  double height_waiting_threshold_;
  double navigate_waiting_threshold_; //waiting time before detecting convergence
  double maximum_navigation_time_;
  double last_latitude_;
  double last_longitude_;

  double longitude_origin_;
  double latitude_origin_;
};
} // namespace agent
} // namespace sampling