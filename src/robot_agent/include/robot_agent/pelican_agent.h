#pragma once

#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>

#include "robot_agent/pelican_agent_params.h"
#include "robot_agent/robot_agent.h"

/// todo \Paul \Yunfei
/// basic function of Pelican execution
/// reference robot_agent.h and jackal_agent.h

namespace sampling {
namespace agent {

// Pelican only supports GPS navigation mode

class PelicanAgent : public SamplingAgent {
 public:
  PelicanAgent() = delete;

  static std::unique_ptr<PelicanAgent> MakeUniqueFromROSParam(
      ros::NodeHandle &nh);

 private:
  PelicanAgent(ros::NodeHandle &nh, const std::string &agent_id,
               const PelicanAgentParam &params);

  ros::Publisher xb_command_publisher_;

  ros::Subscriber gps_subscriber_;

  void UpdatePositionFromGPS(const sensor_msgs::NavSatFix &msg);

  void CheckConvergence();

  bool WaypointNavigate(const double &latitude, const double &longitude,
                        const double &height, const double &converge_duration);

  bool Navigate() override;

  PelicanAgentParam params_;

  double cmd_latitude_;
  double cmd_longitude_;
  double last_cmd_latitude_;
  double last_cmd_longitude_;

  int converge_count_;

  bool gps_converge_flag_;  // flag to detect the converge of GPS sensor

  boost::optional<geometry_msgs::Point> current_gps_position_;

  boost::optional<geometry_msgs::Point> last_gps_position_;
};
}  // namespace agent
}  // namespace sampling
