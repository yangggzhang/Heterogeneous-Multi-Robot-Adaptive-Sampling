#pragma once

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_listener.h>

#include "sampling_agent/jackal_agent_params.h"
#include "sampling_agent/sampling_agent.h"

namespace sampling {
namespace agent {

using JackalNavigator =
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

class JackalAgent : public SamplingAgent {
 public:
  JackalAgent() = delete;

  static std::unique_ptr<JackalAgent> MakeUniqueFromROSParam(
      ros::NodeHandle &nh, ros::NodeHandle &ph);

 private:
  JackalAgent(ros::NodeHandle &nh, const SamplingAgentParams &agent_params,
              const JackalAgentParams &jackal_params,
              std::unique_ptr<JackalNavigator> jackal_navigator);

  bool Navigate() override;

  JackalAgentParams jackal_params_;

  tf::TransformListener listener_;

  ros::Subscriber odom_subscriber_;

  void UpdatePositionFromOdom(const nav_msgs::Odometry &msg);

  bool GPStoOdom(const double &latitude, const double &longitude,
                 geometry_msgs::PointStamped &odom_point);

  ros::Subscriber gps_subscriber_;

  void UpdatePositionFromGPS(const sensor_msgs::NavSatFix &msg);

  std::unique_ptr<JackalNavigator> jackal_navigator_;
};
}  // namespace agent
}  // namespace sampling
