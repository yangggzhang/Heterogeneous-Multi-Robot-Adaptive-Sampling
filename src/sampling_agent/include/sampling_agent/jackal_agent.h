#pragma once

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_listener.h>

#include "sampling_agent/sampling_agent.h"

namespace sampling {
namespace agent {

enum JackalNavigationMode { GPS, ODOM };

using JackalNavigator =
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

const std::string KWorldFrame = "map";

class JackalAgent : public SamplingAgent {
 public:
  JackalAgent() = delete;

  static std::unique_ptr<JackalAgent> MakeUniqueFromROS(
      ros::NodeHandle &nh, const std::string &agent_id);

 private:
  JackalAgent(ros::NodeHandle &nh, const std::string &agent_id,
              std::unique_ptr<JackalNavigator> jackal_navigator,
              const JackalNavigationMode &navigation_mode);

  bool Navigate() override;

  tf::TransformListener listener_;

  ros::Subscriber odom_subscriber_;

  void UpdatePositionFromOdom(const nav_msgs::Odometry &msg);

  bool GPStoOdom(const double &latitude, const double &longitude,
                 geometry_msgs::PointStamped &odom_point);

  ros::Subscriber gps_subscriber_;

  void UpdatePositionFromGPS(const sensor_msgs::NavSatFix &msg);

  std::unique_ptr<JackalNavigator> jackal_navigator_;

  double execute_timeout_s_;

  double preempt_timeout_s_;

  JackalNavigationMode navigation_mode_;
};
}  // namespace agent
}  // namespace sampling
