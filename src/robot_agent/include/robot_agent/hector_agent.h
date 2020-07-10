#pragma once

#include <hector_navigation/hector_navigation.h>
#include <nav_msgs/Odometry.h>

#include "robot_agent/hector_agent_params.h"
#include "robot_agent/robot_agent.h"

namespace sampling {
namespace agent {

using Hector = hector::navigation::HectorQuadrotor;

class HectorAgent : public SamplingAgent {
 public:
  HectorAgent() = delete;

  static std::unique_ptr<HectorAgent> MakeUniqueFromROSParam(
      ros::NodeHandle &nh);

 private:
  HectorAgent(ros::NodeHandle &nh, const std::string &agent_id,
              const HectorAgentParam &params,
              std::unique_ptr<Hector> hector_agent);

  std::unique_ptr<Hector> hector_agent_;

  HectorAgentParam params_;

  bool Navigate() override;

  ros::Subscriber odom_subscriber_;

  ros::ServiceClient hector_takeoff_client_;

  ros::ServiceClient hector_navigate_client_;

  void UpdatePositionFromOdom(const nav_msgs::Odometry &msg);

  bool taken_off_;
};
}  // namespace agent
}  // namespace sampling
