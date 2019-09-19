#pragma once

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

  void update_GPS_location_callback(const sensor_msgs::NavSatFix &msg);

private:
};
} // namespace agent
} // namespace sampling