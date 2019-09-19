#include "robot_agent/pelican_agent.h"

namespace sampling {
namespace agent {
PelicanNode::PelicanNode(const ros::NodeHandle &nh, const ros::NodeHandle &rh)
    : AgentNode(nh, rh) {
  /// todo \paul \yunfei
  /// load necessary parameters
}

bool PelicanNode::update_goal_from_gps() {
  /// todo \paul \yunfei
  /// transform gps signal from rtk frame to pelican local frame
  return true;
};

bool PelicanNode::navigate() {
  /// todo \paul \yunfei
  /// GPS waypoint navigation
  return true;
}
void PelicanNode::update_GPS_location_callback(
    const sensor_msgs::NavSatFix &msg) {
  /// todo \paul \yunfei
  /// update the local gps signal back to rtk frame
}

} // namespace agent
} // namespace sampling