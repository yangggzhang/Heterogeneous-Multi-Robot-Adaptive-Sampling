#pragma once

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <robot_localization/navsat_conversions.h>
#include <tf/transform_listener.h>
#include "robot_agent/robot_agent.h"

namespace sampling {
namespace agent {

class JackalNode : public AgentNode {
 public:
  JackalNode(){};

  JackalNode(const ros::NodeHandle &nh, const ros::NodeHandle &rh);

  bool update_goal_from_gps();

  bool navigate();

  void update_GPS_location_callback(const sensor_msgs::NavSatFix &msg) override;

 private:
  std::string jackal_movebase_channel_;
  std::string jackal_movebase_goal_frame_id_;
  double jackal_moving_duration_threshold_s_;
  move_base_msgs::MoveBaseGoal move_base_goal_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
      *jackal_action_client_;
  bool facing_heatsource_;
  double heat_source_lat_;
  double heat_source_lng_;

  geometry_msgs::PointStamped GPStoUTM(const double &latitude,
                                       const double &longitude);

  geometry_msgs::PointStamped UTMtoMapPoint(
      const geometry_msgs::PointStamped &UTM_input);
};
}  // namespace agent
}  // namespace sampling