#pragma once

#include "robot_agent/robot_agent.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

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
};
} // namespace agent
} // namespace sampling