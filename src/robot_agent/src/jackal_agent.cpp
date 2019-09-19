#include "robot_agent/jackal_agent.h"

namespace sampling {
namespace agent {
JackalNode::JackalNode(const ros::NodeHandle &nh, const ros::NodeHandle &rh)
    : AgentNode(nh, rh) {
  if (!rh_.getParam("jackal_movebase_channel", jackal_movebase_channel_)) {
    ROS_ERROR("Error! Missing jackal movebase channel!");
  }

  if (!rh_.getParam("jackal_movebase_goal_frame_id",
                    jackal_movebase_goal_frame_id_)) {
    ROS_ERROR("Error! Missing jackal movebase goal frame id!");
  }

  if (!rh_.getParam("jackal_moving_duration_threshold_s",
                    jackal_moving_duration_threshold_s_)) {
    ROS_ERROR("Error! Missing jackal navigation time threshold!");
  }

  jackal_action_client_ =
      new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
          jackal_movebase_channel_, true);
  while (!jackal_action_client_->waitForServer(ros::Duration(5.0))) {
    ROS_INFO_STREAM(
        "Waiting for the move_base action server for jackal to come up");
  }
}

bool JackalNode::update_goal_from_gps() {
  /// to do \yang \yunfei
  move_base_msgs::MoveBaseGoal empty_goal;
  move_base_goal_ = empty_goal;
  move_base_goal_.target_pose.header.frame_id = jackal_movebase_goal_frame_id_;
  move_base_goal_.target_pose.header.stamp = ros::Time::now();

  /// to do Use utm to update gps to "map"
  move_base_goal_.target_pose.pose.position.x = goal_rtk_latitude_;
  move_base_goal_.target_pose.pose.position.y = goal_rtk_longitude_;

  /// todo \yang calculate
  move_base_goal_.target_pose.pose.orientation.w = 1.0;
};

bool JackalNode::navigate() {
  jackal_action_client_->sendGoal(move_base_goal_);
  jackal_action_client_->waitForResult(
      ros::Duration(jackal_moving_duration_threshold_s_));
  if (jackal_action_client_->getState() ==
      actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO_STREAM("Hooray, robot " << agent_id_
                                     << " reached the target location!");
    return true;
  } else {
    ROS_INFO_STREAM("Robot "
                    << agent_id_
                    << " failed to reach the target location with state "
                    << jackal_action_client_->getState().toString());
    return false;
  }
}
void JackalNode::update_GPS_location_callback(
    const sensor_msgs::NavSatFix &msg) {
  current_latitude_ = msg.latitude;
  current_longitude_ = msg.longitude;
}

} // namespace agent
} // namespace sampling