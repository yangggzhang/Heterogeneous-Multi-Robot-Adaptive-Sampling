#include "sampling_core/utils.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <sampling_core/RequestGoal.h>
#include <string>

namespace sampling {
class JackalNode {
public:
  JackalNode(const ros::NodeHandle &nh, const ros::NodeHandle &rh)
      : nh_(nh), rh_(rh) {
    if (!load_parameter()) {
      ROS_INFO_STREAM("Missing Jackal parameter!");
    };

    Jackal_action_client_ =
        new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
            Jackal_movebase_channel_, true);
    while (!Jackal_action_client_->waitForServer(ros::Duration(5.0))) {
      ROS_INFO_STREAM("Waiting for the move_base action server for "
                      << robot_id_ << " to come up");
    }
    request_target_client_ =
        nh_.serviceClient<sampling_core::RequestGoal>(request_target_channel_);
  }

  bool load_parameter() {

    if (!rh_.getParam("Jackal_movebase_channel", Jackal_movebase_channel_)) {
      ROS_INFO_STREAM("Error! Missing Jackal move base channel!");
      return false;
    }

    if (!rh_.getParam("Jackal_movebase_frame_id", Jackal_movebase_frame_id_)) {
      ROS_INFO_STREAM("Error! Missing Jackal move base goal frame!");
      return false;
    }

    if (!rh_.getParam("Jackal_request_target_channel",
                      request_target_channel_)) {
      ROS_INFO_STREAM("Error! Missing Jackal target requesting channel!");
      return false;
    }

    if (!rh_.getParam("Jackal_moving_duration_threshold_s",
                      Jackal_moving_duration_threshold_s_)) {
      ROS_INFO_STREAM("Error! Missing Jackal moving maximum duration!");
      return false;
    }

    Jackal_moving_duration_threshold_s_;
    return true;
  }

  bool request_target_from_master() {
    sampling_core::RequestGoal srv;
    srv.request.robot_id = robot_id_;
    srv.request.robot_latitude = current_location_.latitude;
    srv.request.robot_longitude = current_location_.longitude;

    if (request_target_client_.call(srv)) {
      ROS_INFO_STREAM("Robot " << robot_id_ << " received new target : ");
      ROS_INFO_STREAM("Latitude : " << srv.response.latitude << " Longitude : "
                                    << srv.response.longitude);
      gps_target_.latitude = srv.response.latitude;
      gps_target_.longitude = srv.response.longitude;
      return true;
    } else {
      ROS_INFO_STREAM("Robot "
                      << robot_id_
                      << " failed to request target from master computer!");
      return false;
    }
  }

  bool calculate_goal_from_gps(move_base_msgs::MoveBaseGoal &goal) {
    /// todo \yang use utm to transform gps goal to map goal
    goal.target_pose.header.frame_id = Jackal_movebase_frame_id_;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = gps_target_.latitude;
    goal.target_pose.pose.position.y = gps_target_.longitude;

    /// todo \yang calculate
    goal.target_pose.pose.orientation.w = 1.0;

    return true;
  }

  bool collect_sample() {
    /// request target from master computer w/ GPS
    while (!request_target_from_master()) {
      ROS_INFO_STREAM("Robot : "
                      << robot_id_
                      << " failed to request target from master computer");
    }

    move_base_msgs::MoveBaseGoal move_base_goal;
    /// Coordination transformation from GPS to map
    if (!calculate_goal_from_gps(move_base_goal)) {
      ROS_INFO_STREAM("Failed to transform GPS target to " << robot_id_
                                                           << " goal frame.");
      return false;
    }

    /// Infinite timing allowance rn
    Jackal_action_client_->sendGoal(move_base_goal);
    Jackal_action_client_->waitForResult();
    if (Jackal_action_client_->getState() ==
        actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO_STREAM("Hooray, robot " << robot_id_
                                       << " reached the target location!");
    else {
      ROS_INFO_STREAM("Robot "
                      << robot_id_
                      << " failed to reach the target location with state "
                      << Jackal_action_client_->getState().toString());
    }

    /// Collect temperature
  }

private:
  ros::NodeHandle nh_, rh_;
  ros::ServiceClient request_target_client_;
  std::string request_target_channel_;
  std::string Jackal_movebase_channel_;
  std::string Jackal_movebase_frame_id_;

  double Jackal_moving_duration_threshold_s_;

  std::string robot_id_;

  utils::gps_location current_location_;
  utils::gps_location gps_target_;
  utils::map_location map_target_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
      *Jackal_action_client_;
};
} // namespace sampling

int main(int argc, char **argv) {
  ros::init(argc, argv, "jackal_node");
  ros::NodeHandle nh, rh("~");
  ros::Rate r(10);
  sampling::JackalNode node(nh, rh);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
