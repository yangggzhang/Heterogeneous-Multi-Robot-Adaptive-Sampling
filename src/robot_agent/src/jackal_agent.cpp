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
  ROS_INFO_STREAM("Finish Jackal Loading!");

  jackal_action_client_ =
      new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
          jackal_movebase_channel_, true);
  while (!jackal_action_client_->waitForServer(ros::Duration(5.0))) {
    ROS_INFO_STREAM(
        "Waiting for the move_base action server for jackal to come up");
  }
  ROS_INFO_STREAM("Jackal move base server came up! READY TO GO!!!");
}

bool JackalNode::update_goal_from_gps() {
  /// to do \yang \yunfei
  move_base_msgs::MoveBaseGoal empty_goal;
  move_base_goal_ = empty_goal;
  move_base_goal_.target_pose.header.frame_id = jackal_movebase_goal_frame_id_;
  move_base_goal_.target_pose.header.stamp = ros::Time::now();

  geometry_msgs::PointStamped utm_goal =
      GPStoUTM(goal_rtk_latitude_, goal_rtk_longitude_);
  geometry_msgs::PointStamped map_goal = UTMtoMapPoint(utm_goal);
  /// to do Use utm to update gps to "map"
  move_base_goal_.target_pose.pose.position = map_goal.point;

  /// todo \yang calculate
  move_base_goal_.target_pose.pose.orientation.w = 1.0;
  return true;
};

bool JackalNode::navigate() {
  jackal_action_client_->sendGoal(move_base_goal_);
  jackal_action_client_->waitForResult(
      ros::Duration(jackal_moving_duration_threshold_s_));
  if (jackal_action_client_->getState() ==
      actionlib::SimpleClientGoalState::SUCCEEDED) {
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

geometry_msgs::PointStamped JackalNode::GPStoUTM(const double &latitude,
                                                 const double &longitude) {
  double utm_x = 0, utm_y = 0;
  geometry_msgs::PointStamped UTM_point_output;
  std::string utm_zone;

  // convert lat/long to utm
  RobotLocalization::NavsatConversions::LLtoUTM(latitude, longitude, utm_y,
                                                utm_x, utm_zone);

  // Construct UTM_point and map_point geometry messages
  UTM_point_output.header.frame_id = "utm";
  UTM_point_output.header.stamp = ros::Time::now();
  UTM_point_output.point.x = utm_x;
  UTM_point_output.point.y = utm_y;
  UTM_point_output.point.z = 0;

  return UTM_point_output;
}

geometry_msgs::PointStamped JackalNode::UTMtoMapPoint(
    const geometry_msgs::PointStamped &UTM_input) {
  geometry_msgs::PointStamped map_point_output;
  bool notDone = true;
  tf::TransformListener
      listener;  // create transformlistener object called listener
  ros::Time time_now = ros::Time::now();
  while (notDone) {
    try {
      map_point_output.header.stamp = ros::Time::now();
      listener.waitForTransform(jackal_movebase_goal_frame_id_, "utm", time_now,
                                ros::Duration(3.0));
      listener.transformPoint(jackal_movebase_goal_frame_id_, UTM_input,
                              map_point_output);
      notDone = false;
    } catch (tf::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(0.01).sleep();
    }
  }
  return map_point_output;
}

}  // namespace agent
}  // namespace sampling
