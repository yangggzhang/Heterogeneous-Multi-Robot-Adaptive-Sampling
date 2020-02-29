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

  if (!rh_.getParam("facing_heatsource", facing_heatsource_)) {
    ROS_ERROR("Error! Missing if jackal need to face heat source!");
  }

  if (!rh_.getParam("heat_source_lat", heat_source_lat_)) {
    ROS_ERROR("Error! Missing heat_source_lat!");
  }

  if (!rh_.getParam("heat_source_lng", heat_source_lng_)) {
    ROS_ERROR("Error! Missing heat_source_lng!");
  }
  if (!rh_.getParam("poly_coeff", poly_coeff_)) {
    ROS_ERROR("Error! Missing poly_coeff");
  }
  assert(poly_coeff_.size() == 21);
  if (!rh_.getParam("get_ground_truth", get_ground_truth_)) {
    ROS_ERROR("Error! Missing get_ground_truth");
  }
  if (!rh_.getParam("observation_noise_std", observation_noise_std_)) {
    ROS_ERROR("Error! Missing observation noise std!");
  }
  if (!rh_.getParam("lat_constant", lat_constant_)) {
    ROS_ERROR("Error! Missing lat_constant!");
  }
  if (!rh_.getParam("lng_constant", lng_constant_)) {
    ROS_ERROR("Error! Missing observation noise std!");
  }

  jackal_action_client_ =
      new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
          jackal_movebase_channel_, true);
  while (!jackal_action_client_->waitForServer(ros::Duration(5.0))) {
    ROS_INFO_STREAM(
        "Waiting for the move_base action server for jackal to come up");
  }
  agent_state_ = REPORT;  // to test wifi,need to delete this !
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
  ROS_INFO_STREAM("Map goal : " << map_goal.point.x << " " << map_goal.point.y);
  /// todo \yang calculate
  if (facing_heatsource_) {
    tf::Matrix3x3 rot_euler;
    tf::Quaternion rot_quat;
    geometry_msgs::PointStamped utm_heatsource =
        GPStoUTM(heat_source_lat_, heat_source_lng_);
    geometry_msgs::PointStamped map_heatsource = UTMtoMapPoint(utm_heatsource);
    float delta_x = map_heatsource.point.x - map_goal.point.x;
    float delta_y =
        map_heatsource.point.y - map_goal.point.y;  // change in coords.
    float yaw_curr = atan2(delta_y, delta_x);
    float pitch_curr = 0;
    float roll_curr = 0;

    // Specify quaternions
    rot_euler.setEulerYPR(yaw_curr, pitch_curr, roll_curr);
    rot_euler.getRotation(rot_quat);
    move_base_goal_.target_pose.pose.orientation.x = rot_quat.getX();
    move_base_goal_.target_pose.pose.orientation.y = rot_quat.getY();
    move_base_goal_.target_pose.pose.orientation.z = rot_quat.getZ();
    move_base_goal_.target_pose.pose.orientation.w = rot_quat.getW();
  } else {
    move_base_goal_.target_pose.pose.orientation.w = 1.0;
  }
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
  current_latitude_ = (msg.latitude - lat_constant_) * pow(10, 5);
  current_longitude_ = (msg.longitude - lng_constant_) * pow(10, 5);
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
  ROS_INFO_STREAM("Goal frame : " << map_point_output.header.frame_id);
  return map_point_output;
}

double JackalNode::getGroundTruth() {
  double total_value = getPoly(current_latitude_, current_longitude_);
  return total_value;
}

bool JackalNode::collect_temperature_sample() {
  if (get_ground_truth_) {
    temperature_measurement_ = getGroundTruth();
    // add noise:
    std::normal_distribution<float> dist(
        0, observation_noise_std_);  // mean followed by stdiv
    temperature_measurement_ += dist(generator);
    return true;
  } else {
    sampling_msgs::RequestTemperatureMeasurement srv;
    srv.request.robot_id = agent_id_;

    if (temperature_measurement_client_.call(srv)) {
      temperature_measurement_ = srv.response.temperature;
      return true;
    } else {
      ROS_INFO_STREAM("Robot "
                      << agent_id_
                      << " failed to receive temperature measurement!");
      return false;
    }
  }
}

double JackalNode::getPoly(double x_, double y_) {
  double value;
  double x = x_ + 1, y = y_ + 1;
  value = poly_coeff_[0] + poly_coeff_[1] * x + poly_coeff_[2] * y +
          poly_coeff_[3] * pow(x, 2) + poly_coeff_[4] * x * y +
          poly_coeff_[5] * pow(y, 2) + poly_coeff_[6] * pow(x, 3) +
          poly_coeff_[7] * pow(x, 2) * y + poly_coeff_[8] * x * pow(y, 2) +
          poly_coeff_[9] * pow(y, 3) + poly_coeff_[10] * pow(x, 4) +
          poly_coeff_[11] * pow(x, 3) * y +
          poly_coeff_[12] * pow(x, 2) * pow(y, 2) +
          poly_coeff_[13] * x * pow(y, 3) + poly_coeff_[14] * pow(y, 4) +
          poly_coeff_[15] * pow(x, 5) + poly_coeff_[16] * pow(x, 4) * y +
          poly_coeff_[17] * pow(x, 3) * pow(y, 2) +
          poly_coeff_[18] * pow(x, 2) * pow(y, 3) +
          poly_coeff_[19] * x * pow(y, 4) + poly_coeff_[20] * pow(y, 5);
  return value;
}

}  // namespace agent
}  // namespace sampling
