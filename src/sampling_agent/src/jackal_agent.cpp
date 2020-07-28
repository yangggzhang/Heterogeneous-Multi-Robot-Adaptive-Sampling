#include "sampling_agent/jackal_agent.h"

#include <robot_localization/navsat_conversions.h>

namespace sampling {
namespace agent {

JackalAgent::JackalAgent(ros::NodeHandle &nh,
                         const SamplingAgentParams &agent_params,
                         const JackalAgentParams &jackal_params,
                         std::unique_ptr<JackalNavigator> jackal_navigator)
    : SamplingAgent(nh, agent_params),
      jackal_params_(jackal_params),
      jackal_navigator_(std::move(jackal_navigator)) {
  switch (jackal_params.navigation_mode) {
    case ODOM: {
      odom_subscriber_ =
          nh.subscribe(agent_params.agent_id + "/odometry/local_filtered", 1,
                       &JackalAgent::UpdatePositionFromOdom, this);
      break;
    }
    case GPS: {
      gps_subscriber_ = nh.subscribe(agent_params.agent_id + "/navsat/fix", 1,
                                     &JackalAgent::UpdatePositionFromGPS, this);
      break;
    }
    default:
      break;
  }

  nh.setParam("/" + agent_params.agent_id +
                  "/jackal_velocity_controller/linear/x/max_velocity",
              jackal_params.max_speed_ms);

  nh.setParam(
      "/" + agent_params.agent_id + "/move_base/TrajectoryPlannerROS/max_vel_x",
      jackal_params.max_speed_ms);
}

std::unique_ptr<JackalAgent> JackalAgent::MakeUniqueFromROSParam(
    ros::NodeHandle &nh, ros::NodeHandle &ph) {
  SamplingAgentParams agent_params;
  if (!agent_params.LoadFromRosParams(ph)) {
    ROS_ERROR("Failed to load agent parameters for hector agent!");
    return nullptr;
  }

  JackalAgentParams jackal_params;
  if (!jackal_params.LoadFromRosParams(ph)) {
    ROS_ERROR_STREAM(
        "Failed to load parameters for agent : " << agent_params.agent_id);
    return nullptr;
  }
  std::unique_ptr<JackalNavigator> jackal_navigator =
      std::unique_ptr<JackalNavigator>(
          new JackalNavigator(agent_params.agent_id + "/move_base", true));
  if (!jackal_navigator->waitForServer(ros::Duration(10.0))) {
    ROS_INFO_STREAM("Missing move base action for Jackal!");
    return nullptr;
  }

  return std::unique_ptr<JackalAgent>(new JackalAgent(
      nh, agent_params, jackal_params, std::move(jackal_navigator)));
}  // namespace agent

bool JackalAgent::GPStoOdom(const double &latitude, const double &longitude,
                            geometry_msgs::PointStamped &odom_point) {
  double utm_x, utm_y;
  geometry_msgs::PointStamped UTM_point;
  std::string utm_zone;

  // convert lat/long to utm
  RobotLocalization::NavsatConversions::LLtoUTM(latitude, longitude, utm_y,
                                                utm_x, utm_zone);

  // Construct UTM_point and odom point geometry messages
  UTM_point.header.frame_id = "utm";
  UTM_point.header.stamp = ros::Time::now();
  UTM_point.point.x = utm_x;
  UTM_point.point.y = utm_y;
  UTM_point.point.z = 0;

  try {
    odom_point.header.stamp = ros::Time::now();
    listener_.transformPoint(jackal_params_.navigation_frame, UTM_point,
                             odom_point);
  } catch (tf::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
  return true;
}

bool JackalAgent::Navigate() {
  move_base_msgs::MoveBaseGoal navigation_goal;
  const std::string navigation_frame = params_.agent_id + "/odom";
  navigation_goal.target_pose.header.frame_id = navigation_frame;
  navigation_goal.target_pose.header.stamp = ros::Time::now();
  navigation_goal.target_pose.pose.orientation.w = 1.0;

  switch (jackal_params_.navigation_mode) {
    case ODOM: {
      navigation_goal.target_pose.pose.position = target_position_.get();

      geometry_msgs::PointStamped point_in, point_out;
      point_in.header.frame_id = jackal_params_.navigation_frame;
      point_in.point = target_position_.get();

      try {
        // geometry_msgs::PointStamped base_point;
        listener_.transformPoint(navigation_frame, point_in, point_out);
        navigation_goal.target_pose.pose.position = point_out.point;

      } catch (tf::TransformException &ex) {
        ROS_ERROR_STREAM(
            "Received an exception trying to transform a point from "
            << jackal_params_.navigation_frame << "to "
            << point_in.header.frame_id << " " << ex.what());
        return false;
      }
      break;
    }
    case GPS: {
      geometry_msgs::PointStamped odom_target_point;
      if (!GPStoOdom(target_position_.get().x, target_position_.get().y,
                     odom_target_point)) {
        ROS_INFO_STREAM(
            "Failed to get local odometry target for jackal GPS navigation!");
        return false;
      }
      navigation_goal.target_pose.pose.position = odom_target_point.point;
      break;
    }
    default: {
      ROS_INFO_STREAM("Unknown jackal navigation mode for jackal!");
      return false;
    }
  }

  jackal_navigator_->sendGoalAndWait(
      navigation_goal, ros::Duration(jackal_params_.execute_timeout_s),
      ros::Duration(jackal_params_.preempt_timeout_s));
  if (jackal_navigator_->getState() ==
      actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO_STREAM("Jackal reached goal!");

    return true;
  } else {
    ROS_INFO_STREAM("Robot "
                    << params_.agent_id
                    << " failed to reach the target location with state "
                    << jackal_navigator_->getState().toString());
    return false;
  }
}

void JackalAgent::UpdatePositionFromOdom(const nav_msgs::Odometry &msg) {
  geometry_msgs::PointStamped point_in, point_out;
  point_in.header = msg.header;
  point_in.point = msg.pose.pose.position;

  try {
    // geometry_msgs::PointStamped base_point;
    listener_.transformPoint(jackal_params_.navigation_frame, point_in,
                             point_out);
    point_out.header.frame_id = jackal_params_.navigation_frame;
    current_position_ = boost::make_optional(point_out.point);

  } catch (tf::TransformException &ex) {
    ROS_ERROR_STREAM("Received an exception trying to transform a point from "
                     << point_in.header.frame_id << "to "
                     << jackal_params_.navigation_frame << " " << ex.what());
  }
}

void JackalAgent::UpdatePositionFromGPS(const sensor_msgs::NavSatFix &msg) {
  current_position_ = boost::none;
  geometry_msgs::PointStamped odom_point;
  if (GPStoOdom(msg.latitude, msg.longitude, odom_point)) {
    current_position_ = boost::make_optional(odom_point.point);
  }
}

}  // namespace agent
}  // namespace sampling
