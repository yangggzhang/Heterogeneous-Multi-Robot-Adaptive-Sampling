#include "sampling_agent/pelican_agent.h"

namespace sampling {
namespace agent {
PelicanAgent::PelicanAgent(ros::NodeHandle &nh, const std::string &agent_id,
                           const PelicanAgentParam &params)
    : SamplingAgent(nh, agent_id), params_(params) {
  /// todo \paul \yunfei
  /// load necessary parameters

  converge_count_ = 0;
  gps_converge_flag_ = true;

  xb_command_publisher_ = nh.advertise<std_msgs::String>("/write", 1);

  gps_subscriber_ =
      nh.subscribe("/fcu/gps", 1, &PelicanAgent::UpdatePositionFromGPS, this);

  ros::Duration(5).sleep();
  std_msgs::String msg;
  msg.data = "launch_waypoint";
  xb_command_publisher_.publish(msg);

  // Wait for initialization of waypint navigation
  ros::Duration(10).sleep();
  ROS_INFO_STREAM("Launching Pelican waypoint navigation.");
}

std::unique_ptr<PelicanAgent> PelicanAgent::MakeUniqueFromROSParam(
    ros::NodeHandle &nh) {
  std::string agent_id;
  nh.param<std::string>("agent_id", agent_id, "pelican0");
  ros::NodeHandle ph("~");
  PelicanAgentParam params;
  if (!params.LoadFromRosParams(ph)) {
    ROS_ERROR("Failed to load pelican parameters!");
    return nullptr;
  }
  return std::unique_ptr<PelicanAgent>(new PelicanAgent(nh, agent_id, params));
}

bool PelicanAgent::WaypointNavigate(const double &latitude,
                                    const double &longitude,
                                    const double &height,
                                    const double &converge_duration) {
  /// todo \paul \yunfei height control
  /// improve ros duration
  std_msgs::String msg;
  msg.data = "waypoint_height_auto," + std::to_string(longitude) + "," +
             std::to_string(latitude) + "," + std::to_string(height);
  xb_command_publisher_.publish(msg);
  if (converge_duration > 0) {
    ros::Duration(converge_duration).sleep();
  }
  return true;
}

bool PelicanAgent::Navigate() {
  /// todo \paul \yunfei
  /// GPS waypoint

  if (!target_position_.is_initialized()) {
    ROS_INFO_STREAM("Pelican failed to get sampling target");
    return false;
  }

  /// step 1. increase height
  /// todo \paul \yunfei
  /// solution when it fails
  if (!current_gps_position_.is_initialized()) {
    ROS_INFO_STREAM("Pelican lost GPS connection");
    return false;
  }

  if (!WaypointNavigate(current_gps_position_.get().x * KGPSScaleFactor,
                        current_gps_position_.get().y * KGPSScaleFactor,
                        params_.hover_height_mm, params_.measure_height_mm)) {
    ROS_INFO_STREAM("Pelican failed to increase height");
    return false;
  }

  /// step 2. navigate to next gap waypoint
  /// todo \paul \yunfei
  /// solution when it fails

  double transformed_gps_rtk_latitude_ =
      target_position_.get().x + params_.latitude_offset;
  double transformed_gps_rtk_lonitude_ =
      target_position_.get().y + params_.longitude_offset;

  double relative_rtk_latitude =
      transformed_gps_rtk_latitude_ - params_.rtk_latitude_offset;
  double relative_rtk_longitude =
      transformed_gps_rtk_lonitude_ - params_.rtk_longitude_offset;

  double cmd_latitude = relative_rtk_latitude * KGPSScaleFactor;
  double cmd_longitude = relative_rtk_longitude * KGPSScaleFactor;

  gps_converge_flag_ = false;
  if (!WaypointNavigate(cmd_latitude, cmd_longitude, params_.hover_height_mm,
                        params_.navigate_wait_time_s)) {
    ROS_INFO_STREAM("Pelican failed to navigate to next location");
    return false;
  }

  /// step 3 wait for gps to converge
  /// todo \paul \yunfei
  /// can not converge forever?
  converge_count_ = 0;
  ros::Rate navigate_loop_rate(params_.navigate_loop_rate_hz);
  ros::Time begin = ros::Time::now();
  ros::Duration navigationTime = ros::Time::now() - begin;
  while (ros::ok() &&            // ros is still alive
         !gps_converge_flag_ &&  // gps not converged
         navigationTime.toSec() <= params_.navigate_timeout_s) {
    ros::spinOnce();
    navigate_loop_rate.sleep();
    navigationTime = ros::Time::now() - begin;
  }
  if (navigationTime.toSec() > params_.navigate_timeout_s) {
    ROS_INFO_STREAM("Pelican Navigation Time Out");
    return false;
  }

  /// step 4. navigate to next gap waypoint
  /// todo \paul \yunfei
  /// solution when it fails
  if (!WaypointNavigate(cmd_latitude, cmd_longitude, params_.measure_height_mm,
                        params_.hover_time_s)) {
    ROS_INFO_STREAM("Pelican failed to land down to measurement height!");
    return false;
  }
  return true;
}

void PelicanAgent::CheckConvergence() {
  if (!last_gps_position_.is_initialized() ||
      !current_gps_position_.is_initialized()) {
    converge_count_ = 0;
    gps_converge_flag_ = false;
  }
  double distance = std::sqrt(
      pow(last_gps_position_.get().x - current_gps_position_.get().x, 2) +
      pow(last_gps_position_.get().y - current_gps_position_.get().y, 2));
  distance *= KGPSScaleFactor;
  if (distance >= params_.gps_converge_threshold_mm) {
    converge_count_ = 0;
    gps_converge_flag_ = false;
  } else {
    gps_converge_flag_ = ++converge_count_ >= params_.gps_buffer_size;
    if (gps_converge_flag_)
      ROS_INFO_STREAM("Pelican GPS waypoint navigation converged!");
  }
}

void PelicanAgent::UpdatePositionFromGPS(const sensor_msgs::NavSatFix &msg) {
  /// todo \paul \yunfei
  /// update the local gps signal back to rtk frame
  /*TODO: Frame Transformation*/
  // currently use the same orginal uav gps, will need an offset and scale after
  // testing
  ROS_INFO_STREAM("Pelican Callback!!!!!!!!");

  if (current_gps_position_.is_initialized()) {
    last_gps_position_ = current_gps_position_;
    current_gps_position_.get().x = msg.latitude;
    current_gps_position_.get().y = msg.longitude;
    current_gps_position_.get().z = msg.altitude;
  } else {
    geometry_msgs::Point current_gps_position;
    current_gps_position.x = msg.latitude;
    current_gps_position.y = msg.longitude;
    current_gps_position.z = msg.altitude;
    current_gps_position_ = boost::make_optional(current_gps_position);
  }

  CheckConvergence();

  double latitude_in_rtk_frame =
      current_gps_position_.get().x +
      (params_.rtk_latitude_offset - params_.navigation_latitude_offset);
  double longitude_in_rtk_frame =
      current_gps_position_.get().y +
      (params_.rtk_longitude_offset - params_.navigation_longitude_offset);

  if (!current_position_.is_initialized()) {
    geometry_msgs::Point current_position;
    current_position.x = (latitude_in_rtk_frame - params_.latitude_offset);
    current_position.y = (longitude_in_rtk_frame - params_.longitude_offset);
    current_position.z = current_gps_position_.get().z;
    current_position_ = boost::make_optional(current_position);
  } else {
    current_position_.get().x =
        (latitude_in_rtk_frame - params_.latitude_offset);
    current_position_.get().y =
        (longitude_in_rtk_frame - params_.longitude_offset);
    current_position_.get().z = current_gps_position_.get().z;
  }
}

}  // namespace agent
}  // namespace sampling
