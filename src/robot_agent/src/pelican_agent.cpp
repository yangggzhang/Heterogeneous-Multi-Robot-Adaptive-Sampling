#include "robot_agent/pelican_agent.h"
#include <math.h>

namespace sampling {
namespace agent {
PelicanNode::PelicanNode(const ros::NodeHandle &nh, const ros::NodeHandle &rh)
    : AgentNode(nh, rh) {
  /// todo \paul \yunfei
  /// load necessary parameters

  if (!rh_.getParam("xb_command_channel", xb_command_channel_)) {
    ROS_ERROR("Error! Missing pelican xb command channel!");
  }

  if (!rh_.getParam("height_waiting_threshold", height_waiting_threshold_)) {
    ROS_ERROR("Error! Missing pelican height waiting time threshold!");
  }

  if (!rh_.getParam("navigate_waiting_threshold", navigate_waiting_threshold_)) {
    ROS_ERROR("Error! Missing pelican navigation waiting time threshold!");
  }

  if (!rh_.getParam("maximum_navigation_time", maximum_navigation_time_)) {
    ROS_ERROR("Error! Missing pelican navigation waiting time threshold!");
  }

  if (!rh_.getParam("hover_height", hover_height_)) {
    ROS_ERROR("Error! Missing pelican hover height!");
  }

  if (!rh_.getParam("measure_height", measure_height_)) {
    ROS_ERROR("Error! Missing pelican measurement height!");
  }

  if (!rh_.getParam("gps_converge_threshold", gps_converge_threshold_)) {
    ROS_ERROR("Error! Missing pelican gps converge rate!");
  }

  if (!rh_.getParam("gps_converge_buffer_size", gps_converge_buffer_size_)) {
    ROS_ERROR("Error! Missing pelican gps converge buffer size!");
  }

  if (!rh_.getParam("nagivate_loop_rate", nagivate_loop_rate_int_)) {
    ROS_ERROR("Error! Missing loop rate during navigation!");
  }
  if (!rh_.getParam("latitude_origin", latitude_origin_)) {
    ROS_ERROR("Error! Missing latitude origin (take off point)!");
  }

  if (!rh_.getParam("longitude_origin", longitude_origin_)) {
    ROS_ERROR("Error! Missing longitude origin (take off point)!");
  }

  converge_count_ = 0;
  last_latitude_ = 0.0;
  last_longitude_ = 0.0;
  xb_command_pub_ =
      nh_.advertise<std_msgs::String>(xb_command_channel_, ros_queue_size_);

  initialize_pelican();

  gps_converg_flag_ = true;

  ROS_INFO_STREAM("Launching Pelican waypoint navigation.");
}

/// todo check feedback // pelican connection etc.
bool PelicanNode::initialize_pelican() {
  //Wait for initialization of publisher
  ros::Duration(5).sleep();
  std_msgs::String msg;
  msg.data = "launch_waypoint";
  xb_command_pub_.publish(msg);
  //Wait for initialization of waypint navigation
  ros::Duration(10).sleep();
}

bool PelicanNode::update_goal_from_gps() {
  /// todo \paul \yunfei
  /// transform gps signal from rtk frame to pelican local frame

  /*TODO: Frame Transformation*/
  // cmd_latitude_ = goal_rtk_latitude_;
  // cmd_longitude_ = goal_rtk_longitude_;

  // calculate relative command use origin:
  cmd_latitude_ = (goal_rtk_latitude_ - latitude_origin_)*std::pow(10,7);
  cmd_longitude_ = (goal_rtk_longitude_ - longitude_origin_)*std::pow(10,7);

  return true;
};

bool PelicanNode::waypoint_navigate(const double &latitude,
                                    const double &longitude, const int &height,
                                    const double &converge_duration) {
  /// todo \paul \yunfei height control
  /// improve ros duration
  std_msgs::String msg;
  msg.data = "waypoint_height_auto," + std::to_string(longitude) + "," +
             std::to_string(latitude) + "," + std::to_string(height);
  xb_command_pub_.publish(msg);
  if (converge_duration > 0) {
    ros::Duration(converge_duration).sleep();
  }
  return true;
}

bool PelicanNode::navigate() {
  /// todo \paul \yunfei
  /// GPS waypoint

  /// step 1. increase height
  /// todo \paul \yunfei
  /// solution when it fails
  if (!waypoint_navigate(last_cmd_latitude_, last_cmd_longitude_, hover_height_,
                         height_waiting_threshold_)) {
    ROS_INFO_STREAM("Pelican failed to increase height");
    return false;
  }

  /// step 2. navigate to next gap waypoint
  /// todo \paul \yunfei
  /// solution when it fails
  gps_converg_flag_ = false;
  if (!waypoint_navigate(cmd_latitude_, cmd_longitude_, hover_height_, 
                         navigate_waiting_threshold_)) {
    ROS_INFO_STREAM("Pelican failed to navigate to next location");
    return false;
  }

  /// step 3 wait for gps to converge
  /// todo \paul \yunfei
  /// can not converge forever?
  converge_count_ = 0;
  ros::Rate navigate_loop_rate(nagivate_loop_rate_int_);
  ros::Time begin = ros::Time::now();
  ros::Duration navigationTime = ros::Time::now()- begin;
  while (ros::ok() && // ros is still alive
         !gps_converg_flag_ && // gps not converged
         navigationTime.toSec() <= maximum_navigation_time_) {
    ros::spinOnce();
    navigate_loop_rate.sleep();
    navigationTime = ros::Time::now()- begin;
  }
  if (navigationTime.toSec() > maximum_navigation_time_)
  {
      ROS_INFO_STREAM("Pelican Navigation Time Out");
      return false;
  }

  /// step4 update command
  last_cmd_latitude_ = cmd_latitude_;
  last_cmd_longitude_ = cmd_longitude_;

  /// step 5. navigate to next gap waypoint
  /// todo \paul \yunfei
  /// solution when it fails
  if (!waypoint_navigate(cmd_latitude_, cmd_longitude_, measure_height_,
                         height_waiting_threshold_)) {
    ROS_INFO_STREAM("Pelican failed to land down to measurement height!");
    return false;
  }

  return true;
}

bool PelicanNode::gps_is_converged(const double &last_latitude,
                                   const double &last_longitude,
                                   const double &current_latitude,
                                   const double &current_longitude,
                                   const double &difference_threshold,
                                   const int &buffer_size, int &count) {
  double distance = std::sqrt(pow(last_latitude - current_latitude , 2) +
                              pow(last_longitude - current_longitude, 2));
  if (distance >= difference_threshold) {
    count = 0;
    return false;
  } else {
    count++;
    if (count >= buffer_size) {
      return true;
    }
  }

  return false;
}

void PelicanNode::update_GPS_location_callback(
    const sensor_msgs::NavSatFix &msg) {
  /// todo \paul \yunfei
  /// update the local gps signal back to rtk frame
  /*TODO: Frame Transformation*/
  // currently use the same orginal uav gps, will need an offset and scale after testing
  ROS_INFO_STREAM("Pelican Callback!!!!!!!!");

  last_latitude_ = current_latitude_;
  last_longitude_ = current_longitude_;
  current_latitude_ = msg.latitude;
  current_longitude_ = msg.longitude;

  gps_converg_flag_ = gps_is_converged(
      (last_latitude_*pow(10, 7)), (last_longitude_*pow(10, 7)), 
      (current_latitude_*pow(10, 7)), (current_longitude_*pow(10, 7)),
      gps_converge_threshold_, gps_converge_buffer_size_, converge_count_);
  if (gps_converg_flag_) {
    ROS_INFO_STREAM("Pelican GPS waypoint navigation converged!");
  }
}

}  // namespace agent
}  // namespace sampling