#include "robot_agent/robot_agent.h"
#include <sampling_msgs/RequestGoal.h>
#include <sampling_msgs/RequestTemperatureMeasurement.h>
#include <sampling_msgs/measurement.h>

namespace sampling {
namespace agent {
AgentNode::AgentNode(const ros::NodeHandle &nh, const ros::NodeHandle &rh)
    : nh_(nh), rh_(rh) {
  if (!rh_.getParam("ros_queue_size", ros_queue_size_)) {
    ROS_ERROR("Error! Missing default ros queue size!");
  }

  if (!rh_.getParam("agent_id", agent_id_)) {
    ROS_ERROR("Error! Missing robot agent id!");
  }

  if (!rh_.getParam("request_target_channel", request_target_channel_)) {
    ROS_ERROR("Error! Missing robot agent request target channel!");
  }

  if (!rh_.getParam("temperature_measurement_channel",
                    temperature_measurement_channel_)) {
    ROS_ERROR("Error! Missing robot agent temperature measurement channel!");
  }

  if (!rh_.getParam("temperature_update_channel",
                    temperature_update_channel_)) {
    ROS_ERROR("Error! Missing robot agent temperature update channel!");
  }

  if (!rh_.getParam("gps_location_channel", gps_location_channel_)) {
    ROS_ERROR("Error! Missing robot agent gps location channel!");
  }

  if (!rh_.getParam("report_gps_location_channel",
                    report_gps_location_channel_)) {
    ROS_ERROR("Error! Missing robot agent report gps location channel!");
  }

  if (!rh_.getParam("initial_loop", initial_loop_)) {
    ROS_ERROR(
        "Error! Need to specify whether to loop through four corners at the "
        "very beginning!");
    initial_loop_ = false;
  }
  loop_count_ = 0;
  if (initial_loop_) {
    agent_state_ = LOOP;
    if (!rh_.getParam("latitude_waypoints", latitude_waypoints_)) {
      ROS_ERROR("Error! Missing latitude waypoints for initial loop!");
    }
    if (!rh_.getParam("longitude_waypoints", longitude_waypoints_)) {
      ROS_ERROR("Error! Missing longitude waypoints for initial loop!");
    }
    if (latitude_waypoints_.size() != longitude_waypoints_.size()) {
      ROS_ERROR("Initial loop waypoints not match!");
      initial_loop_ = false;
      latitude_waypoints_.clear();
      longitude_waypoints_.clear();
    }
  } else {
    agent_state_ = IDLE;
  }
  request_target_client_ = nh_.serviceClient<sampling_msgs::RequestGoal>(
      "interest_point_service_channel");

  temperature_measurement_client_ =
      nh_.serviceClient<sampling_msgs::RequestTemperatureMeasurement>(
          temperature_measurement_channel_);

  temperature_sample_pub_ = nh_.advertise<sampling_msgs::measurement>(
      "sample_collection_channel", ros_queue_size_);

  agent_location_pub_ =
      nh_.advertise<sampling_msgs::agent_location>("agent_location_channel", 1);

  gps_location_sub_ =
      nh_.subscribe(gps_location_channel_, ros_queue_size_,
                    &AgentNode::update_GPS_location_callback, this);

  gps_location_server_ = nh_.advertiseService(
      report_gps_location_channel_, &AgentNode::ReportGPSService, this);

  event_timer_ = nh_.createTimer(ros::Duration(0.2),
                                 &AgentNode::ReportLocationCallback, this);
}

void AgentNode::ReportLocationCallback(const ros::TimerEvent &) {
  sampling_msgs::agent_location msg;
  msg.agent_id = agent_id_;
  msg.location_x = current_latitude_;
  msg.location_y = current_longitude_;
  agent_location_pub_.publish(msg);
}

void AgentNode::update_GPS_location_callback(
    const sensor_msgs::NavSatFix &msg) {
  current_latitude_ = msg.latitude;
  current_longitude_ = msg.longitude;
}

bool AgentNode::ReportGPSService(
    sampling_msgs::RequestLocation::Request &req,
    sampling_msgs::RequestLocation::Response &res) {
  if (agent_id_ != 0) {
    return false;
  } else {
    res.latitude = current_latitude_;
    res.longitude = current_longitude_;
    return true;
  }
}

bool AgentNode::request_target_from_master() {
  sampling_msgs::RequestGoal srv;
  srv.request.robot_id = agent_id_;
  srv.request.robot_latitude = current_latitude_;
  srv.request.robot_longitude = current_longitude_;
  ROS_INFO_STREAM("Robot " << agent_id_ << " current location : "
                           << current_latitude_ << " " << current_longitude_);

  if (request_target_client_.call(srv)) {
    goal_rtk_latitude_ = srv.response.latitude;
    goal_rtk_longitude_ = srv.response.longitude;
    return true;
  } else {
    ROS_INFO_STREAM("Robot "
                    << agent_id_
                    << " failed to request target from master computer!");
    return false;
  }
}

bool AgentNode::collect_temperature_sample() {
  sampling_msgs::RequestTemperatureMeasurement srv;
  srv.request.robot_id = agent_id_;

  if (temperature_measurement_client_.call(srv)) {
    temperature_measurement_ = srv.response.temperature;
    return true;
  } else {
    ROS_INFO_STREAM("Robot " << agent_id_
                             << " failed to receive temperature measurement!");
    return false;
  }
}

void AgentNode::report_temperature_sample() {
  sampling_msgs::measurement msg;
  msg.valid = true;
  msg.robot_id = agent_id_;
  msg.location_x = current_latitude_;
  msg.location_y = current_longitude_;
  msg.measurement = temperature_measurement_;
  temperature_sample_pub_.publish(msg);
}

void AgentNode::collect_sample() {
  switch (agent_state_) {
    case LOOP: {
      if (initial_loop_ && loop_count_ < latitude_waypoints_.size()) {
        goal_rtk_latitude_ = latitude_waypoints_[loop_count_];
        goal_rtk_longitude_ = longitude_waypoints_[loop_count_];
        loop_count_++;
        agent_state_ = NAVIGATE;
      } else {
        initial_loop_ = false;
        agent_state_ = IDLE;
      }
      break;
    }
    case IDLE: {
      agent_state_ = REQUEST;
      break;
    }
    case REQUEST: {
      if (!request_target_from_master()) {
        ROS_INFO_STREAM("Robot : "
                        << agent_id_
                        << " failed to request target from master computer");
        ROS_INFO_STREAM("Retrying ... ... ...");
        break;
      } else {
        ROS_INFO_STREAM("Robot : " << agent_id_
                                   << " succeeded in receiving "
                                      "target from master "
                                      "computer : ");
        ROS_INFO_STREAM("Latitude : " << goal_rtk_latitude_ << " Longitude : "
                                      << goal_rtk_longitude_);
        agent_state_ = NAVIGATE;
      }
      break;
    }
    case NAVIGATE: {
      /// Update local goal
      if (!update_goal_from_gps()) {
        ROS_INFO_STREAM(
            "Failed to update local goal from GPS target location !");
        agent_state_ = REQUEST;
        /// todo \yang keeps requesting?
        break;
      } else {
        ROS_INFO_STREAM("Successfully updated local map goal");
      }
      /// Infinite timing allowance rn
      if (navigate()) {
        if (initial_loop_) {
          ROS_INFO_STREAM("Successfully navigated to waypoint : "
                          << goal_rtk_latitude_ << " " << goal_rtk_longitude_);
          agent_state_ = LOOP;
          break;
        } else {
          ROS_INFO_STREAM("Hooray, robot " << agent_id_
                                           << " reached the target location!");
          ros::Duration(1.0).sleep();
          agent_state_ = REPORT;
          break;
        }
      } else {
        if(agent_state_ == DIED) {
          break;
        }
        if (initial_loop_) {
          ROS_INFO_STREAM("Faliled to navigate to waypoint : "
                          << goal_rtk_latitude_ << " " << goal_rtk_longitude_);
          loop_count_--;
          agent_state_ = LOOP;
          break;
          ;
        } else {
          ROS_INFO_STREAM("Robot " << agent_id_
                                   << " failed to reach the target location. ");
          agent_state_ = REPORT;
          break;
        }
      }
    }
    case REPORT: {
      if (!collect_temperature_sample()) {
        ROS_INFO_STREAM("Robot : " << agent_id_
                                   << " failed to measure temperature!");
        ROS_INFO_STREAM("Retrying ... ... ...");
      } else {
        ROS_INFO_STREAM("Robot " << agent_id_
                                 << " received new temperature measurement : "
                                 << temperature_measurement_);
        report_temperature_sample();
        agent_state_ = REQUEST;
      }
    }
    case DIED: {
      break;
    }
    default: {
      agent_state_ = REQUEST;
      break;
    }
  }
}  // namespace agent
}  // namespace agent
}  // namespace sampling
