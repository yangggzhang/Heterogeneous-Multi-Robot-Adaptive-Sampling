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

  agent_state_ = IDLE;
  request_target_client_ =
      nh_.serviceClient<sampling_msgs::RequestGoal>(request_target_channel_);

  temperature_measurement_client_ =
      nh_.serviceClient<sampling_msgs::RequestTemperatureMeasurement>(
          temperature_measurement_channel_);

  temperature_sample_pub_ = nh_.advertise<sampling_msgs::measurement>(
      temperature_update_channel_, ros_queue_size_);

  gps_location_sub_ =
      nh_.subscribe(gps_location_channel_, ros_queue_size_,
                    &AgentNode::update_GPS_location_callback, this);
}

void AgentNode::update_GPS_location_callback(
    const sensor_msgs::NavSatFix &msg) {
  current_latitude_ = msg.latitude;
  current_longitude_ = msg.longitude;
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
  msg.latitude = current_latitude_;
  msg.longitude = current_longitude_;
  msg.measurement = temperature_measurement_;
  temperature_sample_pub_.publish(msg);
  ROS_INFO_STREAM("REPORT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
}

void AgentNode::collect_sample() {
  switch (agent_state_) {
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

        if (!update_goal_from_gps()) {
          ROS_INFO_STREAM(
              "Failed to update local goal from GPS target location !");
          /// todo \yang keeps requesting?
          break;
        } else {
          ROS_INFO_STREAM("Successfully updated local map goal");
        }
        agent_state_ = NAVIGATE;
      }
      break;
    }
    case NAVIGATE: {
      /// Infinite timing allowance rn
      if (navigate()) {
        ROS_INFO_STREAM("Hooray, robot " << agent_id_
                                         << " reached the target location!");
        agent_state_ = REPORT;
        break;
      } else {
        ROS_INFO_STREAM("Robot " << agent_id_
                                 << " failed to reach the target location. ");
        agent_state_ = REQUEST;
        break;
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
    default: {
      agent_state_ = REQUEST;
      break;
    }
  }
}
}  // namespace agent
}  // namespace sampling
