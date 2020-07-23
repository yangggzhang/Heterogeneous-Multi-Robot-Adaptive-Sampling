#include "sampling_agent/sampling_agent.h"

#include <sampling_msgs/AgentLocation.h>
#include <sampling_msgs/MeasurementService.h>
#include <sampling_msgs/RequestMeasurement.h>
#include <sampling_msgs/Sample.h>
#include <sampling_msgs/SamplingGoal.h>

#include "sampling_agent/hector_agent.h"
#include "sampling_agent/jackal_agent.h"
#include "sampling_agent/pelican_agent.h"

namespace sampling {
namespace agent {

SamplingAgent::SamplingAgent(ros::NodeHandle &nh, const std::string &agent_id)
    : agent_id_(agent_id) {
  ros_ns_ = nh.getNamespace();
  sampling_service_ =
      nh.serviceClient<sampling_msgs::SamplingGoal>("sampling_target_channel");

  measurement_service_ = nh.serviceClient<sampling_msgs::MeasurementService>(
      "measurement_channel");

  agent_location_publisher_ =
      nh.advertise<sampling_msgs::AgentLocation>("agent_location_channel", 1);

  sample_publisher_ = nh.advertise<sampling_msgs::Sample>("sample_channel", 1);

  event_timer_ = nh.createTimer(ros::Duration(1),
                                &SamplingAgent::ReportLocationCallback, this);

  stop_agent_server_ = nh.advertiseService(
      "stop_agent_channel", &SamplingAgent::StopAgentService, this);

  check_server_ =
      nh.advertiseService("check", &SamplingAgent::CheckService, this);

  agent_state_ = IDLE;
}

std::unique_ptr<SamplingAgent> SamplingAgent::MakeUniqueFromROS(
    ros::NodeHandle &nh, const std::string &agent_type) {
  if (agent_type.compare("JACKAL") == 0) {
    return JackalAgent::MakeUniqueFromROS(nh);
  } else if (agent_type.compare("PELICAN") == 0) {
    return PelicanAgent::MakeUniqueFromROSParam(nh);
  } else if (agent_type.compare("HECTOR") == 0) {
    return HectorAgent::MakeUniqueFromROSParam(nh);
  } else
    return nullptr;
}

void SamplingAgent::ReportLocationCallback(const ros::TimerEvent &) {
  if (current_position_.is_initialized()) {
    sampling_msgs::AgentLocation msg;
    msg.header.stamp = ros::Time::now();
    msg.agent_id = agent_id_;
    msg.position = current_position_.get();
    agent_location_publisher_.publish(msg);
  }
}

bool SamplingAgent::RequestTarget() {
  if (!current_position_.is_initialized()) {
    ROS_ERROR_STREAM("Agent : " << agent_id_ << " is lost!");
    return false;
  }
  sampling_msgs::SamplingGoal srv;
  srv.request.agent_id = agent_id_;
  srv.request.agent_position = current_position_.get();

  if (sampling_service_.call(srv)) {
    target_position_ = boost::make_optional(srv.response.target_position);
    return true;
  } else {
    ROS_INFO_STREAM("Robot " << agent_id_
                             << " failed to get target from master computer!");
    return false;
  }
  return false;
}

bool SamplingAgent::StopAgentService(sampling_msgs::StopAgent::Request &req,
                                     sampling_msgs::StopAgent::Response &res) {
  if (req.agent_id.compare(agent_id_) == 0) {
    res.success = true;
    agent_state_ = DIED;
  } else {
    res.success = false;
  }
  return true;
}

bool SamplingAgent::CheckService(std_srvs::Trigger::Request &req,
                                 std_srvs::Trigger::Response &res) {
  res.success = true;
  res.message = "Agent : " + agent_id_ + "is ready!";
  return true;
}

bool SamplingAgent::Navigate() { return false; }

bool SamplingAgent::CollectMeasurement() {
  if (!current_position_.is_initialized()) {
    ROS_INFO_STREAM("Agent " << agent_id_ << " is lost!");
    return false;
  }
  sampling_msgs::RequestMeasurement srv;
  srv.request.agent_id = agent_id_;
  srv.request.position = current_position_.get();

  if (measurement_service_.call(srv)) {
    measurement_ = boost::make_optional(srv.response.data);
    return true;
  } else {
    ROS_INFO_STREAM("Robot " << agent_id_
                             << " failed to receive temperature measurement!");
    return false;
  }
  return false;
}

bool SamplingAgent::ReportSample() {
  if (!current_position_.is_initialized()) {
    ROS_INFO_STREAM("Agent " << agent_id_ << " is lost!");
    return false;
  } else if (!measurement_.is_initialized()) {
    ROS_INFO_STREAM("Agent " << agent_id_ << " has no measurement!");
    return false;
  }
  sampling_msgs::Sample msg;
  msg.valid = true;
  msg.agent_id = agent_id_;
  msg.position = current_position_.get();
  msg.data = measurement_.get();
  measurement_ = boost::none;
  sample_publisher_.publish(msg);
  return true;
}

bool SamplingAgent::Run() {
  switch (agent_state_) {
    case IDLE: {
      agent_state_ = REQUEST;
      break;
    }
    case REQUEST: {
      if (!RequestTarget()) {
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
        ROS_INFO_STREAM("Position : " << target_position_.get().x << " "
                                      << target_position_.get().y);
        agent_state_ = NAVIGATE;
      }
      break;
    }
    case NAVIGATE: {
      /// Infinite timing allowance rn
      if (Navigate()) {
        ROS_INFO_STREAM("Hooray, agent " << agent_id_
                                         << " reached the target location!");
        ros::Duration(1.0).sleep();
      } else {
        ROS_INFO_STREAM("Agent failed to reach the target location!");
      }
      agent_state_ = MEASURE;
      break;
    }
    case MEASURE: {
      if (!CollectMeasurement()) {
        ROS_INFO_STREAM("Robot : " << agent_id_
                                   << " failed to measure temperature!");
        agent_state_ = REQUEST;
      } else {
        ROS_INFO_STREAM("Robot " << agent_id_
                                 << " received new temperature measurement : "
                                 << measurement_.get());
        agent_state_ = REPORT;
      }
    }
    case REPORT: {
      if (!ReportSample()) {
        ROS_INFO_STREAM("Sample collected by " << agent_id_
                                               << " failed to report!");
      } else {
        ROS_INFO_STREAM("Sample collected by " << agent_id_
                                               << " is sent to master!");
      }
      agent_state_ = REQUEST;
    }
    case DIED: {
      ros::Duration(1.0).sleep();
      break;
    }
    default: {
      ROS_INFO_STREAM("Unknown agent state!");
      return false;
    }
  }
  return true;
}

}  // namespace agent
}  // namespace sampling
