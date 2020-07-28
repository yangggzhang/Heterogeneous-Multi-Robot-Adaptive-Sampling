#include "sampling_agent/sampling_agent.h"

#include <math.h>
#include <sampling_msgs/AgentLocation.h>
#include <sampling_msgs/MeasurementService.h>
#include <sampling_msgs/RequestMeasurement.h>
#include <sampling_msgs/Sample.h>
#include <sampling_msgs/SamplingGoal.h>

#include "sampling_agent/hector_agent.h"
#include "sampling_agent/jackal_agent.h"
#include "sampling_agent/pelican_agent.h"
#include "sampling_msgs/KillAgent.h"

namespace sampling {
namespace agent {

SamplingAgent::SamplingAgent(ros::NodeHandle &nh,
                             const SamplingAgentParams &params)
    : params_(params), last_run_is_done_(false) {
  sampling_goal_service_ =
      nh.serviceClient<sampling_msgs::SamplingGoal>("sampling_goal_channel");

  measurement_service_ = nh.serviceClient<sampling_msgs::RequestMeasurement>(
      "measurement_channel");

  notify_died_agent_service_ =
      nh.serviceClient<sampling_msgs::KillAgent>("kill_agent");

  agent_location_publisher_ =
      nh.advertise<sampling_msgs::AgentLocation>("agent_location_channel", 1);

  sample_publisher_ = nh.advertise<sampling_msgs::Sample>("sample_channel", 1);

  event_timer_ = nh.createTimer(ros::Duration(0.1),
                                &SamplingAgent::ReportLocationCallback, this);

  stop_agent_server_ =
      nh.advertiseService(params_.agent_id + "/stop_agent_channel",
                          &SamplingAgent::StopAgentService, this);

  check_server_ = nh.advertiseService(params_.agent_id + "/check",
                                      &SamplingAgent::CheckService, this);

  agent_state_ = IDLE;

  start_time_ = ros::Time::now();
}

std::unique_ptr<SamplingAgent> SamplingAgent::MakeUniqueFromROS(
    ros::NodeHandle &nh, ros::NodeHandle &ph) {
  std::string agent_type;
  if (!ph.getParam("agent_type", agent_type)) {
    ROS_ERROR("Please specify sampling agent type!");
    return nullptr;
  }
  if (agent_type.compare("JACKAL") == 0) {
    return JackalAgent::MakeUniqueFromROSParam(nh, ph);
  } else if (agent_type.compare("PELICAN") == 0) {
    return PelicanAgent::MakeUniqueFromROSParam(nh, ph);
  } else if (agent_type.compare("HECTOR") == 0) {
    return HectorAgent::MakeUniqueFromROSParam(nh, ph);
  } else
    return nullptr;
}

void SamplingAgent::ReportLocationCallback(const ros::TimerEvent &) {
  if (current_position_.is_initialized()) {
    sampling_msgs::AgentLocation msg;
    msg.header.stamp = ros::Time::now();
    msg.agent_id = params_.agent_id;
    msg.position = current_position_.get();
    agent_location_publisher_.publish(msg);
  }
}

bool SamplingAgent::RequestTarget() {
  if (!current_position_.is_initialized()) {
    ROS_ERROR_STREAM("Agent : " << params_.agent_id << " is lost!");
    return false;
  }
  sampling_msgs::SamplingGoal srv;
  srv.request.agent_location.agent_id = params_.agent_id;
  srv.request.agent_location.position = current_position_.get();

  if (sampling_goal_service_.call(srv)) {
    target_position_ = boost::make_optional(srv.response.target_position);
    return true;
  } else {
    ROS_INFO_STREAM("Robot " << params_.agent_id
                             << " failed to get target from master computer!");
    return false;
  }
  return false;
}

bool SamplingAgent::StopAgentService(sampling_msgs::StopAgent::Request &req,
                                     sampling_msgs::StopAgent::Response &res) {
  if (req.agent_id.compare(params_.agent_id) == 0) {
    res.success = true;
    agent_state_ = DIED;
  } else {
    res.success = false;
  }
  return true;
}

bool SamplingAgent::CheckService(std_srvs::Trigger::Request &req,
                                 std_srvs::Trigger::Response &res) {
  ROS_INFO_STREAM("Agent : " + params_.agent_id << " received check!");
  res.success = true;
  res.message = "Agent : " + params_.agent_id + "is ready!";
  return true;
}

bool SamplingAgent::Navigate() { return false; }

bool SamplingAgent::CollectMeasurement() {
  if (!current_position_.is_initialized()) {
    ROS_INFO_STREAM("Agent " << params_.agent_id << " is lost!");
    return false;
  }
  sampling_msgs::RequestMeasurement srv;
  srv.request.agent_id = params_.agent_id;
  srv.request.position = current_position_.get();

  if (measurement_service_.call(srv)) {
    measurement_ = boost::make_optional(srv.response.data);
    return true;
  } else {
    ROS_INFO_STREAM("Robot " << params_.agent_id
                             << " failed to receive temperature measurement!");
    return false;
  }
  return false;
}

bool SamplingAgent::ReportSample() {
  if (!current_position_.is_initialized()) {
    ROS_INFO_STREAM("Agent " << params_.agent_id << " is lost!");
    return false;
  } else if (!measurement_.is_initialized()) {
    ROS_INFO_STREAM("Agent " << params_.agent_id << " has no measurement!");
    return false;
  }
  sampling_msgs::Sample msg;
  msg.agent_id = params_.agent_id;
  msg.position = current_position_.get();
  msg.position.x =
      static_cast<float>(static_cast<int>(msg.position.x * 10.)) / 10.;
  msg.position.y =
      static_cast<float>(static_cast<int>(msg.position.y * 10.)) / 10.;
  msg.data = measurement_.get();
  measurement_ = boost::none;
  sample_publisher_.publish(msg);
  return true;
}

bool SamplingAgent::Run() {
  switch (agent_state_) {
    case IDLE: {
      if (IsAgentAlive())
        agent_state_ = REQUEST;
      else {
        if (last_run_is_done_) {
          agent_state_ = DIED;
        } else {
          target_position_.get().x = KDiedAgentPositionX_m;
          target_position_.get().y = KDiedAgentPositionY_m;
          agent_state_ = NAVIGATE;
        }
      }
      break;
    }
    case REQUEST: {
      if (IsAgentAlive()) {
        if (!RequestTarget()) {
          ROS_INFO_STREAM("Robot : "
                          << params_.agent_id
                          << " failed to request target from master computer");
          ROS_INFO_STREAM("Retrying ... ... ...");
          break;
        } else {
          ROS_INFO_STREAM("Robot : " << params_.agent_id
                                     << " succeeded in receiving "
                                        "target from master "
                                        "computer : ");
          agent_state_ = NAVIGATE;
        }
      } else {
        if (last_run_is_done_) {
          agent_state_ = DIED;
        } else {
          target_position_.get().x = KDiedAgentPositionX_m;
          target_position_.get().y = KDiedAgentPositionY_m;
          agent_state_ = NAVIGATE;
        }
      }
      break;
    }
    case NAVIGATE: {
      /// Infinite timing allowance rn
      if (IsAgentAlive()) {
        if (Navigate()) {
          ROS_INFO_STREAM("Hooray, agent " << params_.agent_id
                                           << " reached the target location!");
        } else {
          ROS_INFO_STREAM("Agent failed to reach the target location!");
        }
        agent_state_ = MEASURE;
      } else if (target_position_.get().x == KDiedAgentPositionX_m &&
                     target_position_.get().y == KDiedAgentPositionY_m ||
                 !last_run_is_done_) {
        while (!Navigate()) {
          ROS_INFO_STREAM("sgent " << params_.agent_id
                                   << " is trying to reach retiring spot!");
        }
        last_run_is_done_ = true;
        agent_state_ = DIED;
      } else {
        agent_state_ = DIED;
      }
      break;
    }
    case MEASURE: {
      if (IsAgentAlive()) {
        if (!CollectMeasurement()) {
          ROS_INFO_STREAM("Robot : " << params_.agent_id
                                     << " failed to measure temperature!");
          agent_state_ = REQUEST;
        } else {
          ROS_INFO_STREAM("Robot " << params_.agent_id
                                   << " received new temperature measurement : "
                                   << measurement_.get());
          agent_state_ = REPORT;
        }
      } else {
        if (last_run_is_done_) {
          agent_state_ = DIED;
        } else {
          target_position_.get().x = KDiedAgentPositionX_m;
          target_position_.get().y = KDiedAgentPositionY_m;
          agent_state_ = NAVIGATE;
        }
      }
      break;
    }
    case REPORT: {
      if (IsAgentAlive()) {
        if (!ReportSample()) {
          ROS_INFO_STREAM("Sample collected by " << params_.agent_id
                                                 << " failed to report!");
        } else {
          ROS_INFO_STREAM("Sample collected by " << params_.agent_id
                                                 << " is sent to master!");
        }
        agent_state_ = REQUEST;
      } else {
        if (last_run_is_done_) {
          agent_state_ = DIED;
        } else {
          target_position_.get().x = KDiedAgentPositionX_m;
          target_position_.get().y = KDiedAgentPositionY_m;
          agent_state_ = NAVIGATE;
        }
      }
      break;
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

bool SamplingAgent::IsAgentAlive() {
  return (ros::Time::now() - start_time_) >=
         ros::Duration(params_.batterylife_ros_sec);
}

bool SamplingAgent::ReportDiedAgent() {
  sampling_msgs::KillAgent srv;
  srv.request.agent_id = params_.agent_id;
  if (!notify_died_agent_service_.call(srv)) return false;
  return srv.response.success;
}

}  // namespace agent
}  // namespace sampling
