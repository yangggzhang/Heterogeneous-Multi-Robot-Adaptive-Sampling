#include "robot_agent/robot_agent.h"

#include <sampling_msgs/MeasurementService.h>
#include <sampling_msgs/ReportSampleService.h>
#include <sampling_msgs/RequestTemperatureMeasurement.h>
#include <sampling_msgs/SamplingGoal.h>
#include <sampling_msgs/measurement.h>

namespace sampling {
namespace agent {

SamplingAgent::SamplingAgent(ros::NodeHandle &nh) {
  nh.param<std::string>("agent_id", agent_id_, "agent0");

  sampling_service_ =
      nh.serviceClient<sampling_msgs::SamplingGoal>("sampling_channel");

  measurement_service_ = nh.serviceClient<sampling_msgs::MeasurementService>(
      "measurement_channel");

  report_service_ = nh.serviceClient<sampling_msgs::ReportSampleService>(
      "report_sample_channel");

  agent_location_publisher_ =
      nh.advertise<sampling_msgs::AgentLocation>("agent_location_channel", 1);

  event_timer_ = nh.createTimer(ros::Duration(1),
                                &SamplingAgent::ReportLocationCallback, this);

  agent_state_ = IDLE;
}

std::unique_ptr<SamplingAgent> SamplingAgent::MakeUniqueFromRos(
    ros::NodeHandle &nh) {
  return std::unique_ptr<SamplingAgent>(new SamplingAgent(nh));
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

void SamplingAgent::Run() {
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
        ROS_INFO_STREAM("Latitude : " << goal_rtk_latitude_
                                      << " Longitude :
                                         "
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
                                           << " reached the target
                                           location!");
          ros::Duration(1.0).sleep();
          agent_state_ = REPORT;
          break;
        }
      } else {
        if (agent_state_ == DIED) {
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
                                   << " failed to reach the target location.
                                   ");
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
                                 << " received new temperature measurement :
                                    "
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
}

}  // namespace agent
}  // namespace sampling

// SamplingAgent::SamplingAgent(const ros::NodeHandle &nh,
//                              const ros::NodeHandle &rh)
//     : nh_(nh), rh_(rh) {
//   if (!rh_.getParam("ros_queue_size", 1)) {
//     ROS_ERROR("Error! Missing default ros queue size!");
//   }

//   if (!rh_.getParam("agent_id", agent_id_)) {
//     ROS_ERROR("Error! Missing robot agent id!");
//   }

//   if (!rh_.getParam("request_target_channel", request_target_channel_)) {
//     ROS_ERROR("Error! Missing robot agent request target channel!");
//   }

//   if (!rh_.getParam("temperature_measurement_channel",
//                     temperature_measurement_channel_)) {
//     ROS_ERROR("Error! Missing robot agent temperature measurement channel!");
//   }

//   if (!rh_.getParam("temperature_update_channel",
//                     temperature_update_channel_)) {
//     ROS_ERROR("Error! Missing robot agent temperature update channel!");
//   }

//   if (!rh_.getParam("gps_location_channel", gps_location_channel_)) {
//     ROS_ERROR("Error! Missing robot agent gps location channel!");
//   }

//   if (!rh_.getParam("report_gps_location_channel",
//                     report_gps_location_channel_)) {
//     ROS_ERROR("Error! Missing robot agent report gps location channel!");
//   }

//   if (!rh_.getParam("initial_loop", initial_loop_)) {
//     ROS_ERROR(
//         "Error! Need to specify whether to loop through four corners at the "
//         "very beginning!");
//     initial_loop_ = false;
//   }
//   loop_count_ = 0;
//   if (initial_loop_) {
//     agent_state_ = LOOP;
//     if (!rh_.getParam("latitude_waypoints", latitude_waypoints_)) {
//       ROS_ERROR("Error! Missing latitude waypoints for initial loop!");
//     }
//     if (!rh_.getParam("longitude_waypoints", longitude_waypoints_)) {
//       ROS_ERROR("Error! Missing longitude waypoints for initial loop!");
//     }
//     if (latitude_waypoints_.size() != longitude_waypoints_.size()) {
//       ROS_ERROR("Initial loop waypoints not match!");
//       initial_loop_ = false;
//       latitude_waypoints_.clear();
//       longitude_waypoints_.clear();
//     }
//   } else {
//     agent_state_ = IDLE;
//   }
//   sampling_service_ = nh_.serviceClient<sampling_msgs::SamplingGoal>(
//       "interest_point_service_channel");

//   temperature_measurement_client_ =
//       nh_.serviceClient<sampling_msgs::RequestTemperatureMeasurement>(
//           temperature_measurement_channel_);

//   temperature_sample_pub_ =
//       nh_.advertise<sampling_msgs::measurement>("sample_collection_channel",
//       1);

//   agent_location_pub_ =
//       nh_.advertise<sampling_msgs::AgentLocation>("agent_location_channel",
//       1);

//   gps_location_sub_ =
//       nh_.subscribe(gps_location_channel_, 1,
//                     &SamplingAgent::update_GPS_location_callback, this);

//   gps_location_server_ = nh_.advertiseService(
//       report_gps_location_channel_, &SamplingAgent::ReportGPSService, this);

//   event_timer_ = nh_.createTimer(ros::Duration(0.2),
//                                  &SamplingAgent::ReportLocationCallback,
//                                  this);
// }

// void SamplingAgent::update_GPS_location_callback(
//     const sensor_msgs::NavSatFix &msg) {
//   current_latitude_ = msg.latitude;
//   current_longitude_ = msg.longitude;
// }

// bool SamplingAgent::ReportGPSService(
//     sampling_msgs::RequestLocation::Request &req,
//     sampling_msgs::RequestLocation::Response &res) {
//   if (agent_id_ != 0) {
//     return false;
//   } else {
//     res.latitude = current_latitude_;
//     res.longitude = current_longitude_;
//     return true;
//   }
// }

// bool SamplingAgent::request_target_from_master() {
//   sampling_msgs::SamplingGoal srv;
//   srv.request.robot_id = agent_id_;
//   srv.request.robot_latitude = current_latitude_;
//   srv.request.robot_longitude = current_longitude_;
//   ROS_INFO_STREAM("Robot " << agent_id_ << " current location : "
//                            << current_latitude_ << " " <<
//                            current_longitude_);

//   if (sampling_service_.call(srv)) {
//     goal_rtk_latitude_ = srv.response.latitude;
//     goal_rtk_longitude_ = srv.response.longitude;
//     return true;
//   } else {
//     ROS_INFO_STREAM("Robot "
//                     << agent_id_
//                     << " failed to request target from master computer!");
//     return false;
//   }
// }

// bool SamplingAgent::collect_temperature_sample() {
//   sampling_msgs::RequestTemperatureMeasurement srv;
//   srv.request.robot_id = agent_id_;

//   if (temperature_measurement_client_.call(srv)) {
//     temperature_measurement_ = srv.response.temperature;
//     return true;
//   } else {
//     ROS_INFO_STREAM("Robot " << agent_id_
//                              << " failed to receive temperature
//                              measurement!");
//     return false;
//   }
// }

// void SamplingAgent::report_temperature_sample() {
//   sampling_msgs::measurement msg;
//   msg.valid = true;
//   msg.robot_id = agent_id_;
//   msg.location_x = current_latitude_;
//   msg.location_y = current_longitude_;
//   msg.measurement = temperature_measurement_;
//   temperature_sample_pub_.publish(msg);
// }

// }  // namespace agent
// }  // namespace agent
// }  // namespace sampling
