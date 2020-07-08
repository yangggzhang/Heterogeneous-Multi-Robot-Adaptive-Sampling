#pragma once

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sampling_msgs/AgentLocation.h>
#include <sampling_msgs/RequestLocation.h>
#include <sampling_msgs/RequestTemperatureMeasurement.h>
#include <sampling_msgs/SamplingGoal.h>
#include <sensor_msgs/NavSatFix.h>

#include <boost/optional.hpp>
#include <string>

namespace sampling {
namespace agent {

/// robot state machine
/// Default : IDLE
/// Workflow
/// Request : Request next interest point from master computer
/// Navigate : navigate to target location
/// Report : measure temperature and report to master computer
enum STATE { IDLE, REQUEST, NAVIGATE, REPORT, DIED };

class SamplingAgent {
 public:
  SamplingAgent() = delete;

  static std::unique_ptr<SamplingAgent> MakeUniqueFromRos(ros::NodeHandle &nh);

  bool Run();

  // SamplingAgent(const ros::NodeHandle &nh, const ros::NodeHandle &rh);

  // virtual bool update_goal_from_gps() = 0;

  // virtual bool navigate() = 0;

  // virtual void update_GPS_location_callback(const sensor_msgs::NavSatFix
  // &msg);

  // bool request_target_from_master();

  // virtual bool collect_temperature_sample();

  // void report_temperature_sample();

  // bool collect_temperature_sample(
  //     sampling_msgs::RequestTemperatureMeasurement::Request &req,
  //     sampling_msgs::RequestTemperatureMeasurement::Response &res);

  // virtual bool ReportGPSService(sampling_msgs::RequestLocation::Request &req,
  //                               sampling_msgs::RequestLocation::Response
  //                               &res);

  /// State machine
  // void collect_sample();

 protected:
  SamplingAgent(ros::NodeHandle &nh);

  void ReportLocationCallback(const ros::TimerEvent &);

  ros::Timer event_timer_;

  STATE agent_state_;

  std::string agent_id_;

  ros::ServiceClient sampling_service_;

  ros::ServiceClient measurement_service_;

  ros::ServiceClient report_service_;

  ros::Publisher agent_location_publisher_;

  boost::optional<geometry_msgs::Point> current_position_;
};
}  // namespace agent
}  // namespace sampling