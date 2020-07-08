#pragma once

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sampling_msgs/StopAgent.h>

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
enum STATE { IDLE, REQUEST, NAVIGATE, MEASURE, REPORT, DIED };

class SamplingAgent {
 public:
  SamplingAgent() = delete;

  bool Run();

 protected:
  SamplingAgent(ros::NodeHandle &nh);

  bool RequestTarget();

  virtual void ReportLocationCallback(const ros::TimerEvent &);

  virtual bool Navigate() = 0;

  virtual bool CollectMeasurement();

  bool ReportSample();

  bool StopAgentService(sampling_msgs::StopAgent::Request &req,
                        sampling_msgs::StopAgent::Response &res);

  ros::Timer event_timer_;

  STATE agent_state_;

  std::string agent_id_;

  ros::ServiceClient sampling_service_;

  ros::ServiceClient measurement_service_;

  ros::ServiceServer stop_agent_server_;

  ros::Publisher agent_location_publisher_;

  ros::Publisher sample_publisher_;

  boost::optional<geometry_msgs::Point> current_position_;

  boost::optional<geometry_msgs::Point> target_position_;

  boost::optional<double> measurement_;
};
}  // namespace agent
}  // namespace sampling