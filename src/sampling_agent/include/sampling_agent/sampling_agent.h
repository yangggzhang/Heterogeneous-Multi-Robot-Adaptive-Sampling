#pragma once

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sampling_msgs/StopAgent.h>
#include <std_srvs/Trigger.h>

#include <boost/optional.hpp>
#include <string>

namespace sampling {
namespace agent {

/// robot state machine
/// Default : IDLE
/// Workflow
/// Request : Request next interest point from master computer
/// Navigate : navigate to target location
/// Measure: measure data
/// Report : report sample to master computer
enum SamplingState { IDLE, REQUEST, NAVIGATE, MEASURE, REPORT, DIED };

class SamplingAgent {
 public:
  SamplingAgent() = delete;

  static std::unique_ptr<SamplingAgent> MakeUniqueFromROS(ros::NodeHandle &nh,
                                                          ros::NodeHandle &ph);

  bool Run();

 protected:
  SamplingAgent(ros::NodeHandle &nh, const std::string &agent_id);

  bool RequestTarget();

  virtual void ReportLocationCallback(const ros::TimerEvent &);

  virtual bool Navigate();

  virtual bool CollectMeasurement();

  bool ReportSample();

  bool StopAgentService(sampling_msgs::StopAgent::Request &req,
                        sampling_msgs::StopAgent::Response &res);

  bool CheckService(std_srvs::Trigger::Request &req,
                    std_srvs::Trigger::Response &res);

  ros::Timer event_timer_;

  SamplingState agent_state_;

  std::string agent_id_;

  ros::ServiceClient sampling_goal_service_;

  ros::ServiceClient measurement_service_;

  ros::ServiceServer stop_agent_server_;

  ros::ServiceServer check_server_;

  ros::Publisher agent_location_publisher_;

  ros::Publisher sample_publisher_;

  boost::optional<geometry_msgs::Point> current_position_;

  boost::optional<geometry_msgs::Point> target_position_;

  boost::optional<double> measurement_;
};
}  // namespace agent
}  // namespace sampling