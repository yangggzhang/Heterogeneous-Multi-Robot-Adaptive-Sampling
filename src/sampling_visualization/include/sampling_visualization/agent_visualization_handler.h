#pragma once

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>

#include "sampling_msgs/AgentLocation.h"
#include "sampling_visualization/sampling_visualization_params.h"

namespace sampling {
namespace visualization {

const double KAgentVisualizationScale = 1.0;
const double KAgentVisualizationHeight = 1.25;
const double KVisualizationAgentUpdateRate_hz = 10;

class AgentVisualizationHandler {
 public:
  AgentVisualizationHandler() = delete;

  static std::unique_ptr<AgentVisualizationHandler> MakeUniqueFromXML(
      ros::NodeHandle &nh, const XmlRpc::XmlRpcValue &yaml_node,
      const int &number_of_agents, const Eigen::MatrixXd &map);

  bool UpdateMarker(
      const std::vector<sampling_msgs::AgentLocation> &agent_locations);

  std::string GetName();

 private:
  AgentVisualizationHandler(ros::NodeHandle &nh,
                            const visualization_msgs::Marker &marker,
                            const double &map_center_x,
                            const double &map_center_y,
                            const SamplingVisualizationParams &params);

  SamplingVisualizationParams params_;

  ros::Timer event_timer_;

  ros::Publisher Agent_publisher_;

  visualization_msgs::Marker marker_;

  double map_center_x_;

  double map_center_y_;

  void UpdateVisualizationCallback(const ros::TimerEvent &);
};
}  // namespace visualization
}  // namespace sampling