#include "sampling_visualization/agent_visualization_handler.h"

#include "sampling_visualization/sampling_visualization_utils.h"

namespace sampling {
namespace visualization {

std::unique_ptr<AgentVisualizationHandler>
AgentVisualizationHandler::MakeUniqueFromXML(
    ros::NodeHandle &nh, const XmlRpc::XmlRpcValue &yaml_node,
    const int &number_of_agents, const Eigen::MatrixXd &map) {
  SamplingVisualizationParams params;
  if (!params.LoadFromXML(yaml_node)) {
    ROS_ERROR_STREAM("Failed to load parameters for visualization !");
    return nullptr;
  }
  const int map_size = map.rows();
  double map_center_x = map.col(0).mean();
  double map_center_y = map.col(1).mean();

  visualization_msgs::Marker marker;
  marker.header.frame_id = params.name;
  marker.header.stamp = ros::Time::now();
  marker.ns = KVisualizationNamespace;
  marker.pose.orientation.w = 0.0;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = KAgentVisualizationScale;
  marker.scale.y = KAgentVisualizationScale;
  marker.scale.z = 1.0;

  SamplingVisualizationUtils utils;

  marker.points.resize(number_of_agents);
  marker.colors.resize(number_of_agents);
  for (int i = 0; i < number_of_agents; ++i) {
    geometry_msgs::Point waypoint;
    waypoint.x = map_center_x + params.offset[0];
    waypoint.y = map_center_y + params.offset[1];
    waypoint.z = KAgentVisualizationHeight;
    std_msgs::ColorRGBA color;
    if (!utils.UpdateColor(i, color)) {
      ROS_ERROR_STREAM("Failed to assign color for agent visualization!");
      return nullptr;
    }
    marker.points[i] = waypoint;
    marker.colors[i] = color;
  }
  return std::unique_ptr<AgentVisualizationHandler>(
      new AgentVisualizationHandler(nh, marker, map_center_x, map_center_y,
                                    params));
}

AgentVisualizationHandler::AgentVisualizationHandler(
    ros::NodeHandle &nh, const visualization_msgs::Marker &marker,
    const double &map_center_x, const double &map_center_y,
    const SamplingVisualizationParams &params)
    : marker_(marker),
      map_center_x_(map_center_x),
      map_center_y_(map_center_y),
      params_(params) {
  Agent_publisher_ = nh.advertise<visualization_msgs::Marker>(
      KVisualizationNamespace + params.name, 1);

  event_timer_ = nh.createTimer(
      ros::Duration(1.0 / KVisualizationAgentUpdateRate_hz),
      &AgentVisualizationHandler::UpdateVisualizationCallback, this);
}

bool AgentVisualizationHandler::UpdateMarker(
    const std::vector<geometry_msgs::Point> &agent_locations) {
  if (agent_locations.size() != marker_.points.size()) {
    ROS_ERROR_STREAM("Number of agents for visualization does NOT match!");
    return false;
  }
  for (int i = 0; i < agent_locations.size(); ++i) {
    marker_.points[i].x =
        (agent_locations[i].x - map_center_x_) * params_.scale[0] +
        params_.offset[0];
    marker_.points[i].y =
        (agent_locations[i].y - map_center_y_) * params_.scale[1] +
        params_.offset[1];
  }
  return true;
}

void AgentVisualizationHandler::UpdateVisualizationCallback(
    const ros::TimerEvent &) {
  Agent_publisher_.publish(marker_);
}
}  // namespace visualization
}  // namespace sampling