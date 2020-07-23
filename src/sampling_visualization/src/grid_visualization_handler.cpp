#include "sampling_visualization/grid_visualization_handler.h"

namespace sampling {
namespace visualization {

std::unique_ptr<GridVisualizationHandler>
GridVisualizationHandler::MakeUniqueFromXML(
    ros::NodeHandle &nh, const XmlRpc::XmlRpcValue &yaml_node,
    const Eigen::MatrixXd &map) {
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
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.points.resize(map_size);
  marker.colors.resize(map_size);
  std_msgs::ColorRGBA default_color = std_msgs::ColorRGBA();
  default_color.r = 1.0;
  default_color.g = 1.0;
  default_color.b = 1.0;
  default_color.a = 1.0;
  for (int i = 0; i < map_size; ++i) {
    geometry_msgs::Point waypoint;
    waypoint.x =
        (map(i, 0) - map_center_x) * params.scale[0] + params.offset[0];
    waypoint.y =
        (map(i, 1) - map_center_y) * params.scale[1] + params.offset[1];
    waypoint.z = 0.0;
    marker.points[i] = waypoint;
    marker.colors[i] = default_color;
  }
  return std::unique_ptr<GridVisualizationHandler>(
      new GridVisualizationHandler(nh, marker, params));
}

GridVisualizationHandler::GridVisualizationHandler(
    ros::NodeHandle &nh, const visualization_msgs::Marker &marker,
    const SamplingVisualizationParams &params)
    : marker_(marker), params_(params) {
  grid_publisher_ = nh.advertise<visualization_msgs::Marker>(
      KVisualizationNamespace + params.name, 1);

  event_timer_ = nh.createTimer(
      ros::Duration(1.0 / KVisualizationUpdateRate_hz),
      &GridVisualizationHandler::UpdateVisualizationCallback, this);
}

bool GridVisualizationHandler::UpdateMarker(
    const std::vector<double> &marker_value) {
  if (KVisualizationType_Grid.compare(params_.visualization_type) != 0) {
    ROS_ERROR_STREAM("Wrong data type for visualization update!");
    return false;
  } else if (marker_value.size() != marker_.points.size()) {
    ROS_ERROR_STREAM("Visualization data size does not match!");
    return false;
  }
  for (int i = 0; i < marker_.points.size(); ++i) {
    if (!utils_.UpdateColor(marker_value[i], marker_.colors[i])) {
      ROS_ERROR_STREAM("Invalid visualization data point!");
      return false;
    }
  }
  return true;
}

bool GridVisualizationHandler::UpdateMarker(
    const std::vector<int> &marker_value) {
  if (KVisualizationType_Partition.compare(params_.visualization_type) != 0) {
    ROS_ERROR_STREAM("Wrong data type for visualization update!");
    return false;
  } else if (marker_value.size() != marker_.points.size()) {
    ROS_ERROR_STREAM("Visualization data size does not match!");
    return false;
  }
  for (int i = 0; i < marker_.points.size(); ++i) {
    if (!utils_.UpdateColor(marker_value[i], marker_.colors[i])) {
      ROS_ERROR_STREAM("Invalid visualization data point!");
      return false;
    }
  }
  return true;
}

void GridVisualizationHandler::UpdateVisualizationCallback(
    const ros::TimerEvent &) {
  grid_publisher_.publish(marker_);
}
}  // namespace visualization
}  // namespace sampling