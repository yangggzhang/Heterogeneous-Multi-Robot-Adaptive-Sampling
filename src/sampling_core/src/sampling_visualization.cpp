#include "sampling_core/sampling_visualization.h"

namespace sampling {
namespace visualization {
SamplingVisualization::SamplingVisualization() {}

SamplingVisualization::SamplingVisualization(ros::NodeHandle &nh,
                                             const MAP_PARAM &param,
                                             const Eigen::MatrixXd &map)
    : param_(param), map_(map) {
  map_visualization_pub_ = nh.advertise<visualization_msgs::Marker>(
      "sampling_visualization/" + param.map_frame, 1);

  marker_array_ = visualization_msgs::Marker();
  marker_array_.header.frame_id = "sampling_visualization";
  marker_array_.header.stamp = ros::Time::now();
  marker_array_.ns = "sampling_visualization";
  marker_array_.pose.orientation.w = 0.0;
  marker_array_.action = visualization_msgs::Marker::ADD;
  marker_array_.id = param.map_id;
  marker_array_.type = visualization_msgs::Marker::CUBE_LIST;
  marker_array_.scale.x = param.x_scale;
  marker_array_.scale.y = param.y_scale;

  std::unordered_set<double> x_array;
  std::unordered_set<double> y_array;
  for (size_t i = 0; i < map.rows(); ++i) {
    x_array.insert(map(i, 0));
    y_array.insert(map(i, 1));
  }
  size_t x_range = x_array.size();
  size_t y_range = y_array.size();
  double map_x_range = map.col(0).maxCoeff() - map.col(0).minCoeff();
  double map_y_range = map.col(1).maxCoeff() - map.col(1).minCoeff();
  double map_x_origin = (map.col(0).maxCoeff() + map.col(0).minCoeff()) / 2.0;
  double map_y_origin = (map.col(1).maxCoeff() + map.col(1).minCoeff()) / 2.0;
  double map_x_scale = (double)x_range / map_x_range;
  double map_y_scale = (double)y_range / map_y_range;

  assert(x_range * y_range == map.rows());

  marker_array_.points.resize(map.rows());
  marker_array_.colors.resize(map.rows());

  std_msgs::ColorRGBA color = std_msgs::ColorRGBA();
  color.r = 1.0;
  color.g = 0.0;
  color.b = 0.0;
  color.a = 1.0;

  for (size_t i = 0; i < map.rows(); ++i) {
    geometry_msgs::Point waypoint;
    waypoint.x = (map(i, 0) - map_x_origin) * map_x_scale;
    waypoint.y = (map(i, 1) - map_y_origin) * map_y_scale;
    waypoint.z = 0.0;
    waypoint.x = waypoint.x * param.x_scale + param.x_offset;
    waypoint.y = waypoint.y * param.y_scale + param.y_offset;
    marker_array_.points[i] = waypoint;
    marker_array_.colors[i] = color;
  }
  timer_ = nh.createWallTimer(ros::WallDuration(0.1),
                              &SamplingVisualization::MapVisualizationCallback,
                              this);
}

void SamplingVisualization::update_map(const Eigen::VectorXd &filling_value) {
  if (map_.rows() != filling_value.rows()) {
    ROS_ERROR_STREAM(
        "Map size does not match filling value for map : " << param_.map_frame);
    ROS_ERROR_STREAM("Map size : " << map_.rows());
    ROS_ERROR_STREAM("Filling value size : " << filling_value.rows());
    return;
  }

  marker_array_.header.stamp = ros::Time::now();

  double lower_bound = std::min(param_.lower_bound, filling_value.minCoeff());
  double upper_bound = std::max(param_.upper_bound, filling_value.maxCoeff());

  for (size_t i = 0; i < filling_value.rows(); ++i) {
    double norm;
    if (std::isnan(filling_value(i)) || lower_bound == upper_bound) {
      norm = 0;
    } else {
      norm = (filling_value(i) - lower_bound) / (upper_bound - lower_bound);
    }
    marker_array_.colors[i] = get_heatmap_color(norm);
  }
}

void SamplingVisualization::MapVisualizationCallback(
    const ros::WallTimerEvent &event) {
  if (!marker_array_.points.empty() && !marker_array_.colors.empty() &&
      marker_array_.points.size() == marker_array_.colors.size()) {
    map_visualization_pub_.publish(marker_array_);
  }
}

std_msgs::ColorRGBA SamplingVisualization::get_heatmap_color(
    const double &norm) {
  std_msgs::ColorRGBA color;
  int idx1, idx2;
  double fracB = 0;

  double adjusted_norm = norm * ((double)K_NUM_COLOR - 1);
  idx1 = floor(adjusted_norm);
  idx2 = idx1 + 1;
  fracB = adjusted_norm - (double)idx1;

  color.a = 1.0;
  color.r = (K_COLOR[idx2][0] - K_COLOR[idx1][0]) * fracB + K_COLOR[idx1][0];
  color.g = (K_COLOR[idx2][1] - K_COLOR[idx1][1]) * fracB + K_COLOR[idx1][1];
  color.b = (K_COLOR[idx2][2] - K_COLOR[idx1][2]) * fracB + K_COLOR[idx1][2];
  return color;
}

}  // namespace visualization
}  // namespace sampling