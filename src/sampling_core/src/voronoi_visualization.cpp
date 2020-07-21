#include "sampling_core/voronoi_visualization.h"

#include <unordered_set>

namespace sampling {
namespace visualization {

VoronoiVisualization::VoronoiVisualization() {}

VoronoiVisualization::VoronoiVisualization(const Eigen::MatrixXd &map)
    : map_(map) {
  marker_array_ = visualization_msgs::Marker();
  marker_array_.header.frame_id = "sampling_visualization";
  marker_array_.header.stamp = ros::Time::now();
  marker_array_.ns = "voronoi_visualization";
  marker_array_.pose.orientation.w = 0.0;
  marker_array_.action = visualization_msgs::Marker::ADD;
  marker_array_.id = 0;
  marker_array_.type = visualization_msgs::Marker::CUBE_LIST;
  marker_array_.scale.x = 1.0;
  marker_array_.scale.y = 1.0;
  marker_array_.scale.z = 1.0;

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
  map_x_origin_ = (map.col(0).maxCoeff() + map.col(0).minCoeff()) / 2.0;
  map_y_origin_ = (map.col(1).maxCoeff() + map.col(1).minCoeff()) / 2.0;
  map_x_scale_ = (double)x_range / map_x_range;
  map_y_scale_ = (double)y_range / map_y_range;

  // assert(x_range * y_range == map.rows());

  marker_array_.points.resize(map.rows());
  marker_array_.colors.resize(map.rows());

  std_msgs::ColorRGBA color = std_msgs::ColorRGBA();
  color.r = 1.0;
  color.g = 1.0;
  color.b = 1.0;
  color.a = 1.0;

  for (size_t i = 0; i < map.rows(); ++i) {
    geometry_msgs::Point waypoint;
    waypoint.x = (map(i, 0) - map_x_origin_) * map_x_scale_;
    waypoint.y = (map(i, 1) - map_y_origin_) * map_y_scale_;
    waypoint.z = 0.0;
    marker_array_.points[i] = waypoint;
    marker_array_.colors[i] = color;
  }
}

VoronoiVisualization::VoronoiVisualization(const MAP_PARAM &graph_params,
                                           const Eigen::MatrixXd &map)
    : map_(map) {
  marker_array_ = visualization_msgs::Marker();
  marker_array_.header.frame_id = "sampling_visualization";
  marker_array_.header.stamp = ros::Time::now();
  marker_array_.ns = "voronoi_visualization";
  marker_array_.pose.orientation.w = 0.0;
  marker_array_.action = visualization_msgs::Marker::ADD;
  marker_array_.id = 0;
  marker_array_.type = visualization_msgs::Marker::CUBE_LIST;
  marker_array_.scale.x = graph_params.x_scale;
  marker_array_.scale.y = graph_params.y_scale;
  marker_array_.scale.z = 1.0;

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
  map_x_origin_ = (map.col(0).maxCoeff() + map.col(0).minCoeff()) / 2.0;
  map_y_origin_ = (map.col(1).maxCoeff() + map.col(1).minCoeff()) / 2.0;
  map_x_scale_ = (double)x_range / map_x_range;
  map_y_scale_ = (double)y_range / map_y_range;

  // assert(x_range * y_range == map.rows());

  marker_array_.points.resize(map.rows());
  marker_array_.colors.resize(map.rows());

  std_msgs::ColorRGBA color = std_msgs::ColorRGBA();
  color.r = 0.0;
  color.g = 0.0;
  color.b = 0.0;
  color.a = 1.0;

  for (size_t i = 0; i < map.rows(); ++i) {
    geometry_msgs::Point waypoint;
    waypoint.x =
        (map(i, 0) - map_x_origin_) * map_x_scale_ + graph_params.x_offset;
    ;
    waypoint.y =
        (map(i, 1) - map_y_origin_) * map_y_scale_ + graph_params.y_offset;
    waypoint.z = 0.0;
    marker_array_.points[i] = waypoint;
    marker_array_.colors[i] = color;
  }
}

void VoronoiVisualization::UpdateMap(const std::vector<int> &cell_labels) {
  if (map_.rows() != cell_labels.size()) {
    ROS_ERROR_STREAM("Map size does not match filling value for voronoi map");
    ROS_ERROR_STREAM("Map size : " << map_.rows());
    ROS_ERROR_STREAM("Filling value size : " << cell_labels.size());
    return;
  }

  marker_array_.header.stamp = ros::Time::now();

  for (size_t i = 0; i < cell_labels.size(); ++i) {
    switch (cell_labels[i]) {
      case 0: {
        marker_array_.colors[i].r = KRGBRed[0];
        marker_array_.colors[i].g = KRGBRed[1];
        marker_array_.colors[i].b = KRGBRed[2];
        break;
      }
      case 1: {
        marker_array_.colors[i].r = KRGBGreen[0];
        marker_array_.colors[i].g = KRGBGreen[1];
        marker_array_.colors[i].b = KRGBGreen[2];
        break;
      }
      case 2: {
        marker_array_.colors[i].r = KRGBBlue[0];
        marker_array_.colors[i].g = KRGBBlue[1];
        marker_array_.colors[i].b = KRGBBlue[2];
        break;
      }
      case 3: {
        marker_array_.colors[i].r = KRGBYellow[0];
        marker_array_.colors[i].g = KRGBYellow[1];
        marker_array_.colors[i].b = KRGBYellow[2];
        break;
      }
      case 4: {
        marker_array_.colors[i].r = KRGBGray[0];
        marker_array_.colors[i].g = KRGBGray[1];
        marker_array_.colors[i].b = KRGBGray[2];
        break;
      }
      case 5: {
        marker_array_.colors[i].r = KRGBPink[0];
        marker_array_.colors[i].g = KRGBPink[1];
        marker_array_.colors[i].b = KRGBPink[2];
        break;
      }
      default: {
        marker_array_.colors[i].r = 255.0;
        marker_array_.colors[i].g = 255.0;
        marker_array_.colors[i].b = 255.0;
        break;
      }
    }
  }
}

visualization_msgs::Marker VoronoiVisualization::GetVoronoiMap() {
  return marker_array_;
}

visualization_msgs::Marker VoronoiVisualization::GetRobotMarker(
    const Eigen::MatrixXd &robot_locations) {
  visualization_msgs::Marker robot_array = visualization_msgs::Marker();
  robot_array.header.frame_id = "sampling_visualization";
  robot_array.header.stamp = ros::Time::now();
  robot_array.ns = "robot_visualization";
  robot_array.pose.orientation.w = 0.0;
  robot_array.action = visualization_msgs::Marker::ADD;
  robot_array.id = 1;
  robot_array.type = visualization_msgs::Marker::SPHERE_LIST;
  robot_array.scale.x = 5.0;
  robot_array.scale.y = 5.0;
  robot_array.scale.z = 5.0;

  robot_array.points.resize(robot_locations.rows());
  robot_array.colors.resize(robot_locations.rows());

  for (int i = 0; i < robot_locations.rows(); i++) {
    geometry_msgs::Point waypoint;
    waypoint.x = (robot_locations(i, 0) - map_x_origin_) * map_x_scale_;
    waypoint.y = (robot_locations(i, 1) - map_y_origin_) * map_y_scale_;
    waypoint.z = 0.0;
    robot_array.points[i] = waypoint;
    std_msgs::ColorRGBA color = std_msgs::ColorRGBA();
    color.a = 1.0;
    switch (i) {
      case 0: {
        color.r = KRGBRed[0];
        color.g = KRGBRed[1];
        color.b = KRGBRed[2];
        break;
      }
      case 1: {
        color.r = KRGBGreen[0];
        color.g = KRGBGreen[1];
        color.b = KRGBGreen[2];
        break;
      }
      case 2: {
        color.r = KRGBBlue[0];
        color.g = KRGBBlue[1];
        color.b = KRGBBlue[2];
        break;
      }
      case 3: {
        color.r = KRGBYellow[0];
        color.g = KRGBYellow[1];
        color.b = KRGBYellow[2];
        break;
      }
      case 4: {
        color.r = KRGBGray[0];
        color.g = KRGBGray[1];
        color.b = KRGBGray[2];
        break;
      }
      case 5: {
        color.r = KRGBPink[0];
        color.g = KRGBPink[1];
        color.b = KRGBPink[2];
        break;
      }
      default: {
        color.r = 255.0;
        color.g = 255.0;
        color.b = 255.0;
        break;
      }
    }
    robot_array.colors[i] = color;
  }
  return robot_array;
}

}  // namespace visualization
}  // namespace sampling