#include "sampling_core/voronoi_visualization.h"
#include <unordered_set>

namespace sampling {
namespace visualization {

VoronoiVisualization::VoronoiVisualization() {}

VoronoiVisualization::VoronoiVisualization(const Eigen::MatrixXd &map)
    : map_(map) {
  marker_array_ = visualization_msgs::Marker();
  marker_array_.header.frame_id = "voronoi_visualization";
  marker_array_.header.stamp = ros::Time::now();
  marker_array_.ns = "sampling_visualization";
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
  double map_x_origin = (map.col(0).maxCoeff() + map.col(0).minCoeff()) / 2.0;
  double map_y_origin = (map.col(1).maxCoeff() + map.col(1).minCoeff()) / 2.0;
  double map_x_scale = (double)x_range / map_x_range;
  double map_y_scale = (double)y_range / map_y_range;

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
    waypoint.x = (map(i, 0) - map_x_origin) * map_x_scale;
    waypoint.y = (map(i, 1) - map_y_origin) * map_y_scale;
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
visualization_msgs::Marker VoronoiVisualization::GetMarker() {
  return marker_array_;
}

}  // namespace visualization
}  // namespace sampling