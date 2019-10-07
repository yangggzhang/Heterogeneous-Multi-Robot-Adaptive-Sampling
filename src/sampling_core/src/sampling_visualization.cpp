#include "sampling_core/sampling_visualization.h"

namespace sampling {
namespace visualization {
sampling_visualization::sampling_visualization() {
  latitude_range_ = 0;
  longitude_range_ = 0;
  x_scale_ = 1.0;
  y_scale_ = 1.0;
  z_scale_ = 1.0;
}

sampling_visualization::sampling_visualization(const Eigen::MatrixXd &location,
                                               const double &map_resolution,
                                               const double &x_scale,
                                               const double &y_scale,
                                               const double &z_scale)
    : location_(location),
      x_scale_(x_scale),
      y_scale_(y_scale),
      z_scale_(z_scale) {
  assert(location.cols() == 2);
  latitude_range_ =
      std::round((location.col(0).maxCoeff() - location.col(0).minCoeff()) /
                 map_resolution) +
      1;
  longitude_range_ =
      std::round((location.col(1).maxCoeff() - location.col(1).minCoeff()) /
                 map_resolution) +
      1;
}

void sampling_visualization::get_heatmap_color(const double &norm,
                                               std_msgs::ColorRGBA &color) {
  int idx1, idx2;
  double fracB = 0;

  if (norm <= 0) {
    idx1 = idx2 = 0;
  } else if (norm >= 1) {
    idx1 = idx2 = K_NUM_COLOR - 1;
  } else {
    double adjusted_norm = norm * ((double)K_NUM_COLOR - 1);
    idx1 = floor(adjusted_norm);
    idx2 = idx1 + 1;
    fracB = adjusted_norm - (double)idx1;
  }
  color.a = 1.0;
  color.r = (K_COLOR[idx2][0] - K_COLOR[idx1][0]) * fracB + K_COLOR[idx1][0];
  color.g = (K_COLOR[idx2][1] - K_COLOR[idx1][1]) * fracB + K_COLOR[idx1][1];
  color.b = (K_COLOR[idx2][2] - K_COLOR[idx1][2]) * fracB + K_COLOR[idx1][2];
}

void sampling_visualization::initialize_map(
    const std::string &visualization_frame, const std::string &name_space,
    const int &map_id, visualization_msgs::Marker &map) {
  visualization_msgs::Marker empty_marker;
  map = empty_marker;
  map.header.frame_id = visualization_frame;
  map.header.stamp = ros::Time::now();
  map.ns = name_space;
  map.pose.orientation.w = 0.0;
  map.action = visualization_msgs::Marker::ADD;
  map.id = map_id;
  map.type = visualization_msgs::Marker::CUBE_LIST;
  map.scale.x = x_scale_;
  map.scale.y = y_scale_;
  map.scale.z = z_scale_;
}

void sampling_visualization::update_map(const int &offset,
                                        const Eigen::VectorXd &filling_value,
                                        visualization_msgs::Marker &map) {
  assert(filling_value.size() == latitude_range_ * longitude_range_);
  map.header.stamp = ros::Time::now();
  double upper_bound = filling_value.maxCoeff();
  double lower_bound = filling_value.minCoeff();
  bool equal = upper_bound == lower_bound;
  map.points.resize(filling_value.size());
  map.colors.resize(filling_value.size());

  for (int lat = 0; lat < latitude_range_; lat++) {
    for (int lng = 0; lng < longitude_range_; lng++) {
      int index = lat * longitude_range_ + lng;
      geometry_msgs::Point p;
      p.x = (lat - latitude_range_ / 2) * map.scale.x;
      p.y = (lng - longitude_range_ / 2) * map.scale.y + offset;
      p.z = -1.0;
      map.points[index] = p;

      std_msgs::ColorRGBA color;
      double norm;
      if (std::isnan(filling_value(index)) || equal) {
        norm = 0;
      } else {
        norm =
            (filling_value(index) - lower_bound) / (upper_bound - lower_bound);
      }
      get_heatmap_color(norm, color);
      map.colors[index] = color;
    }
  }
}
}  // namespace visualization
}  // namespace sampling