#include "sampling_core/sampling_visualization.h"

#include "sampling_core/voronoi_visualization.h"
#include "sampling_msgs/RequestLocation.h"

namespace sampling {
namespace visualization {
SamplingVisualization::SamplingVisualization() {}

void SamplingVisualization::HSVtoRGB(const double &fH, const double &fS,
                                     const double &fV, double &fR, double &fG,
                                     double &fB) {
  float fC = fV * fS;  // Chroma
  float fHPrime = fmod(fH / 60.0, 6);
  float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
  float fM = fV - fC;

  if (0 <= fHPrime && fHPrime < 1) {
    fR = fC;
    fG = fX;
    fB = 0;
  } else if (1 <= fHPrime && fHPrime < 2) {
    fR = fX;
    fG = fC;
    fB = 0;
  } else if (2 <= fHPrime && fHPrime < 3) {
    fR = 0;
    fG = fC;
    fB = fX;
  } else if (3 <= fHPrime && fHPrime < 4) {
    fR = 0;
    fG = fX;
    fB = fC;
  } else if (4 <= fHPrime && fHPrime < 5) {
    fR = fX;
    fG = 0;
    fB = fC;
  } else if (5 <= fHPrime && fHPrime < 6) {
    fR = fC;
    fG = 0;
    fB = fX;
  } else {
    fR = 0;
    fG = 0;
    fB = 0;
  }
  fR += fM;
  fG += fM;
  fB += fM;
}

SamplingVisualization::SamplingVisualization(
    const std::vector<MAP_PARAM> &graph_params, const MAP_PARAM &robot_param,
    const int &num_robots, const Eigen::MatrixXd &map)
    : graph_params_(graph_params),
      robot_params_(robot_param),
      map_(map),
      num_robots_(num_robots) {
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

  std_msgs::ColorRGBA default_color = std_msgs::ColorRGBA();
  default_color.r = 0.0;
  default_color.g = 0.0;
  default_color.b = 0.0;
  default_color.a = 1.0;

  for (int i = 0; i < graph_params.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "sampling_visualization";
    marker.header.stamp = ros::Time::now();
    marker.ns = "sampling_visualization";
    marker.pose.orientation.w = 0.0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.id = graph_params_[i].map_id;
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.scale.x = graph_params_[i].x_scale;
    marker.scale.y = graph_params_[i].y_scale;
    marker.scale.z = 1.0;
    marker.points.resize(map.rows());
    marker.colors.resize(map.rows());

    for (size_t j = 0; j < map.rows(); ++j) {
      geometry_msgs::Point waypoint;
      waypoint.x = (map(j, 0) - map_x_origin_) * map_x_scale_ +
                   graph_params_[i].x_offset;
      waypoint.y = (map(j, 1) - map_y_origin_) * map_y_scale_ +
                   graph_params_[i].y_offset;
      waypoint.z = 0.0;
      marker.points[j] = waypoint;
      marker.colors[j] = default_color;
    }
    graph_markers_.push_back(marker);
  }

  robot_marker_ = visualization_msgs::Marker();
  robot_marker_.header.frame_id = "sampling_visualization";
  robot_marker_.header.stamp = ros::Time::now();
  robot_marker_.ns = "sampling_visualization";
  robot_marker_.pose.orientation.w = 0.0;
  robot_marker_.action = visualization_msgs::Marker::ADD;
  robot_marker_.id = robot_params_.map_id;
  robot_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
  robot_marker_.scale.x = robot_params_.x_scale;
  robot_marker_.scale.y = robot_params_.y_scale;
  robot_marker_.scale.z = 5.0;
  robot_marker_.points.resize(num_robots_);
  robot_marker_.colors.resize(num_robots_);

  for (size_t i = 0; i < num_robots_; ++i) {
    switch (i) {
      case 0: {
        robot_marker_.colors[i].r = KRGBRed[0];
        robot_marker_.colors[i].g = KRGBRed[1];
        robot_marker_.colors[i].b = KRGBRed[2];
        robot_marker_.colors[i].a = 1.0;
        break;
      }
      case 1: {
        robot_marker_.colors[i].r = KRGBGreen[0];
        robot_marker_.colors[i].g = KRGBGreen[1];
        robot_marker_.colors[i].b = KRGBGreen[2];
        robot_marker_.colors[i].a = 1.0;
        break;
      }
      case 2: {
        robot_marker_.colors[i].r = KRGBBlue[0];
        robot_marker_.colors[i].g = KRGBBlue[1];
        robot_marker_.colors[i].b = KRGBBlue[2];
        robot_marker_.colors[i].a = 1.0;
        break;
      }
      case 3: {
        robot_marker_.colors[i].r = KRGBYellow[0];
        robot_marker_.colors[i].g = KRGBYellow[1];
        robot_marker_.colors[i].b = KRGBYellow[2];
        robot_marker_.colors[i].a = 1.0;
        break;
      }
      case 4: {
        robot_marker_.colors[i].r = KRGBGray[0];
        robot_marker_.colors[i].g = KRGBGray[1];
        robot_marker_.colors[i].b = KRGBGray[2];
        robot_marker_.colors[i].a = 1.0;
        break;
      }
      case 5: {
        robot_marker_.colors[i].r = KRGBPink[0];
        robot_marker_.colors[i].g = KRGBPink[1];
        robot_marker_.colors[i].b = KRGBPink[2];
        robot_marker_.colors[i].a = 1.0;
        break;
      }
      default: {
        robot_marker_.colors[i].r = 0.0;
        robot_marker_.colors[i].g = 0.0;
        robot_marker_.colors[i].b = 0.0;
        robot_marker_.colors[i].a = 1.0;
        break;
      }
    }
  }
}

std_msgs::ColorRGBA SamplingVisualization::GetHeatMapColor(const double &norm) {
  std_msgs::ColorRGBA color;
  double r, g, b;
  HSVtoRGB((1 - norm) * 100, 1.0, 1.0, r, g, b);
  // HSVtoRGB(norm * 120 + 240, 1.0, 1.0, r, g, b);
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = 1.0;
  return color;
}

visualization_msgs::Marker SamplingVisualization::UpdateMap(
    const int &map_id, const Eigen::VectorXd &filling_value) {
  if (map_.rows() != filling_value.size()) {
    ROS_ERROR_STREAM("Map size does not match filling value for map : "
                     << graph_params_[map_id].map_frame);
    ROS_ERROR_STREAM("Map size : " << map_.rows());
    ROS_ERROR_STREAM("Filling value size : " << filling_value.size());
    return visualization_msgs::Marker();
  }

  graph_markers_[map_id].header.stamp = ros::Time::now();

  double lower_bound =
      std::min(graph_params_[map_id].lower_bound, filling_value.minCoeff());
  double upper_bound =
      std::max(graph_params_[map_id].upper_bound, filling_value.maxCoeff());

  for (size_t i = 0; i < filling_value.rows(); ++i) {
    double norm =
        (filling_value(i) - lower_bound) / (upper_bound - lower_bound);
    graph_markers_[map_id].colors[i] = GetHeatMapColor(norm);
  }
  return graph_markers_[map_id];
}

visualization_msgs::MarkerArray SamplingVisualization::UpdateMap(
    const std::vector<Eigen::VectorXd> &filling_values) {
  visualization_msgs::MarkerArray marker_array;
  for (int i = 0; i < filling_values.size(); ++i) {
    if (map_.rows() != filling_values[i].size()) {
      ROS_ERROR_STREAM("Map size does not match filling value for map : "
                       << graph_params_[i].map_frame);
      ROS_ERROR_STREAM("Map size : " << map_.rows());
      ROS_ERROR_STREAM("Filling value size : " << filling_values.size());
      return marker_array;
    }

    graph_markers_[i].header.stamp = ros::Time::now();

    double lower_bound =
        std::min(graph_params_[i].lower_bound, filling_values[i].minCoeff());
    double upper_bound =
        std::max(graph_params_[i].upper_bound, filling_values[i].maxCoeff());

    for (size_t j = 0; j < filling_values[i].rows(); ++j) {
      double norm =
          (filling_values[i](j) - lower_bound) / (upper_bound - lower_bound);
      graph_markers_[i].colors[j] = GetHeatMapColor(norm);
    }
    marker_array.markers.push_back(graph_markers_[i]);
  }
  return marker_array;
}

visualization_msgs::Marker SamplingVisualization::UpdateRobot(
    const Eigen::MatrixXd &robot_locations) {
  assert(robot_locations.rows() == num_robots_);
  for (int i = 0; i < num_robots_; ++i) {
    geometry_msgs::Point waypoint;
    waypoint.x = (robot_locations(i, 0) - map_x_origin_) * map_x_scale_;
    waypoint.y = (robot_locations(i, 1) - map_y_origin_) * map_y_scale_;
    waypoint.z = 2.5;
    robot_marker_.points[i] = waypoint;
    // ROS_INFO_STREAM("ROBOT LOCATION : " << waypoint.x << " " << waypoint.y);
  }
  return robot_marker_;
}

}  // namespace visualization
}  // namespace sampling