#include "sampling_core/sampling_visualization.h"
#include "sampling_msgs/RequestLocation.h"

namespace sampling {
namespace visualization {
SamplingVisualization::SamplingVisualization() {}

SamplingVisualization::SamplingVisualization(ros::NodeHandle &nh,
                                             const MAP_PARAM &param,
                                             const Eigen::MatrixXd &map)
    : param_(param), map_(map) {
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
  color.r = 1.0;
  color.g = 0.0;
  color.b = 0.0;
  color.a = 1.0;

  for (size_t i = 0; i < map.rows(); ++i) {
    geometry_msgs::Point waypoint;
    waypoint.x = (map(i, 0) - map_x_origin) * map_x_scale;
    waypoint.y = (map(i, 1) - map_y_origin) * map_y_scale;
    waypoint.z = 0.0;
    waypoint.x = waypoint.x + param.x_offset;
    waypoint.y = waypoint.y + param.y_offset;
    marker_array_.points[i] = waypoint;
    marker_array_.colors[i] = color;
  }
}

void SamplingVisualization::UpdateMap(const Eigen::VectorXd &filling_value) {
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
    marker_array_.colors[i] = GetHeatMapColor(norm);
  }
}

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

visualization_msgs::Marker SamplingVisualization::GetMarker() {
  return marker_array_;
}

RobotVisualization::RobotVisualization() {}

RobotVisualization::RobotVisualization(ros::NodeHandle &nh,
                                       const MAP_PARAM &param,
                                       const std_msgs::ColorRGBA &color,
                                       const Eigen::MatrixXd &map)
    : param_(param) {
  marker_ = visualization_msgs::Marker();
  marker_.header.frame_id = "sampling_visualization";
  marker_.header.stamp = ros::Time::now();
  marker_.ns = "sampling_visualization";
  marker_.pose.orientation.w = 0.0;
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.id = param.map_id;
  marker_.type = visualization_msgs::Marker::SPHERE;
  marker_.scale.x = param.x_scale;
  marker_.scale.y = param.y_scale;
  marker_.scale.z = 1.0;

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

  marker_.color = color;

  target_ = marker_;
  target_.id = target_.id++;
  target_.type = visualization_msgs::Marker::CUBE;
}

void RobotVisualization::UpdateMap(const double &robot_x,
                                   const double &robot_y) {
  marker_.pose.position.x =
      ((robot_x - map_x_origin_) * map_x_scale_) + param_.x_offset;
  marker_.pose.position.y =
      ((robot_y - map_y_origin_) * map_y_scale_) + param_.y_offset;
  marker_.pose.position.z = 0.5;
}

void RobotVisualization::UpdateTarget(const double &target_x,
                                      const double &target_y) {
  marker_.pose.position.x =
      ((target_x - map_x_origin_) * map_x_scale_) + param_.x_offset;
  marker_.pose.position.y =
      ((target_y - map_y_origin_) * map_y_scale_) + param_.y_offset;
  marker_.pose.position.z = 0.5;
}

visualization_msgs::Marker RobotVisualization::GetMarker() { return marker_; }

visualization_msgs::Marker RobotVisualization::GetTarget() { return marker_; }

}  // namespace visualization
}  // namespace sampling