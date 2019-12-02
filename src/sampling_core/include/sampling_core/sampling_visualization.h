#pragma once
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <unordered_set>

namespace sampling {
namespace visualization {

struct MAP_PARAM {
  std::string map_frame;
  int map_id;
  double x_scale;
  double y_scale;
  double x_offset;
  double y_offset;
  double lower_bound;
  double upper_bound;
};

class SamplingVisualization {
 public:
  SamplingVisualization();

  SamplingVisualization(ros::NodeHandle &nh, const MAP_PARAM &param,
                        const Eigen::MatrixXd &map);

  void UpdateMap(const Eigen::VectorXd &filling_value);

  void HSVtoRGB(const double &fH, const double &fS, const double &fV,
                double &fR, double &fG, double &fB);

  visualization_msgs::Marker GetMarker();

 private:
  MAP_PARAM param_;

  visualization_msgs::Marker marker_array_;

  Eigen::MatrixXd map_;

  // this function gets the color for each pixel given the normalized value
  // of the pixel
  std_msgs::ColorRGBA GetHeatMapColor(const double &norm);
};

class RobotVisualization {
 public:
  RobotVisualization();

  RobotVisualization(ros::NodeHandle &nh, const MAP_PARAM &param,
                     const std_msgs::ColorRGBA &color,
                     const Eigen::MatrixXd &map);

  void UpdateMap(const double &robot_x, const double &robot_y);

  visualization_msgs::Marker GetMarker();

 private:
  MAP_PARAM param_;

  double map_x_origin_;
  double map_y_origin_;
  double map_x_scale_;
  double map_y_scale_;

  visualization_msgs::Marker marker_;
};
}  // namespace visualization
}  // namespace sampling