#pragma once
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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

  SamplingVisualization(const std::vector<MAP_PARAM> &graph_params,
                        const MAP_PARAM &robot_param, const int &num_robots,
                        const Eigen::MatrixXd &map);

  visualization_msgs::Marker UpdateMap(const int &map_id,
                                       const Eigen::VectorXd &filling_value);

  visualization_msgs::MarkerArray UpdateMap(
      const std::vector<Eigen::VectorXd> &filling_values);

  visualization_msgs::Marker UpdateRobot(
      const Eigen::MatrixXd &robot_locations);

 private:
  std::vector<MAP_PARAM> graph_params_;

  MAP_PARAM robot_params_;

  int num_robots_;

  std::vector<visualization_msgs::Marker> graph_markers_;

  Eigen::MatrixXd map_;

  double map_x_origin_;

  double map_y_origin_;

  double map_x_scale_;

  double map_y_scale_;

  visualization_msgs::Marker robot_marker_;

  void HSVtoRGB(const double &fH, const double &fS, const double &fV,
                double &fR, double &fG, double &fB);

  // this function gets the color for each pixel given the normalized value
  // of the pixel
  std_msgs::ColorRGBA GetHeatMapColor(const double &norm);
};

}  // namespace visualization
}  // namespace sampling