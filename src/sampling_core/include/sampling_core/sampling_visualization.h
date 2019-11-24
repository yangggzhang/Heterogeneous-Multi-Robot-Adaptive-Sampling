#pragma once
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <unordered_set>

namespace sampling {
namespace visualization {

const int K_NUM_COLOR = 5;

const double K_COLOR[K_NUM_COLOR][3] = {
    {0, 0, 1}, {0, 1, 1}, {1, 1, 0}, {1, 0, 0}, {0.6, 0, 0}};

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

  // bool LoadMapParam(XmlRpc::XmlRpcValue &YamlNode, MAP_PARAM &param);

  SamplingVisualization(ros::NodeHandle &nh, const MAP_PARAM &param,
                        const Eigen::MatrixXd &map);

  void update_map(const Eigen::VectorXd &filling_value);

 private:
  MAP_PARAM param_;

  visualization_msgs::Marker marker_array_;

  Eigen::MatrixXd map_;

  ros::Publisher map_visualization_pub_;

  ros::WallTimer timer_;

  // this function gets the color for each pixel given the normalized value
  // of the pixel
  std_msgs::ColorRGBA get_heatmap_color(const double &norm);

  void MapVisualizationCallback(const ros::WallTimerEvent &event);
};
}  // namespace visualization
}  // namespace sampling