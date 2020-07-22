#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Dense>

#include "sampling_visualization/sampling_visualization_params.h"

namespace sampling {
namespace visualization {

class GridVisualizationHandler {
 public:
  GridVisualizationHandler() = delete;

  static std::unique_ptr<GridVisualizationHandler> MakeUniqueFromXML(
      ros::NodeHandle &nh, const XmlRpc::XmlRpcValue &yaml_node,
      const Eigen::MatrixXd &map);

  bool UpdateMarker(const std::vector<double> &marker_value);

  bool UpdateMarker(const std::vector<int> &marker_value);

 private:
  GridVisualizationHandler(ros::NodeHandle &nh,
                           const visualization_msgs::Marker &marker,
                           const SamplingVisualizationParams &params);

  SamplingVisualizationParams params_;

  ros::Timer event_timer_;

  ros::Publisher grid_publisher_;

  visualization_msgs::Marker marker_;

  void UpdateVisualizationCallback(const ros::TimerEvent &);
};
}  // namespace visualization
}  // namespace sampling