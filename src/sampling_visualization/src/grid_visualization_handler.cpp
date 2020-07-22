#include "sampling_visualization/grid_visualization_handler.h"

#include "sampling_visualization/sampling_visualization_utils.h"

namespace sampling {
namespace visualization {

std::unique_ptr<GridVisualizationHandler>
GridVisualizationHandler::MakeUniqueFromXML(ros::NodeHandle &nh,
                                            const XmlRpc::XmlRpcValue &param) {
  return nullptr;
}

GridVisualizationHandler::GridVisualizationHandler(
    ros::NodeHandle &nh, const visualization_msgs::Marker &marker,
    const SamplingVisualizationParams &params)
    : marker_(marker), params_(params) {
  grid_publisher_ = nh.advertise<visualization_msgs::Marker>(
      KVisualizationNamespace + params.name, 1);

  event_timer_ = nh.createTimer(
      ros::Duration(1.0 / KVisualizationUpdateRate_hz),
      &GridVisualizationHandler::UpdateVisualizationCallback, this);
}

bool GridVisualizationHandler::UpdateMarker(
    const std::vector<double> &marker_value) {
  if (KVisualizationType_Grid.compare(params_.visualization_type) != 0) {
    ROS_ERROR_STREAM("Wrong data type for visualization update!");
    return false;
  }
  return true;
}

bool GridVisualizationHandler::UpdateMarker(
    const std::vector<int> &marker_value) {
  if (KVisualizationType_Partition.compare(params_.visualization_type) != 0) {
    ROS_ERROR_STREAM("Wrong data type for visualization update!");
    return false;
  }
  return true;
}

void GridVisualizationHandler::UpdateVisualizationCallback(
    const ros::TimerEvent &) {
  grid_publisher_.publish(marker_);
}
}  // namespace visualization
}  // namespace sampling