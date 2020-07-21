#include "sampling_visualization/sampling_visualization_params.h"

#include "sampling_utils/utils.h"

namespace sampling {
namespace visualization {

SamplingVisualizationParams::SamplingVisualizationParams() {}

bool SamplingVisualizationParams::LoadFromXML(
    const XmlRpc::XmlRpcValue& yaml_node) {
  if (!utils::GetParam(yaml_node, "name", name)) {
    ROS_ERROR_STREAM("Error loading name for sampling visualization!");
    return false;
  }

  if (!utils::GetParam(yaml_node, "visualization_type", visualization_type)) {
    ROS_ERROR_STREAM(
        "Error loading visualization type for sampling visualization!");
    return false;
  } else if (KVisualizationType_Grid.compare(visualization_type) != 0 &&
             KVisualizationType_Location.compare(visualization_type) != 0 &&
             KVisualizationType_Partition.compare(visualization_type) != 0) {
    ROS_ERROR_STREAM("Unknow visualization type!");
    return false;
  }
  if (!utils::GetParam(yaml_node, "offset", offset) ||
      offset.size() != KVisualizationDimension) {
    ROS_ERROR_STREAM("Error loading offset for sampling visualization!");
    return false;
  }

  if (!utils::GetParam(yaml_node, "scale", scale) ||
      scale.size() != KVisualizationDimension) {
    ROS_ERROR_STREAM("Error loading scale for sampling visualization!");
    return false;
  }

  if (!utils::GetParam(yaml_node, "bounds", bounds) ||
      bounds.size() != KVisualizationDimension) {
    ROS_WARN_STREAM("Error loading scale for sampling visualization!");
    bounds =
        std::vector<double>{KVisualizationLowerBound, KVisualizationUpperBound};
  }

  return true;
}

}  // namespace visualization
}  // namespace sampling