#include "sampling_visualization/sampling_visualization_params.h"

#include "sampling_utils/utils.h"

namespace sampling {
namespace visualization {

SamplingVisualizationParams::SamplingVisualizationParams() {}

bool SamplingVisualizationParams::LoadFromXML(
    const XmlRpc::XmlRpcValue& param) {
  if (!utils::GetParam(param, "name", name)) {
    ROS_ERROR_STREAM("Error loading name for sampling visualization!");
    return false;
  }

  if (!utils::GetParam(param, "offset", offset) ||
      offset.size() != KVisualizationDimension) {
    ROS_ERROR_STREAM("Error loading offset for sampling visualization!");
    return false;
  }

  if (!utils::GetParam(param, "scale", scale) ||
      scale.size() != KVisualizationDimension) {
    ROS_ERROR_STREAM("Error loading scale for sampling visualization!");
    return false;
  }

  if (!utils::GetParam(param, "bounds", bounds) ||
      bounds.size() != KVisualizationDimension) {
    ROS_WARN_STREAM("Error loading scale for sampling visualization!");
    bounds =
        std::vector<double>{KVisualizationLowerBound, KVisualizationUpperBound};
  }

  return true;
}

}  // namespace visualization
}  // namespace sampling