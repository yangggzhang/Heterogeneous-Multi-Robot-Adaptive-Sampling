#include "sampling_partition/weighted_voronoi_partition_params.h"

#include <ros/ros.h>

#include "sampling_partition/heterogeneity.h"
#include "sampling_utils/utils.h"

namespace sampling {
namespace partition {

WeightedVoronoiPartitionParam::WeightedVoronoiPartitionParam() {}

bool WeightedVoronoiPartitionParam::LoadFromXML(
    const XmlRpc::XmlRpcValue& param) {
  if (!utils::GetParam(param, "heterogenities", heterogenities)) {
    ROS_ERROR_STREAM(
        "Error loading heterogeneities for heterogeneous property!");
    return false;
  }

  if (!utils::GetParam(param, "weight_factor", weight_factor) ||
      weight_factor.size() != heterogenities.size()) {
    ROS_ERROR_STREAM(
        "Error loading weight factors for heterogeneous property!");
    return false;
  }
  return true;
}

}  // namespace partition
}  // namespace sampling