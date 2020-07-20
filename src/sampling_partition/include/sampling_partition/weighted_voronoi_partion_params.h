#pragma once

#include <ros/ros.h>

#include <string>
#include <unordered_set>

namespace sampling {
namespace partition {

class WeightedVoronoiPartitionParam {
 public:
  WeightedVoronoiPartitionParam();

  bool LoadFromXML(const XmlRpc::XmlRpcValue& param);

  std::unordered_set<std::string> agent_ids;

  std::vector<std::string> heterogenities;

  std::vector<double> weight_factor;
};
}  // namespace partition
}  // namespace sampling