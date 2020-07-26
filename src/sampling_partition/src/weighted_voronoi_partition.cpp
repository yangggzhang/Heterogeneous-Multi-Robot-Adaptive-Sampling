#include "sampling_partition/weighted_voronoi_partition.h"

#include "sampling_partition/heterogeneity_distance.h"
#include "sampling_partition/heterogeneity_distance_dependent.h"
#include "sampling_partition/heterogeneity_topography_dependent.h"
#include "sampling_utils/utils.h"

namespace sampling {
namespace partition {

std::unique_ptr<WeightedVoronoiPartition>
WeightedVoronoiPartition::MakeUniqueFromRosParam(
    const std::vector<std::string> &agent_ids, const Eigen::MatrixXd &map,
    ros::NodeHandle &ph) {
  XmlRpc::XmlRpcValue heterogeneity_param_list;
  if (!ph.getParam("HeterogeneousProperty", heterogeneity_param_list)) {
    ROS_ERROR("Missing heterogeneous properties!");
    return nullptr;
  }

  XmlRpc::XmlRpcValue heterogeneous_param_yaml_node =
      heterogeneity_param_list[0];
  WeightedVoronoiPartitionParam partiton_params;
  if (!partiton_params.LoadFromXML(heterogeneous_param_yaml_node)) {
    ROS_ERROR("Error loading partition params!");
    return nullptr;
  }

  partiton_params.agent_ids =
      std::unordered_set<std::string>(agent_ids.begin(), agent_ids.end());

  std::unordered_map<std::string, std::vector<HeterogeneityParams>>
      heterogeneity_param_map;

  for (int i = 1; i < heterogeneity_param_list.size(); ++i) {
    XmlRpc::XmlRpcValue agent_heterogeneity_yaml_node =
        heterogeneity_param_list[i];
    std::string agent_id;
    if (!utils::GetParam(agent_heterogeneity_yaml_node, "agent_id", agent_id)) {
      ROS_ERROR_STREAM("Missing agent id for heterogeneous property");
      return nullptr;
    }
    if (!partiton_params.agent_ids.count(agent_id)) {
      ROS_ERROR_STREAM("Agent id does NOT match the agents launched!");
      return nullptr;
    }

    std::vector<double> heterogeneity_primitive;
    if (!utils::GetParam(agent_heterogeneity_yaml_node,
                         "heterogeneity_primitive", heterogeneity_primitive) ||
        heterogeneity_primitive.size() !=
            partiton_params.heterogenities.size()) {
      ROS_ERROR_STREAM(
          "Error loading heterogeneity primitive for heterogeneous property "
          "for "
          "agent :"
          << agent_id);
      return nullptr;
    }

    int number_control_area;
    if (!utils::GetParam(agent_heterogeneity_yaml_node, "number_control_area",
                         number_control_area)) {
      ROS_ERROR_STREAM(
          "Missing number of control area for heterogeneous property for agent "
          ":"
          << agent_id);
      return nullptr;
    }
    std::vector<double> control_area_radius;
    std::vector<geometry_msgs::Point> control_area_center;
    for (int j = 0; j < number_control_area; ++j) {
      std::vector<double> point;
      double radius;
      const std::string point_param_name =
          "control_area_center_" + std::to_string(j);
      const std::string radius_param_name =
          "control_area_radius_" + std::to_string(j);

      if (!utils::GetParam(agent_heterogeneity_yaml_node, point_param_name,
                           point) ||
          point.size() < 2 ||
          !utils::GetParam(agent_heterogeneity_yaml_node, radius_param_name,
                           radius)) {
        ROS_ERROR_STREAM(
            "Missing traversability information for agent :" << agent_id);
        return nullptr;
      }
      geometry_msgs::Point point_ros;
      point_ros.x = point[0];
      point_ros.y = point[1];
      control_area_center.push_back(point_ros);
      control_area_radius.push_back(radius);
    }

    for (int j = 0; j < heterogeneity_primitive.size(); ++j) {
      HeterogeneityParams params;
      params.heterogeneity_type = partiton_params.heterogenities[j];
      params.heterogeneity_primitive = heterogeneity_primitive[j];
      params.control_area_center = control_area_center;
      params.control_area_radius = control_area_radius;
      std::unique_ptr<Heterogeneity> hetero_ptr = nullptr;
      if (KHomogeneityDistance.compare(params.heterogeneity_type) == 0)
        hetero_ptr = std::make_unique<HeterogeneityDistance>(params, map);
      else if (KHeterogeneitySpeed.compare(params.heterogeneity_type) == 0)
        hetero_ptr =
            std::make_unique<HeterogeneityDistanceDepedent>(params, map);
      else if (KHeterogeneityBatteryLife.compare(params.heterogeneity_type) ==
               0)
        hetero_ptr =
            std::make_unique<HeterogeneityDistanceDepedent>(params, map);
      else if (KHeterogeneityTraversability.compare(
                   params.heterogeneity_type) == 0)
        hetero_ptr =
            std::make_unique<HeterogeneityTopographyDepedent>(params, map);

      if (hetero_ptr == nullptr) {
        ROS_ERROR_STREAM("Error information of heterogeneity : "
                         << params.heterogeneity_type
                         << " for agent : " << agent_id);
        return nullptr;
      }
      heterogeneity_param_map[agent_id].push_back(params);
    }
  }
  return std::unique_ptr<WeightedVoronoiPartition>(new WeightedVoronoiPartition(
      partiton_params, heterogeneity_param_map, map));
}

bool WeightedVoronoiPartition::ComputePartitionForAgent(
    const std::string &agent_id,
    const std::vector<sampling_msgs::AgentLocation> &location,
    std::vector<int> &partition_index) {
  partition_index.clear();
  Eigen::MatrixXd cost_map =
      Eigen::MatrixXd::Zero(map_.rows(), location.size());
  for (int i = 0; i < location.size(); ++i) {
    sampling_msgs::AgentLocation agent_info = location.at(i);
    if (!heterogeneity_map_.count(agent_info.agent_id)) {
      ROS_ERROR_STREAM(
          "Failed to do partition for unknown agent : " << agent_info.agent_id);
      return false;
    }
    const Eigen::VectorXd distance =
        CalculateEuclideanDistance(agent_info.position, map_);
    for (int j = 0; j < heterogeneity_map_[agent_info.agent_id].size(); ++j) {
      Eigen::VectorXd heterogeneity_cost =
          params_.weight_factor[j] *
          heterogeneity_map_[agent_info.agent_id][j]->CalculateCost(
              agent_info.position, distance);
      cost_map.col(i).array() =
          cost_map.col(i).array() + heterogeneity_cost.array();
    }
  }
  for (int i = 0; i < map_.rows(); ++i) {
    Eigen::MatrixXd::Index index;
    cost_map.row(i).minCoeff(&index);
    if (agent_id.compare(location[(int)index].agent_id) == 0)
      partition_index.push_back((int)index);
  }
  return true;
}

bool WeightedVoronoiPartition::ComputePartitionForMap(
    const std::vector<sampling_msgs::AgentLocation> &location,
    std::vector<int> &index_for_map) {
  index_for_map.clear();
  Eigen::MatrixXd cost_map =
      Eigen::MatrixXd::Zero(map_.rows(), location.size());
  for (int i = 0; i < location.size(); ++i) {
    sampling_msgs::AgentLocation agent_info = location.at(i);
    if (!heterogeneity_map_.count(agent_info.agent_id)) {
      ROS_ERROR_STREAM(
          "Failed to do partition for unknown agent : " << agent_info.agent_id);
      return false;
    }
    const Eigen::VectorXd distance =
        CalculateEuclideanDistance(agent_info.position, map_);
    for (int j = 0; j < heterogeneity_map_[agent_info.agent_id].size(); ++j) {
      Eigen::VectorXd heterogeneity_cost =
          params_.weight_factor[j] *
          heterogeneity_map_[agent_info.agent_id][j]->CalculateCost(
              agent_info.position, distance);
      cost_map.col(i).array() =
          cost_map.col(i).array() + heterogeneity_cost.array();
    }
  }
  index_for_map.resize(map_.rows());
  for (int i = 0; i < map_.rows(); ++i) {
    Eigen::MatrixXd::Index index;
    cost_map.row(i).minCoeff(&index);
    index_for_map[i] = (int)index;
  }
  return true;
}

WeightedVoronoiPartition::WeightedVoronoiPartition(
    const WeightedVoronoiPartitionParam &params,
    const std::unordered_map<std::string, std::vector<HeterogeneityParams>>
        &heterogeneity_param_map,
    const Eigen::MatrixXd &map)
    : params_(params), map_(map) {
  heterogeneity_map_.clear();
  for (auto it = heterogeneity_param_map.begin();
       it != heterogeneity_param_map.end(); ++it) {
    heterogeneity_map_[it->first].reserve(it->second.size());
    for (const auto &param : it->second) {
      if (KHomogeneityDistance.compare(param.heterogeneity_type) == 0)
        heterogeneity_map_[it->first].push_back(
            std::make_unique<HeterogeneityDistance>(param, map));
      else if (KHeterogeneitySpeed.compare(param.heterogeneity_type) == 0)
        heterogeneity_map_[it->first].push_back(
            std::make_unique<HeterogeneityDistanceDepedent>(param, map));
      else if (KHeterogeneityBatteryLife.compare(param.heterogeneity_type) == 0)
        heterogeneity_map_[it->first].push_back(
            std::make_unique<HeterogeneityDistanceDepedent>(param, map));
      else if (KHeterogeneityTraversability.compare(param.heterogeneity_type) ==
               0)
        heterogeneity_map_[it->first].push_back(
            std::make_unique<HeterogeneityTopographyDepedent>(param, map));
    }
  }
}

Eigen::VectorXd WeightedVoronoiPartition::CalculateEuclideanDistance(
    const geometry_msgs::Point &point, const Eigen::MatrixXd &map) {
  Eigen::MatrixXd distance_map(map.rows(), map.cols());
  distance_map.col(0).array() = map.col(0).array() - point.x;
  distance_map.col(1).array() = map.col(1).array() - point.y;
  return distance_map.rowwise().norm();
}

}  // namespace partition
}  // namespace sampling