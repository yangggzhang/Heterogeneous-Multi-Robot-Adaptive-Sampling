#include <ros/package.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>

#include "sampling_msgs/AgentLocation.h"
#include "sampling_partition/weighted_voronoi_partition.h"
#include "sampling_utils/utils.h"
#include "sampling_visualization/agent_visualization_handler.h"
#include "sampling_visualization/grid_visualization_handler.h"

namespace sampling {
namespace partition {

bool LoadMap(const std::string &path, Eigen::MatrixXd &data) {
  std::ifstream file(path.c_str(), std::ifstream::in);

  if (!file.is_open()) {
    ROS_INFO_STREAM("Error opening file " << path);
    return false;
  }

  std::vector<std::vector<double>> data_vec;

  std::string new_line;

  while (getline(file, new_line)) {
    std::stringstream ss(new_line);
    std::vector<double> new_data;
    for (double k; ss >> k;) {
      new_data.push_back(k);
      if (ss.peek() == ',') ss.ignore();
    }
    data_vec.push_back(new_data);
  }

  if (data_vec.empty()) {
    ROS_ERROR_STREAM("Empty data!");
    return false;
  }

  data.resize(data_vec.size(), data_vec.front().size());
  for (int i = 0; i < data_vec.size(); ++i) {
    for (int j = 0; j < data_vec[i].size(); ++j) {
      data(i, j) = data_vec[i][j];
    }
  }
  return true;
}

class PartitionNode {
 public:
  PartitionNode() = delete;

  static std::unique_ptr<PartitionNode> MakeUniqueFromRos(ros::NodeHandle &nh,
                                                          ros::NodeHandle &ph) {
    std::string test_map_file;
    if (!ph.getParam("test_map_file", test_map_file)) {
      ROS_ERROR_STREAM("Please provide test map file for partition!");
      return nullptr;
    }

    std::string pack_path = ros::package::getPath("sampling_partition");
    std::string test_map_dir = pack_path + "/map/" + test_map_file;
    Eigen::MatrixXd map;

    if (!LoadMap(test_map_dir, map)) {
      ROS_ERROR_STREAM("Failed to load test map for partition!");
      return nullptr;
    }

    std::vector<std::string> agent_ids;
    if (!ph.getParam("agent_ids", agent_ids)) {
      ROS_ERROR_STREAM("Please provide agent ids for partition!");
      return nullptr;
    }

    XmlRpc::XmlRpcValue agent_locations_list;
    if (!ph.getParam("AgentLocations", agent_locations_list)) {
      ROS_ERROR("Missing agent locations for partition!");
      return nullptr;
    }

    if (agent_locations_list.size() != agent_ids.size()) {
      ROS_ERROR("Location size does not match the number of agents!");
      return nullptr;
    }

    std::vector<sampling_msgs::AgentLocation> locations;
    for (int i = 0; i < agent_locations_list.size(); ++i) {
      double x, y;
      if (!utils::GetParam(agent_locations_list[i], "x", x)) {
        ROS_ERROR_STREAM("Please provide agent location x!");
        return nullptr;
      }
      if (!utils::GetParam(agent_locations_list[i], "y", y)) {
        ROS_ERROR_STREAM("Please provide agent location y!");
        return nullptr;
      }
      sampling_msgs::AgentLocation msg;
      msg.agent_id = agent_ids[i];
      msg.position.x = x;
      msg.position.y = y;
      locations.push_back(msg);
    }

    std::unique_ptr<WeightedVoronoiPartition> partition_ptr =
        WeightedVoronoiPartition::MakeUniqueFromRosParam(agent_ids, map, ph);
    if (partition_ptr == nullptr) {
      ROS_ERROR_STREAM("Failed to create sampling patition handler!");
      return nullptr;
    }

    std::unique_ptr<visualization::AgentVisualizationHandler>
        agent_visualization_handler = nullptr;

    std::unique_ptr<visualization::GridVisualizationHandler>
        partition_visualization_handler = nullptr;

    XmlRpc::XmlRpcValue visualization_param_list;
    if (!ph.getParam("VisualizationProperty", visualization_param_list) ||
        visualization_param_list.size() != 2) {
      ROS_ERROR_STREAM(
          "Please provide corrent visualization property for partition!");
      return nullptr;
    } else {
      for (int i = 0; i < visualization_param_list.size(); ++i) {
        XmlRpc::XmlRpcValue yaml_node = visualization_param_list[i];
        std::string visualization_type;
        if (!utils::GetParam(yaml_node, "visualization_type",
                             visualization_type)) {
          return nullptr;
        } else {
          if (visualization::KVisualizationType_Location.compare(
                  visualization_type) == 0) {
            agent_visualization_handler =
                visualization::AgentVisualizationHandler::MakeUniqueFromXML(
                    nh, yaml_node, int(agent_ids.size()), map);
          } else if (visualization::KVisualizationType_Partition.compare(
                         visualization_type) == 0) {
            partition_visualization_handler =
                visualization::GridVisualizationHandler::MakeUniqueFromXML(
                    nh, yaml_node, map);
            if (partition_visualization_handler == nullptr) return nullptr;
          } else {
            ROS_ERROR_STREAM("Unkown visualization type");
            return nullptr;
          }
        }
      }
    }

    return std::unique_ptr<PartitionNode>(
        new PartitionNode(map, locations, std::move(partition_ptr),
                          std::move(agent_visualization_handler),
                          std::move(partition_visualization_handler)));
  }

  bool ComputePartition() {
    std::vector<int> partition_index;
    if (!partition_handler_->ComputePartitionForMap(locations_,
                                                    partition_index)) {
      ROS_ERROR_STREAM("Failed to generate map partition!");
      return false;
    }
    if (!agent_visualization_handler_->UpdateMarker(locations_)) {
      ROS_ERROR_STREAM("Failed to update agent location visualization!");
      return false;
    }
    if (!partition_visualization_handler_->UpdateMarker(partition_index)) {
      ROS_ERROR_STREAM("Failed to update map partition visualization!");
      return false;
    }
    return true;
  }

 private:
  PartitionNode(
      const Eigen::MatrixXd &map,
      const std::vector<sampling_msgs::AgentLocation> &locations,
      std::unique_ptr<partition::WeightedVoronoiPartition> partition_handler,
      std::unique_ptr<visualization::AgentVisualizationHandler>
          agent_visualization_handler,
      std::unique_ptr<visualization::GridVisualizationHandler>
          partition_visualization_handler)
      : map_(map),
        locations_(locations),
        partition_handler_(std::move(partition_handler)),
        agent_visualization_handler_(std::move(agent_visualization_handler)),
        partition_visualization_handler_(
            std::move(partition_visualization_handler)) {}

  Eigen::MatrixXd map_;

  std::vector<sampling_msgs::AgentLocation> locations_;

  // Partition
  std::unique_ptr<partition::WeightedVoronoiPartition> partition_handler_;

  // Visualization
  std::unique_ptr<visualization::AgentVisualizationHandler>
      agent_visualization_handler_;

  std::unique_ptr<visualization::GridVisualizationHandler>
      partition_visualization_handler_;
};

}  // namespace partition
}  // namespace sampling

int main(int argc, char **argv) {
  ros::init(argc, argv, "partition");
  ros::NodeHandle nh, ph("~");
  std::unique_ptr<sampling::partition::PartitionNode> partition_node =
      sampling::partition::PartitionNode::MakeUniqueFromRos(nh, ph);
  if (partition_node == nullptr) {
    ROS_ERROR_STREAM("Failed to launch partition node!");
    return -1;
  }
  if (!partition_node->ComputePartition()) {
    ROS_ERROR_STREAM("Failed to generate partition!");
    return -1;
  }
  ros::Rate loop_rate(2);
  while (ros::ok()) {
    ros::spin();
    loop_rate.sleep();
  }
  return 0;
}