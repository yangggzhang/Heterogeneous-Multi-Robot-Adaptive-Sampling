#pragma once

#include <ros/ros.h>

#include <boost/optional.hpp>
#include <unordered_map>

#include "sampling_core/sampling_core_params.h"
#include "sampling_msgs/AddSampleToModel.h"
#include "sampling_msgs/AgentLocation.h"
#include "sampling_msgs/Sample.h"
#include "sampling_msgs/SamplingGoal.h"
#include "sampling_online_learning/online_learning_handler.h"
#include "sampling_partition/weighted_voronoi_partition.h"
#include "sampling_visualization/agent_visualization_handler.h"
#include "sampling_visualization/grid_visualization_handler.h"

namespace sampling {
namespace core {

// todo agent die

const std::string KModelingNamespace = "modeling/";

class SamplingCore {
 public:
  SamplingCore() = delete;

  static std::unique_ptr<SamplingCore> MakeUniqueFromRos(ros::NodeHandle &nh,
                                                         ros::NodeHandle &ph);

  bool Loop();

 private:
  SamplingCore(
      ros::NodeHandle &nh, const SamplingCoreParams &params,
      std::unique_ptr<partition::WeightedVoronoiPartition> partition_handler,
      std::unique_ptr<learning::OnlineLearningHandler> learning_handler,
      std::unique_ptr<visualization::AgentVisualizationHandler>
          agent_visualization_handler,
      std::vector<std::unique_ptr<visualization::GridVisualizationHandler>>
          &grid_visualization_handlers);

  SamplingCoreParams params_;

  // Agent
  ros::Subscriber agent_location_subscriber_;

  ros::Subscriber sample_subscriber_;

  std::unordered_map<std::string, sampling_msgs::AgentLocation>
      agents_locations_;

  ros::ServiceClient modeling_add_test_location_client_;

  ros::ServiceClient modeling_add_sample_client_;

  ros::ServiceClient modeling_update_model_client_;

  ros::ServiceClient modeling_predict_client_;

  ros::ServiceServer sampling_goal_server_;

  std::vector<ros::ServiceClient> agent_check_clients_;

  // Partition
  std::unique_ptr<partition::WeightedVoronoiPartition> partition_handler_;

  // Online Learning
  std::unique_ptr<learning::OnlineLearningHandler> learning_handler_;

  // Visualization
  std::unique_ptr<visualization::AgentVisualizationHandler>
      agent_visualization_handler_;

  std::unordered_map<std::string,
                     std::unique_ptr<visualization::GridVisualizationHandler>>
      grid_visualization_handlers_;

  void AgentLocationUpdateCallback(
      const sampling_msgs::AgentLocationConstPtr &msg);

  std::vector<sampling_msgs::Sample> sample_buffer_;

  int new_sample_buffer_count_;

  bool SampleToSrv(const std::vector<sampling_msgs::Sample> &samples,
                   sampling_msgs::AddSampleToModel &srv);

  void SampleUpdateCallback(const sampling_msgs::SampleConstPtr &msg);

  bool InitializeModelAndPrediction();

  bool Initialize();

  bool UpdateModel();

  bool UpdatePrediction();

  bool UpdateVisualization();

  bool AssignSamplingGoal(sampling_msgs::SamplingGoal::Request &req,
                          sampling_msgs::SamplingGoal::Response &res);

  boost::optional<std::vector<double>> updated_mean_prediction_;

  boost::optional<std::vector<double>> updated_var_prediction_;

  bool is_initialized_;
};
}  // namespace core
}  // namespace sampling
